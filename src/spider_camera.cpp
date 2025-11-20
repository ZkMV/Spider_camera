/*
 * spider_camera.cpp
 *
 * Implementation file for the SpiderCamera library (v0.4.3 - C-API GPIO Fix).
 * v0.4.3: 1. Switched from gpiod.hpp (C++ wrapper) to gpiod.h (C API)
 * to fix "undefined symbol" linking errors.
 * 2. Re-implemented all GPIO logic using C functions (gpiod_chip_open, etc.)
 */

#include "spider_camera.hpp"
#include <libcamera/pixel_format.h>
#include <libcamera/formats.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cerrno>
#include <iostream>
#include <iomanip>
#include <pybind11/stl.h>
#include <unistd.h>
#include <cstring> 
#include <sstream> 

// 🎯 v0.4.3: ПЕРЕМИКАЄМОСЬ НА C API
#include <gpiod.h>

// Макроси логування
#define LOG_INFO(msg)   std::cout << "[INFO]  " << msg << std::endl
#define LOG_DEBUG(msg)  if (debug_enabled_) std::cout << "[DEBUG] " << msg << std::endl
#define LOG_WARN(msg)   std::cerr << "[WARN]  " << msg << std::endl
#define LOG_ERROR(msg)  std::cerr << "[ERROR] " << msg << std::endl


SpiderCamera::SpiderCamera() : state_(0), active_camera_id_(-1) {
    cam_mgr_ = std::make_shared<libcamera::CameraManager>();
    int ret = cam_mgr_->start();
    if (ret) {
        LOG_ERROR("Failed to start CameraManager: " << ret);
        state_ = 4;
    } else {
        LOG_INFO("CameraManager started");
        if (cam_mgr_->cameras().empty()) {
            LOG_ERROR("No cameras found");
            state_ = 4;
        }
    }
}

SpiderCamera::~SpiderCamera() {
    LOG_INFO("Shutting down...");
    if (state_ == 2) {
        streaming_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (stream_thread_) {
        if (stream_thread_->joinable()) {
            stream_thread_->detach();
        }
        stream_thread_.reset();
    }
    try {
        if (camera_ && state_ > 0) {
            camera_->stop();
        }
    } catch (...) {}
    stop();
    if (cam_mgr_) {
        cam_mgr_->stop();
    }
}

// ... (set_cam, get_cam, release_camera - БЕЗ ЗМІН) ...
void SpiderCamera::set_cam(int cam_id) {
    if (state_ != 0) {
        throw std::runtime_error("Cannot set camera while active. Call stop() first.");
    }
    if (cam_id < 0 || static_cast<size_t>(cam_id) >= cam_mgr_->cameras().size()) {
        throw std::out_of_range("Invalid camera ID: " + std::to_string(cam_id));
    }
    camera_ = cam_mgr_->cameras()[cam_id];
    active_camera_id_ = cam_id;
    LOG_INFO("Camera selected: " << camera_->id());
}

int SpiderCamera::get_cam() const {
    return active_camera_id_;
}

void SpiderCamera::release_camera() {
    if (streaming_) {
        pause();
    }
    if (camera_) {
        if (state_ > 0) {
            try {
                requests_.clear();
                allocator_.reset();
                camera_->release();
                LOG_INFO("Camera released: " << camera_->id());
            } catch (const std::exception& e) {
                LOG_ERROR("Exception during camera release: " << e.what());
            }
        }
        config_.reset();
    }
}

/**
 * @brief [v0.4.3: ОНОВЛЕНО]
 * Використовує C-API (gpiod_line_release, gpiod_chip_close).
 */
void SpiderCamera::stop() {
    if (state_ == 0) return;
    LOG_INFO("Stopping camera...");
    if (state_ == 2) {
        streaming_ = false;
        if (stream_thread_ && stream_thread_->joinable()) {
            stream_thread_->detach();
        }
        stream_thread_.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    try {
        std::atomic<bool> stop_done{false};
        std::thread stop_thread([this, &stop_done]() {
            try {
                camera_->stop();
                stop_done = true;
            } catch (...) {}
        });
        auto start = std::chrono::steady_clock::now();
        while (!stop_done && 
               std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (stop_thread.joinable()) {
            if (stop_done) {
                stop_thread.join();
            } else {
                LOG_WARN("Camera stop timeout, detaching");
                stop_thread.detach();
            }
        }
    } catch (...) {
        LOG_WARN("Camera stop failed (ignored)");
    }
    
    // =======================================================
    // 🎯 v0.4.3: ВИПРАВЛЕНА ЛОГІКА ОЧИЩЕННЯ GPIO (C-API)
    // =======================================================
    if (gpio_trigger_line_ != nullptr) {
        LOG_DEBUG("Releasing GPIO trigger line");
        gpiod_line_set_value(gpio_trigger_line_, 0); // Повертаємо в LOW
        gpiod_line_release(gpio_trigger_line_);
        gpio_trigger_line_ = nullptr;
    }
    if (gpio_chip_ != nullptr) {
        gpiod_chip_close(gpio_chip_);
        gpio_chip_ = nullptr;
    }
    // =======================================================
    
    release_camera();
    camera_.reset();
    active_camera_id_ = -1;
    state_ = 0;
    frame_width_ = 0;
    frame_height_ = 0;
    frame_stride_y_ = 0;
    frame_stride_uv_ = 0;
    LOG_INFO("Camera stopped (state 0)");
}

// ... (create_capture_config, allocate_buffers, create_requests - БЕЗ ЗМІН) ...
std::unique_ptr<libcamera::CameraConfiguration> SpiderCamera::create_capture_config() {
    auto config = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!config) {
        throw std::runtime_error("Failed to generate configuration.");
    }
    libcamera::StreamConfiguration &stream_config = config->at(0);
    LOG_INFO("Generated base config for stream role 'Viewfinder'");
    const libcamera::StreamFormats &formats = stream_config.formats();
    
    if (formats.pixelformats().end() != std::find(formats.pixelformats().begin(), 
                                                  formats.pixelformats().end(), 
                                                  libcamera::formats::YUV420)) {
        stream_config.pixelFormat = libcamera::formats::YUV420;
        LOG_INFO("Found preferred PixelFormat: YUV420");
    } 
    else if (formats.pixelformats().end() != std::find(formats.pixelformats().begin(), 
                                                       formats.pixelformats().end(), 
                                                       libcamera::formats::NV12)) {
        stream_config.pixelFormat = libcamera::formats::NV12;
        LOG_INFO("Found alternate PixelFormat: NV12");
    }
    else {
        stream_config.pixelFormat = formats.pixelformats()[0];
        LOG_WARN("No preferred YUV format found. Using default: " 
                 << stream_config.pixelFormat.toString());
    }

    const auto &sizes = stream_config.formats().sizes(stream_config.pixelFormat);
    if (sizes.empty()) {
        throw std::runtime_error("No sizes available for the selected format.");
    }
    
    libcamera::Size target_size;
    std::string target_res_str;

    if (target_width_ == 0 || target_height_ == 0) {
        LOG_INFO("No target resolution set via set_resolution(), using default (4056x3040).");
        target_size = {4056, 3040};
        target_res_str = "4056x3040";
    } else {
        target_size = {target_width_, target_height_};
        target_res_str = std::to_string(target_width_) + "x" + std::to_string(target_height_);
        LOG_INFO("Using target resolution from config: " << target_res_str);
    }
    
    bool found = false;
    for (const auto& size : sizes) {
        if (size.width == target_size.width && size.height == target_size.height) {
            stream_config.size = size;
            found = true;
            break;
        }
    }
    if (!found) {
        stream_config.size = sizes.back(); 
        LOG_WARN("Target resolution " << target_res_str << " not available for this format.");
        LOG_WARN("Falling back to largest available: " 
                 << stream_config.size.width << "x" << stream_config.size.height);
    }
    
    LOG_INFO("Set Resolution to: " << stream_config.size.width << "x" 
             << stream_config.size.height);

    stream_config.bufferCount = 8;
    LOG_INFO("Requesting " << (int)stream_config.bufferCount << " buffers");

    if (config->validate() == libcamera::CameraConfiguration::Invalid) {
        throw std::runtime_error("Failed to validate the created YUV configuration.");
    }
    LOG_INFO("Configuration validated successfully");
    return config;
}

void SpiderCamera::allocate_buffers() {
    if (!camera_ || !config_) {
        throw std::runtime_error("Camera not configured");
    }
    libcamera::Stream *stream = config_->at(0).stream();
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    int ret = allocator_->allocate(stream);
    if (ret < 0) {
        throw std::runtime_error("Failed to allocate buffers: " + std::to_string(ret));
    }
    LOG_INFO("Allocated " << ret << " buffers for stream");
}

void SpiderCamera::create_requests() {
    if (!camera_ || !allocator_) {
        throw std::runtime_error("Camera not ready for request creation");
    }
    libcamera::Stream *stream = config_->at(0).stream();
    const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = 
        allocator_->buffers(stream);
    for (const auto& buffer : buffers) {
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request) {
            throw std::runtime_error("Failed to create request");
        }
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0) {
            throw std::runtime_error("Failed to add buffer to request: " + 
                                   std::to_string(ret));
        }
        requests_.push_back(std::move(request));
    }
    LOG_INFO("Created " << requests_.size() << " requests");
}


void SpiderCamera::be_ready() {
    if (!camera_) {
        throw std::runtime_error("No camera selected. Call set_cam() first.");
    }
    if (state_ != 0) {
        throw std::runtime_error("Camera is already active. Call stop() first.");
    }

    LOG_INFO("Moving to 'ready' state (be_ready)...");
    try {
        if (camera_->acquire()) {
            throw std::runtime_error("Failed to acquire camera.");
        }
        LOG_INFO("Camera acquired: " << camera_->id());

        config_ = create_capture_config();
        
        if (camera_->configure(config_.get()) < 0) {
            throw std::runtime_error("Failed to configure camera.");
        }
        LOG_INFO("Camera configured for ISP (YUV) streaming");

        libcamera::StreamConfiguration &stream_config = config_->at(0);
        current_pixel_format_ = stream_config.pixelFormat;
        frame_width_ = stream_config.size.width;
        frame_height_ = stream_config.size.height;
        
        if (current_pixel_format_ == libcamera::formats::YUV420) {
            frame_stride_y_ = stream_config.stride;
            frame_stride_uv_ = stream_config.stride / 2;
        } else { 
            frame_stride_y_ = stream_config.stride;
            frame_stride_uv_ = stream_config.stride;
        }

        LOG_INFO("Final negotiated Format: " << current_pixel_format_.toString());
        LOG_INFO("  -> Size: " << frame_width_ << "x" << frame_height_);
        LOG_INFO("  -> Y Stride: " << frame_stride_y_);
        LOG_INFO("  -> UV Stride: " << frame_stride_uv_);

        allocate_buffers();
        create_requests();

        // v0.4.1: Підключаємо лише requestCompleted
        camera_->requestCompleted.connect(this, &SpiderCamera::handle_request_complete);
        LOG_DEBUG("Connected requestCompleted signal");

        state_ = 1;
        LOG_INFO("Camera is READY (state 1)");

    } catch (const std::exception& e) {
        LOG_ERROR("Failed during be_ready(): " << e.what());
        state_ = 4;
        release_camera();
    }
}

// =======================================================
// 🎯 v0.4.3: РЕАЛІЗАЦІЯ СЕТТЕРІВ GPIO (C-API)
// =======================================================

void SpiderCamera::enable_frame_trigger(bool enable) {
    LOG_INFO("Setting frame trigger to: " << (enable ? "ON" : "OFF"));
    trigger_enabled_ = enable;
}

/**
 * @brief [v0.4.4: RPi 5 Fix]
 * Використовує 'gpiochip4' за замовчуванням (для RPi 5).
 * Використовує C-API (gpiod_line_request_output).
 */
void SpiderCamera::set_frame_trigger_pin(int pin_num) {
    if (state_ != 0) {
        throw std::runtime_error("Cannot set GPIO pin while camera is active. Call stop() first.");
    }
    LOG_INFO("Configuring GPIO trigger on pin " << pin_num);

    // Очищаємо попередні, якщо є
    if (gpio_trigger_line_) { gpiod_line_release(gpio_trigger_line_); }
    if (gpio_chip_) { gpiod_chip_close(gpio_chip_); }

    try {
        // 🎯 v0.4.4: ЗМІНЕНО НА 'gpiochip4' ДЛЯ RASPBERRY PI 5
        const char* chip_name = "gpiochip4";
        gpio_chip_ = gpiod_chip_open_by_name(chip_name);
        if (!gpio_chip_) {
            throw std::runtime_error("gpiod_chip_open_by_name failed.");
        }
        
        gpio_trigger_line_ = gpiod_chip_get_line(gpio_chip_, pin_num);
        if (!gpio_trigger_line_) {
            throw std::runtime_error("gpiod_chip_get_line failed.");
        }
        
        int ret = gpiod_line_request_output(gpio_trigger_line_, "SpiderCameraTrigger", 0);
        if (ret < 0) {
            throw std::runtime_error("gpiod_line_request_output failed.");
        }
        
        trigger_pin_num_ = pin_num;
        LOG_INFO("GPIO pin " << pin_num << " on " << chip_name << " configured as OUTPUT");
        
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to configure GPIO pin " << pin_num << ": " << e.what());
        LOG_ERROR("Common issues: 1) Is 'libgpiod-dev' installed? 2) Is 'gpiochip4' correct? (Try 'gpiochip0' on RPi 4/older)");
        // Очищаємо, якщо сталася помилка
        if (gpio_trigger_line_) { gpiod_line_release(gpio_trigger_line_); }
        if (gpio_chip_) { gpiod_chip_close(gpio_chip_); }
        gpio_trigger_line_ = nullptr;
        gpio_chip_ = nullptr;
        trigger_pin_num_ = -1;
        throw;
    }
}


// ... (set_exposure, set_focus, set_iso, set_resolution - БЕЗ ЗМІН) ...
void SpiderCamera::set_exposure(int exposure_us) {
    if (exposure_us <= 0) {
        LOG_WARN("Exposure value must be positive. Ignoring.");
        return;
    }
    LOG_INFO("Setting Exposure to " << exposure_us << " us");
    exposure_us_ = exposure_us;
}

void SpiderCamera::set_focus(float focus_value) {
    LOG_INFO("Setting Focus to " << focus_value);
    focus_value_ = focus_value;
}

void SpiderCamera::set_iso(int iso) {
    if (iso <= 0) {
        LOG_WARN("ISO value must be positive. Ignoring.");
        return;
    }
    total_gain_ = iso / BASE_ISO_;
    LOG_INFO("Setting ISO to " << iso 
             << " (Calculated Total AnalogueGain: " << total_gain_ << ")");
}

void SpiderCamera::set_resolution(int w, int h) {
    if (w <= 0 || h <= 0) {
        LOG_WARN("Resolution values must be positive. Ignoring.");
        return;
    }
    LOG_INFO("Setting Target Resolution to " << w << "x" << h);
    target_width_ = static_cast<uint32_t>(w);
    target_height_ = static_cast<uint32_t>(h);
}


// ... (go() - БЕЗ ЗМІН) ...
void SpiderCamera::go() {
    if (state_ != 1) {
        throw std::runtime_error("Camera must be in ready state (1). Current state: " + 
                               std::to_string(state_.load()));
    }
    LOG_INFO("Starting streaming...");
    {
        std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
        frame_data_buffer_.clear();
    }
    py::gil_scoped_release release_gil;
    try {
        for (auto &request : requests_) {
            try {
                request->reuse(libcamera::Request::ReuseBuffers);
            } catch (const std::exception& e) {
                LOG_WARN("Failed to reuse request buffer: " + std::string(e.what()));
            }
        }
        int ret = camera_->start();
        if (ret) {
            throw std::runtime_error("Failed to start camera: " + std::to_string(ret));
        }

        std::stringstream log_msg;
        log_msg << "Queued " << requests_.size() << " requests: ";

        // 💡 ПРИМІТКА: Початкове заповнення черги не викликає requestCapture(),
        // оскільки логіка "розігріву" (в handle_request_complete)
        // обробляє перший сигнал HIGH.
        for (auto &request : requests_) {
            libcamera::ControlList &controls = request->controls();
            
            // v0.3.15: ФІНАЛЬНИЙ ФІКС
            controls.set(libcamera::controls::AeEnable, false);
            controls.set(libcamera::controls::AwbEnable, false); 
            controls.set(libcamera::controls::ExposureTime, exposure_us_);
            controls.set(libcamera::controls::AnalogueGain, total_gain_);
            controls.set(libcamera::controls::DigitalGain, 1.0f); // Завжди 1.0
            controls.set(libcamera::controls::LensPosition, focus_value_);
            
            if (request == requests_[0]) {
                 log_msg << "AE/AWB=FALSE (Manual), "
                         << "ISO~" << (int)(BASE_ISO_ * total_gain_)
                         << " (AG: " << total_gain_ << ", DG: 1.0), "
                         << "Exp: " << exposure_us_ << "us, "
                         << "Focus: " << focus_value_;
            }
            
            camera_->queueRequest(request.get());
        }

        LOG_INFO(log_msg.str());
        
        streaming_ = true;
        frame_count_ = 0; // Скидаємо лічильник для "розігріву"
        error_count_ = 0;
        fps_start_time_ = std::chrono::steady_clock::now();
        stream_thread_ = std::make_unique<std::thread>(&SpiderCamera::stream_loop, this);
        state_ = 2;
        LOG_INFO("Streaming started (state 2)");
    } catch (const std::exception& e) {
        py::gil_scoped_acquire acquire_gil;
        LOG_ERROR("Failed to start streaming: " << e.what());
        streaming_ = false;
        state_ = 4;
        throw;
    }
}

// ... (pause(), stream_loop() - БЕЗ ЗМІН) ...
void SpiderCamera::pause() {
    if (state_ != 2) {
        throw std::runtime_error("Camera is not streaming. Current state: " + 
                               std::to_string(state_.load()));
    }
    LOG_INFO("Pausing streaming...");
    py::gil_scoped_release release_gil;
    streaming_ = false;
    if (stream_thread_) {
        if (stream_thread_->joinable()) {
            stream_thread_->join();
        }
        stream_thread_.reset();
    }
    try {
        if (camera_) {
            camera_->stop();
            LOG_INFO("Camera pipeline stopped");
        }
    } catch (const std::exception& e) {
        LOG_WARN("Exception during camera->stop() in pause: " << e.what());
    }
    state_ = 1;
    LOG_INFO("Streaming paused (state 1)");
}

void SpiderCamera::stream_loop() {
    LOG_DEBUG("Stream loop started");
    while (streaming_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    LOG_DEBUG("Stream loop ended");
}

/**
 * @brief [v0.4.3: ОНОВЛЕНО]
 * Використовує C-API (gpiod_line_set_value).
 * [v0.4.4: FIX] Прибрано '.get()' з 'queueRequest(request)', оскільки 'request'
 * вже є звичайним вказівником (Request*), а не unique_ptr.
 * * 💡 [РЕАЛІЗАЦІЯ ТЗ]:
 * 1. Логіка SET_LOW залишається на самому початку (як вимагає ТЗ).
 * 2. Логіка SET_HIGH + queueRequest винесена в requestCapture().
 */
// Знайти цей метод у src/spider_camera.cpp і замінити повністю:

void SpiderCamera::handle_request_complete(libcamera::Request *request) {
    
    // 1. Якщо зупиняємось - ігноруємо все.
    if (!streaming_) {
        return;
    }

    // 2. Якщо запит скасовано камерою - це нормально при зупинці, ігноруємо.
    if (request->status() == libcamera::Request::RequestCancelled) {
        return;
    }

    const int FRAMES_TO_SKIP_FOR_WARMUP = 5;

    // GPIO: Вимикаємо світло (LOW)
    if (trigger_enabled_ && gpio_trigger_line_ != nullptr) {
        if (frame_count_ > FRAMES_TO_SKIP_FOR_WARMUP) {
            gpiod_line_set_value(gpio_trigger_line_, 0); 
        }
    }

    // 🛑 FIX ЦИКЛУ ПОМИЛОК:
    // Якщо запит завершився невдало (Status != Complete), ми НЕ ПОВИННІ 
    // його перезапускати, бо це створює нескінченний шторм помилок.
    if (request->status() != libcamera::Request::RequestComplete) {
        // Логуємо лише перші кілька помилок, щоб не забивати консоль при падінні
        if (error_count_ < 10) {
             LOG_ERROR("Request failed (status: " << static_cast<int>(request->status()) << "). Dropping frame.");
        }
        error_count_++;
        
        // ☠️ ВАЖЛИВО: МИ НЕ ВИКЛИКАЄМО requestCapture(request) ТУТ!
        // Ми просто "губимо" цей буфер. Це безпечніше, ніж класти камеру.
        return; 
    }

    // Перевірка буферів
    const std::map<const libcamera::Stream *, libcamera::FrameBuffer *> &buffers = request->buffers();
    if (buffers.empty()) {
        LOG_ERROR("No buffers in completed request");
        return; // Також не перезапускаємо, щоб уникнути циклу
    }

    libcamera::FrameBuffer *buffer = buffers.begin()->second;
    
    // --- ЛОГІКА WARMUP ---
    frame_count_++; 
    
    if (frame_count_ <= FRAMES_TO_SKIP_FOR_WARMUP) {
        if (frame_count_ == 1) {
            LOG_INFO("Dropping first " << FRAMES_TO_SKIP_FOR_WARMUP << " frames for sensor warmup...");
        }
        if (frame_count_ == FRAMES_TO_SKIP_FOR_WARMUP) { 
            if (trigger_enabled_ && gpio_trigger_line_ != nullptr) {
                gpiod_line_set_value(gpio_trigger_line_, 1);
            }
        }
        // Тут все безпечно, бо ми знаємо, що RequestComplete
        requestCapture(request);
        return;
    }

    // --- ОБРОБКА КАДРУ (COPY/MMAP) ---
    
    // FPS Logger
    if (frame_count_ % 10 == 0) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start_time_).count();
        if (elapsed > 0) {
            double fps = 10000.0 / elapsed;
            LOG_INFO("Frame rate: " << std::fixed << std::setprecision(1) << fps << " fps");
        }
        fps_start_time_ = now;
    }
    
    if (buffer->planes().size() < 3) {
        LOG_ERROR("YUV: Expected 3 planes");
        requestCapture(request); // Тут можна ризикнути повернути
        return;
    }

    const libcamera::FrameBuffer::Plane &y_plane = buffer->planes()[0];
    const libcamera::FrameBuffer::Plane &u_plane = buffer->planes()[1];
    const libcamera::FrameBuffer::Plane &v_plane = buffer->planes()[2];

    size_t total_buffer_span = (v_plane.offset - y_plane.offset) + v_plane.length;
    void *buffer_base_ptr = mmap(nullptr, total_buffer_span, PROT_READ, MAP_SHARED, y_plane.fd.get(), y_plane.offset);

    if (buffer_base_ptr != MAP_FAILED) {
        try {
            const uint8_t *base_ptr = static_cast<const uint8_t*>(buffer_base_ptr);
            
            // ВАШ КОД КОПІЮВАННЯ (зберіг скорочено для читабельності)
            size_t y_size = frame_width_ * frame_height_;
            size_t uv_width = frame_width_ / 2;
            size_t uv_height = frame_height_ / 2;
            size_t uv_size = uv_width * uv_height;
            size_t total_size = y_size + uv_size + uv_size;

            std::vector<uint8_t> frame_data(total_size);
            uint8_t *dst = frame_data.data();
            
            // Y Plane
            if (frame_stride_y_ == frame_width_) {
                std::memcpy(dst, base_ptr, y_size);
            } else {
                const uint8_t *src = base_ptr;
                for (uint32_t i = 0; i < frame_height_; ++i) {
                    std::memcpy(dst, src, frame_width_);
                    src += frame_stride_y_;
                    dst += frame_width_;
                }
            }
            
            // UV Planes
            const uint8_t *u_src = base_ptr + (u_plane.offset - y_plane.offset);
            uint8_t *u_dst = dst + y_size;
             if (frame_stride_uv_ == uv_width) {
                std::memcpy(u_dst, u_src, uv_size);
            } else {
                for(uint32_t i=0; i<uv_height; ++i) {
                    std::memcpy(u_dst, u_src, uv_width);
                    u_src += frame_stride_uv_;
                    u_dst += uv_width;
                }
            }
            
            const uint8_t *v_src = base_ptr + (v_plane.offset - y_plane.offset);
            uint8_t *v_dst = u_dst + uv_size;
             if (frame_stride_uv_ == uv_width) {
                std::memcpy(v_dst, v_src, uv_size);
            } else {
                for(uint32_t i=0; i<uv_height; ++i) {
                    std::memcpy(v_dst, v_src, uv_width);
                    v_src += frame_stride_uv_;
                    v_dst += uv_width;
                }
            }

            {
                std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
                frame_data_buffer_.push_back(std::move(frame_data));
            }
            error_count_ = 0;
            
        } catch (...) {
            LOG_ERROR("Frame copy exception");
        }
        munmap(buffer_base_ptr, total_buffer_span);
    }

    // Успішно оброблений кадр повертаємо в чергу
    requestCapture(request);
}

/**
 * @brief [РЕАЛІЗАЦІЯ ТЗ]
 * Новий приватний метод, що інкапсулює логіку ТЗ:
 * "Ввімкнути світло (HIGH) -> Поставити запит в чергу (queueRequest)"
 */
void SpiderCamera::requestCapture(libcamera::Request *request) {
    
    // =======================================================
    // 🎯 [РЕАЛІЗАЦІЯ ТЗ]: 💡 Вмикаємо світло ДО старту експозиції
    // =======================================================
    if (trigger_enabled_ && gpio_trigger_line_ != nullptr) {
        // Ми перевіряємо streaming_, щоб не вмикати світло,
        // коли ми зупиняємо потік (streaming_ == false)
        if (streaming_) { 
            if (gpiod_line_set_value(gpio_trigger_line_, 1) < 0) { // 1 = HIGH
                LOG_WARN("Failed to set GPIO HIGH");
            }
            LOG_DEBUG("GPIO HIGH (Trigger for next frame)");
        }
    }
    // =======================================================
    
    // Повторно ставимо запит в чергу
    request->reuse(libcamera::Request::ReuseBuffers);
    camera_->queueRequest(request);
}


// ... (get_frame_properties, get_burst_frames - БЕЗ ЗМІН) ...
py::tuple SpiderCamera::get_frame_properties() {
    py::gil_scoped_acquire acquire_gil;
    if (state_ == 0) {
        throw std::runtime_error("Camera not ready. Call be_ready() first.");
    }
    return py::make_tuple(frame_width_, 
                          frame_height_, 
                          current_pixel_format_.toString());
}

py::list SpiderCamera::get_burst_frames() {
    std::vector<std::vector<uint8_t>> frames_to_process;
    {
        std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
        frames_to_process = std::move(frame_data_buffer_);
        frame_data_buffer_.clear();
    }
    LOG_INFO("Processing " << (int)frames_to_process.size() << " buffered frames...");
    LOG_INFO("Copying " << (int)frames_to_process.size() << " YUV frames to Python...");

    py::list frame_list;
    std::vector<py::array_t<uint8_t>> py_arrays;
    py_arrays.reserve(frames_to_process.size());

    try {
        for (const auto& frame_data : frames_to_process) {
            py::array_t<uint8_t> arr(frame_data.size());
            py_arrays.push_back(arr);
            frame_list.append(arr);
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Failed during Python memory pre-allocation: " << e.what());
        return py::list();
    }
    
    LOG_DEBUG("Pre-allocated " << py_arrays.size() << " numpy arrays.");

    {
        py::gil_scoped_release release_gil;
        for (size_t i = 0; i < frames_to_process.size(); ++i) {
            auto buf = py_arrays[i].request(false);
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            std::memcpy(ptr, frames_to_process[i].data(), frames_to_process[i].size());
        }
    } 
    
    int final_count = 0;
    try { 
        final_count = py::len(frame_list); 
    } catch (...) { /* ігноруємо */ }
    
    LOG_INFO("Data copy complete. Returning " << final_count << " frames (YUV data).");
    return frame_list;
}

// ... (get_state, set_frame_callback, enable_debug, convert_to_numpy - БЕЗ ЗМІН) ...
int SpiderCamera::get_state() const {
    return state_;
}

void SpiderCamera::set_frame_callback(std::function<void(py::array, double)> callback) {
    LOG_WARN("set_frame_callback is not used in YUV Burst mode.");
    std::lock_guard<std::mutex> lock(callback_mutex_);
    frame_callback_ = callback;
}

void SpiderCamera::enable_debug(bool enable) {
    debug_enabled_ = enable;
    LOG_INFO("Debug logging " << (enable ? "enabled" : "disabled"));
}

py::array SpiderCamera::convert_to_numpy(libcamera::FrameBuffer *buffer) {
    LOG_WARN("convert_to_numpy is deprecated.");
    (void)buffer;
    return py::array();
}
