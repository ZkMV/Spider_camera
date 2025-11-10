/*
 * spider_camera.cpp
 *
 * Implementation file for the SpiderCamera library (v0.2.11 - Race Condition Fix).
 * v0.2.11: Fixed race condition on stop/pause by adding a second
 * 'streaming_' check before requeueing.
 */

#include "spider_camera.hpp"
#include "pisp_decompress.hpp"
#include "frame_buffer.hpp"
#include <libcamera/pixel_format.h>
#include <iostream>
#include <iomanip>
#include <sys/mman.h>
#include <unistd.h>
#include <cerrno>
#include <pybind11/stl.h>
#include <libcamera/control_ids.h>

// Logging macros
#define LOG_INFO(msg)   std::cout << "[INFO]  " << msg << std::endl
#define LOG_DEBUG(msg)  if (debug_enabled_) std::cout << "[DEBUG] " << msg << std::endl
#define LOG_WARN(msg)   std::cerr << "[WARN]  " << msg << std::endl
#define LOG_ERROR(msg)  std::cerr << "[ERROR] " << msg << std::endl

// (SpiderCamera(), ~SpiderCamera(), set_cam(), get_cam(), release_camera(), stop(),
//  create_raw_config(), allocate_buffers(), create_requests(), be_ready(), go(),
//  pause(), stream_loop() - УСІ ЦІ ФУНКЦІЇ ЗАЛИШАЮТЬСЯ БЕЗ ЗМІН з v0.2.9)
SpiderCamera::SpiderCamera() : state_(0), active_camera_id_(-1) {
    cam_mgr_ = std::make_shared<libcamera::CameraManager>();
    int ret = cam_mgr_->start();
    if (ret) {
        LOG_ERROR("Failed to start CameraManager: " << ret);
        state_ = 4; // Error state
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
    } catch (...) {
        // Ignore errors during shutdown
    }
    
    stop();
    
    if (cam_mgr_) {
        cam_mgr_->stop();
    }
}

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
    
    release_camera();
    
    camera_.reset();
    active_camera_id_ = -1;
    state_ = 0;
    frame_width_ = 0;
    frame_height_ = 0;
    LOG_INFO("Camera stopped (state 0)");
}

std::unique_ptr<libcamera::CameraConfiguration> SpiderCamera::create_raw_config() {
    auto config = camera_->generateConfiguration({libcamera::StreamRole::Raw});
    if (!config) {
        throw std::runtime_error("Failed to generate configuration.");
    }

    libcamera::StreamConfiguration &stream_config = config->at(0);
    LOG_INFO("Generated base config for stream role 'Raw'");

    const libcamera::StreamFormats &formats = stream_config.formats();
    const auto &pix_formats = formats.pixelformats();

    if (pix_formats.empty()) {
        throw std::runtime_error("No pixel formats available for RAW stream.");
    }

    libcamera::PixelFormat target_format;
    bool found_target = false;

    for (const auto& pf : pix_formats) {
        if (pf.toString().find("PISP_COMP1") != std::string::npos) {
            target_format = pf;
            found_target = true;
            LOG_INFO("Found preferred PixelFormat: " << target_format.toString());
            break;
        }
    }

    if (!found_target) {
        for (const auto& pf : pix_formats) {
            if (pf.toString().find("CSI2P") != std::string::npos) {
                target_format = pf;
                found_target = true;
                LOG_INFO("Found alternate PixelFormat: " << target_format.toString());
                break;
            }
        }
    }

    if (!found_target) {
        target_format = pix_formats[0];
        LOG_WARN("No preferred RAW format found. Using default: " << target_format.toString());
    }
    
    stream_config.pixelFormat = target_format;

    const auto &sizes = stream_config.formats().sizes(target_format);
    if (sizes.empty()) {
        throw std::runtime_error("No sizes available for the selected format.");
    }
    
    libcamera::Size target_size{4056, 3040};
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
        LOG_WARN("Target resolution 4056x3040 not available, using: " 
                 << stream_config.size.width << "x" << stream_config.size.height);
    }
    
    LOG_INFO("Set Resolution to: " << stream_config.size.width << "x" 
             << stream_config.size.height);


    stream_config.bufferCount = 8;
    LOG_INFO("Requesting " << (int)stream_config.bufferCount << " buffers");

    if (config->validate() == libcamera::CameraConfiguration::Invalid) {
        throw std::runtime_error("Failed to validate the created RAW configuration.");
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
    
    const libcamera::StreamConfiguration &cfg = config_->at(0);
    frame_width_ = cfg.size.width;
    frame_height_ = cfg.size.height;

    size_t num_pixels = frame_width_ * frame_height_;
    if (decompression_buffer_.size() != num_pixels) {
        LOG_DEBUG("Resizing decompression buffer to " << num_pixels << " (uint16_t)");
        try {
            decompression_buffer_.resize(num_pixels);
        } catch (const std::bad_alloc& e) {
            LOG_ERROR("Failed to allocate decompression buffer (size: " 
                      << num_pixels * sizeof(uint16_t) << "): " << e.what());
            throw std::runtime_error("Failed to allocate decompression buffer");
        }
    }
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

        config_ = create_raw_config();
        
        if (camera_->configure(config_.get()) < 0) {
            throw std::runtime_error("Failed to configure camera.");
        }
        LOG_INFO("Camera configured for RAW streaming");

        current_pixel_format_ = config_->at(0).pixelFormat;
        LOG_INFO("Final negotiated PixelFormat: " << current_pixel_format_.toString());

        allocate_buffers();
        create_requests();

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

void SpiderCamera::go() {
    if (state_ != 1) {
        throw std::runtime_error("Camera must be in ready state (1). Current state: " + 
                               std::to_string(state_.load()));
    }

    LOG_INFO("Starting streaming...");

    {
        std::lock_guard<std::mutex> lock(burst_buffer_mutex_);
        compressed_frame_buffer_.clear();
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

        for (auto &request : requests_) {
            libcamera::ControlList &controls = request->controls();
            
            int64_t frame_duration[2] = {71428, 71428};
            controls.set(libcamera::controls::FrameDurationLimits, frame_duration);
            controls.set(libcamera::controls::AeEnable, false);
            controls.set(libcamera::controls::ExposureTime, 100);
            controls.set(libcamera::controls::AnalogueGain, 16.0f);
            
            camera_->queueRequest(request.get());
        }
        
        LOG_INFO("Queued " << requests_.size() << " requests: 14fps, ISO 4000, Exp 100us");
        
        streaming_ = true;
        frame_count_ = 0;
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

/*
 * === v0.2.11: handle_request_complete (Race Condition Fix) ===
 */
void SpiderCamera::handle_request_complete(libcamera::Request *request) {
    // 1. Перевіряємо, чи ми вже не зупинились
    if (!streaming_) {
        LOG_DEBUG("Request completed, but streaming is false (ignoring)");
        return;
    }

    if (request->status() == libcamera::Request::RequestCancelled) {
        LOG_DEBUG("Request was cancelled");
        return;
    }

    if (request->status() != libcamera::Request::RequestComplete) {
        LOG_ERROR("Request failed (status: " << static_cast<int>(request->status()) << "), frame dropped.");
        error_count_++;
        // Все одно ставимо запит у чергу (якщо все ще стрімимо)
        if (streaming_) { // <--- v0.2.11 Додаткова перевірка
            request->reuse(libcamera::Request::ReuseBuffers);
            camera_->queueRequest(request);
        }
        return;
    }

    const auto &buffers = request->buffers();
    if (buffers.empty()) {
        LOG_ERROR("No buffers in completed request");
        return;
    }

    libcamera::FrameBuffer *buffer = buffers.begin()->second;
    const libcamera::FrameBuffer::Plane &plane = buffer->planes()[0];
    
    frame_count_++;

    void *data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, 
                     plane.fd.get(), 0);
    
    if (data == MAP_FAILED) {
        LOG_ERROR("Failed to mmap buffer");
        if (streaming_) { // <--- v0.2.11 Додаткова перевірка
            request->reuse(libcamera::Request::ReuseBuffers);
            camera_->queueRequest(request);
        }
        return;
    }

    try {
        // === Логіка копіювання (залишається та ж сама) ===
        std::vector<uint8_t> compressed_data_copy(plane.length);
        std::memcpy(compressed_data_copy.data(), 
                      static_cast<const uint8_t*>(data), 
                      plane.length);

        munmap(data, plane.length);

        {
            std::lock_guard<std::mutex> lock(burst_buffer_mutex_);
            compressed_frame_buffer_.push_back(std::move(compressed_data_copy));
            
            LOG_DEBUG("Burst frame " << (int)compressed_frame_buffer_.size() 
                      << " captured (compressed)");
        }
        
        error_count_ = 0;

    } catch (const std::bad_alloc& e) {
        LOG_ERROR("Failed to allocate copy buffer: " << e.what());
        munmap(data, plane.length);
    } catch (const std::exception& e) {
        LOG_ERROR("Frame capture/copy failed: " << e.what());
        munmap(data, plane.length);
    }

    // === v0.2.11: FIX: Головна перевірка на Race Condition ===
    // Повторно перевіряємо прапор 'streaming_' ПЕРЕД тим, як 
    // поставити запит у чергу.
    // Якщо pause() була викликана, поки ми копіювали дані,
    // 'streaming_' буде 'false', і ми не будемо нічого ставити в чергу.
    if (streaming_) {
        request->reuse(libcamera::Request::ReuseBuffers);
        
        libcamera::ControlList &controls = request->controls();
        int64_t frame_duration[2] = {71428, 71428};
        controls.set(libcamera::controls::FrameDurationLimits, frame_duration);
        controls.set(libcamera::controls::AeEnable, false);
        controls.set(libcamera::controls::ExposureTime, 100);
        controls.set(libcamera::controls::AnalogueGain, 16.0f);
        
        // Цей виклик тепер безпечний
        camera_->queueRequest(request);
    } else {
        // Ми в процесі зупинки, не ставимо цей запит у чергу.
        // Буфер буде звільнено, коли камера зупиниться/звільниться.
        LOG_DEBUG("Streaming stopped, not requeueing frame.");
    }
}


/**
 * @brief [v0.2.12: FIX] Обробляє буферизовані кадри.
 * Виправлено Segfault шляхом попереднього виділення пам'яті NumPy.
 */
py::list SpiderCamera::get_burst_frames() {
    int frame_count_to_process = 0;
    
    // 1. Отримуємо список кадрів для обробки (з GIL)
    std::vector<std::vector<uint8_t>> frames_to_process;
    {
        std::lock_guard<std::mutex> lock(burst_buffer_mutex_);
        frame_count_to_process = (int)compressed_frame_buffer_.size();
        frames_to_process = std::move(compressed_frame_buffer_);
        compressed_frame_buffer_.clear();
    }
    
    LOG_INFO("Processing " << frame_count_to_process << " buffered frames...");
    LOG_INFO("Decompressing " << (int)frames_to_process.size() << " frames...");

    py::list frame_list;
    // Вектор для зберігання "сирих" C++ вказівників на буфери NumPy
    std::vector<uint16_t*> output_pointers;
    
    size_t pixel_count = frame_width_ * frame_height_;

    // 2. === ЕТАП ПОПЕРЕДНЬОГО ВИДІЛЕННЯ (з GIL) ===
    // Створюємо всі об'єкти Python (масиви) заздалегідь.
    try {
        for (int i = 0; i < frame_count_to_process; ++i) {
            // Перевіряємо, чи кадр не пошкоджений, ПЕРЕД виділенням пам'яті
            if (frames_to_process[i].size() < pixel_count) {
                continue; // Пропустимо цей кадр
            }
            
            // Створюємо новий порожній масив NumPy
            py::array_t<uint16_t> arr(
                {static_cast<py::ssize_t>(frame_height_), 
                 static_cast<py::ssize_t>(frame_width_)}
            );
            
            // Додаємо його до списку Python
            frame_list.append(arr);
            
            // Зберігаємо "сирий" C++ вказівник на його буфер
            auto buf = arr.request();
            output_pointers.push_back(static_cast<uint16_t*>(buf.ptr));
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Failed during Python memory allocation: " << e.what());
        // Повертаємо порожній список у разі помилки виділення пам'яті
        return py::list(); 
    }
    
    LOG_DEBUG("Pre-allocated " << (int)output_pointers.size() << " numpy arrays.");

    // 3. === ЕТАП ДЕКОМПРЕСІЇ (Без GIL) ===
    // Тепер у нас є чиста C++ робота: заповнити N буферів
    {
        py::gil_scoped_release release_gil;
        
        int output_idx = 0; // Індекс для вказівників
        for (const auto& compressed_frame : frames_to_process) {
            
            // Ми маємо повторити ту саму перевірку, щоб
            // 'output_pointers' і 'frames_to_process' були синхронізовані
            if (compressed_frame.size() < pixel_count) {
                LOG_WARN("Skipping damaged/incomplete frame (size mismatch)");
                continue;
            }

            // Перевірка, чи не вийшли ми за межі
            if (output_idx >= (int)output_pointers.size()) {
                // Цього не повинно статися, але це безпечна перевірка
                break; 
            }

            // Отримуємо вказівник на буфер NumPy
            uint16_t* ptr = output_pointers[output_idx];
            
            // Виконуємо декомпресію
            decompress_pisp(ptr, compressed_frame.data(), frame_width_, frame_height_);
            
            output_idx++;
        }
    } // GIL автоматично повертається тут
    
    // 4. === ПОВЕРНЕННЯ (з GIL) ===
    int final_count = 0;
    try { 
        final_count = py::len(frame_list); 
    } catch (...) { /* ігноруємо */ }
    
    LOG_INFO("Decompression complete. Returning " << final_count << " frames.");
    
    return frame_list;
}


// (get_state(), set_frame_callback(), enable_debug(), convert_to_numpy() 
//  залишаються без змін з v0.2.9)
int SpiderCamera::get_state() const {
    return state_;
}

void SpiderCamera::set_frame_callback(std::function<void(py::array, double)> callback) {
    LOG_WARN("set_frame_callback is not used in Burst Capture mode.");
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
