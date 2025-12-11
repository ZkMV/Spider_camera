/*
 * spider_camera.cpp
 *
 * Implementation file for the SpiderCamera library (v0.6.1 - FPS Calc).
 * v0.6.1: Implemented physical FPS calculation (Hardware timestamps).
 * v0.6: 
 * 1. Implemented Raw Memory Copy in handle_request_complete (fixes SegFaults/Padding issues).
 * 2. Added stride reporting in get_frame_properties.
 * v0.4.3: Uses C-API for libgpiod.
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

// 🎯 v0.4.3: C API
#include <gpiod.h>

// Macros
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
    
    // 🎯 v0.6.1: Calculate FPS if we were streaming
    if (state_ == 2) {
        // Stop streaming logic
        streaming_ = false;
        
        // --- FPS CALCULATION ---
        if (valid_frames_count_ > 1) {
            auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count();
            double duration_sec = duration_us / 1000000.0;
            if (duration_sec > 0.0001) {
                last_calculated_fps_ = static_cast<float>((valid_frames_count_ - 1) / duration_sec);
                LOG_INFO("Capture finished. Calculated FPS: " << last_calculated_fps_);
            }
        } else {
            last_calculated_fps_ = 0.0f;
        }
        // -----------------------

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
            if (stop_done) stop_thread.join();
            else stop_thread.detach();
        }
    } catch (...) {
        LOG_WARN("Camera stop failed (ignored)");
    }
    
    // Cleanup GPIO (C-API)
    if (gpio_trigger_line_ != nullptr) {
        LOG_DEBUG("Releasing GPIO trigger line");
        gpiod_line_set_value(gpio_trigger_line_, 0); 
        gpiod_line_release(gpio_trigger_line_);
        gpio_trigger_line_ = nullptr;
    }
    if (gpio_chip_ != nullptr) {
        gpiod_chip_close(gpio_chip_);
        gpio_chip_ = nullptr;
    }
    
    release_camera();
    camera_.reset();
    active_camera_id_ = -1;
    state_ = 0;
    frame_width_ = 0;
    frame_height_ = 0;
    stride_ = 0; // Reset stride
    
    LOG_INFO("Camera stopped (state 0)");
}

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

// --- be_ready: v0.6 Changes (Reading Stride) ---
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
        
        // 🎯 v0.6: Capture Stride from hardware config
        stride_ = stream_config.stride; 
        
        // Legacy fields used only for logging now
        frame_stride_y_ = stream_config.stride;
        frame_stride_uv_ = (current_pixel_format_ == libcamera::formats::YUV420) ? stream_config.stride / 2 : stream_config.stride;

        LOG_INFO("Final negotiated Format: " << current_pixel_format_.toString());
        LOG_INFO("  -> Size: " << frame_width_ << "x" << frame_height_);
        LOG_INFO("  -> Hardware Stride: " << stride_ << " bytes (Use this for reshaping!)");

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
        throw;
    }
}

// --- GPIO Functions (C-API) ---

void SpiderCamera::enable_frame_trigger(bool enable) {
    LOG_INFO("Setting frame trigger to: " << (enable ? "ON" : "OFF"));
    trigger_enabled_ = enable;
}

void SpiderCamera::set_frame_trigger_pin(int pin_num) {
    if (state_ != 0) {
        throw std::runtime_error("Cannot set GPIO pin while camera is active. Call stop() first.");
    }
    LOG_INFO("Configuring GPIO trigger on pin " << pin_num);

    // Cleanup old
    if (gpio_trigger_line_) { gpiod_line_release(gpio_trigger_line_); }
    if (gpio_chip_) { gpiod_chip_close(gpio_chip_); }

    try {
        // 識 v0.4.4: Fixed for RPi 5 (gpiochip4)
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
        if (gpio_trigger_line_) { gpiod_line_release(gpio_trigger_line_); }
        if (gpio_chip_) { gpiod_chip_close(gpio_chip_); }
        gpio_trigger_line_ = nullptr;
        gpio_chip_ = nullptr;
        trigger_pin_num_ = -1;
        throw;
    }
}

// --- Hot Params ---
void SpiderCamera::set_exposure(int exposure_us) {
    if (exposure_us <= 0) { LOG_WARN("Exposure value must be positive."); return; }
    LOG_INFO("Setting Exposure to " << exposure_us << " us");
    exposure_us_ = exposure_us;
}

void SpiderCamera::set_focus(float focus_value) {
    LOG_INFO("Setting Focus to " << focus_value);
    focus_value_ = focus_value;
}

void SpiderCamera::set_iso(int iso) {
    if (iso <= 0) { LOG_WARN("ISO value must be positive."); return; }
    total_gain_ = iso / BASE_ISO_;
    LOG_INFO("Setting ISO to " << iso << " (AnalogueGain: " << total_gain_ << ")");
}

void SpiderCamera::set_resolution(int w, int h) {
    if (w <= 0 || h <= 0) { LOG_WARN("Resolution must be positive."); return; }
    LOG_INFO("Setting Target Resolution to " << w << "x" << h);
    target_width_ = static_cast<uint32_t>(w);
    target_height_ = static_cast<uint32_t>(h);
}

// --- Streaming ---
void SpiderCamera::go() {
    if (state_ != 1) {
        throw std::runtime_error("Camera must be in ready state (1).");
    }
    LOG_INFO("Starting streaming...");

    // 🎯 v0.6.1: Reset FPS calculation variables
    valid_frames_count_ = 0;
    last_calculated_fps_ = 0.0f;
    first_valid_frame_received_ = false;

    {
        std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
        frame_data_buffer_.clear();
    }
    py::gil_scoped_release release_gil;
    try {
        for (auto &request : requests_) {
            try { request->reuse(libcamera::Request::ReuseBuffers); } 
            catch (...) {}
        }
        int ret = camera_->start();
        if (ret) throw std::runtime_error("Failed to start camera: " + std::to_string(ret));

        std::stringstream log_msg;
        log_msg << "Queued " << requests_.size() << " requests: ";

        for (auto &request : requests_) {
            libcamera::ControlList &controls = request->controls();
            controls.set(libcamera::controls::AeEnable, false);
            controls.set(libcamera::controls::AwbEnable, false); 
            controls.set(libcamera::controls::ExposureTime, exposure_us_);
            controls.set(libcamera::controls::AnalogueGain, total_gain_);
            controls.set(libcamera::controls::DigitalGain, 1.0f); 
            controls.set(libcamera::controls::LensPosition, focus_value_);
            
            if (request == requests_[0]) {
                 log_msg << "Manual Exp, ISO~" << (int)(BASE_ISO_ * total_gain_)
                         << ", Exp: " << exposure_us_ << "us, Focus: " << focus_value_;
            }
            camera_->queueRequest(request.get());
        }

        LOG_INFO(log_msg.str());
        
        streaming_ = true;
        frame_count_ = 0;
        error_count_ = 0;
        fps_start_time_ = std::chrono::steady_clock::now(); // For logging only
        
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
    if (state_ != 2) throw std::runtime_error("Camera is not streaming.");
    LOG_INFO("Pausing streaming...");
    
    // 🎯 v0.6.1: Calculate FPS upon pausing
    if (streaming_) {
         if (valid_frames_count_ > 1) {
            auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count();
            double duration_sec = duration_us / 1000000.0;
            if (duration_sec > 0.0001) {
                last_calculated_fps_ = static_cast<float>((valid_frames_count_ - 1) / duration_sec);
                LOG_INFO("Paused. Series FPS: " << last_calculated_fps_);
            }
        } else {
            last_calculated_fps_ = 0.0f;
        }
    }

    py::gil_scoped_release release_gil;
    streaming_ = false;
    if (stream_thread_) {
        if (stream_thread_->joinable()) stream_thread_->join();
        stream_thread_.reset();
    }
    try {
        if (camera_) {
            camera_->stop();
            LOG_INFO("Camera pipeline stopped");
        }
    } catch (...) {}
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


// --- handle_request_complete: v0.6 REWRITE (Raw Copy) ---
void SpiderCamera::handle_request_complete(libcamera::Request *request) {
    
    if (!streaming_) return;
    if (request->status() == libcamera::Request::RequestCancelled) return;

    const int FRAMES_TO_SKIP_FOR_WARMUP = 5;

    // GPIO: LOW (Light Off)
    if (trigger_enabled_ && gpio_trigger_line_ != nullptr) {
        if (frame_count_ > FRAMES_TO_SKIP_FOR_WARMUP) {
            gpiod_line_set_value(gpio_trigger_line_, 0); 
        }
    }

    if (request->status() != libcamera::Request::RequestComplete) {
        if (error_count_ < 10) {
             LOG_ERROR("Request failed (status: " << static_cast<int>(request->status()) << "). Dropping.");
        }
        error_count_++;
        return; 
    }

    const auto &buffers = request->buffers();
    if (buffers.empty()) {
        LOG_ERROR("No buffers in completed request");
        return;
    }

    libcamera::FrameBuffer *buffer = buffers.begin()->second;
    
    // --- Warmup Logic ---
    frame_count_++; 
    if (frame_count_ <= FRAMES_TO_SKIP_FOR_WARMUP) {
        if (frame_count_ == 1) LOG_INFO("Dropping warmup frames...");
        if (frame_count_ == FRAMES_TO_SKIP_FOR_WARMUP) { 
            if (trigger_enabled_ && gpio_trigger_line_ != nullptr) {
                gpiod_line_set_value(gpio_trigger_line_, 1); // Pre-trigger next frame
            }
        }
        requestCapture(request);
        return;
    }

    // 🎯 v0.6.1: Physical FPS Timestamping
    {
        auto now = std::chrono::steady_clock::now();
        if (!first_valid_frame_received_) {
            start_time_ = now;
            first_valid_frame_received_ = true;
        }
        end_time_ = now;
        valid_frames_count_++;
    }

    // --- FPS Logger (Visual only) ---
    if (frame_count_ % 10 == 0) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start_time_).count();
        if (elapsed > 0) {
            double fps = 10000.0 / elapsed;
            LOG_INFO("Frame rate (log): " << std::fixed << std::setprecision(1) << fps << " fps");
        }
        fps_start_time_ = now;
    }
    
    // --- v0.6: Raw Memory Copy (Fixing Padding/SegFaults) ---
    const std::vector<libcamera::FrameBuffer::Plane> &planes = buffer->planes();
    if (planes.empty()) {
        requestCapture(request);
        return;
    }

    // Calculate total span of the buffer in memory
    const auto &first_plane = planes[0];
    const auto &last_plane = planes[planes.size() - 1];
    size_t total_span = (last_plane.offset - first_plane.offset) + last_plane.length;

    // Mmap the file descriptor
    void *buffer_base_ptr = mmap(nullptr, total_span, PROT_READ, MAP_SHARED, first_plane.fd.get(), first_plane.offset);

    if (buffer_base_ptr != MAP_FAILED) {
        try {
            // Allocate vector for the RAW buffer (including stride padding)
            std::vector<uint8_t> frame_data(total_span);
            
            // Memcpy everything
            std::memcpy(frame_data.data(), buffer_base_ptr, total_span);

            {
                std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
                frame_data_buffer_.push_back(std::move(frame_data));
            }
            error_count_ = 0;
            
        } catch (...) {
            LOG_ERROR("Frame copy exception");
        }
        munmap(buffer_base_ptr, total_span);
    } else {
        LOG_ERROR("mmap failed: " << std::strerror(errno));
    }

    // Return request to queue
    requestCapture(request);
}

void SpiderCamera::requestCapture(libcamera::Request *request) {
    // GPIO HIGH (Light ON for next frame)
    if (trigger_enabled_ && gpio_trigger_line_ != nullptr && streaming_) {
        if (gpiod_line_set_value(gpio_trigger_line_, 1) < 0) {
            LOG_WARN("Failed to set GPIO HIGH");
        }
    }
    
    request->reuse(libcamera::Request::ReuseBuffers);
    camera_->queueRequest(request);
}


// --- v0.6: Updated Property Getter ---
py::tuple SpiderCamera::get_frame_properties() {
    py::gil_scoped_acquire acquire_gil;
    if (state_ == 0) {
        throw std::runtime_error("Camera not ready. Call be_ready() first.");
    }
    // Returns (width, height, format, STRIDE)
    return py::make_tuple(frame_width_, 
                          frame_height_, 
                          current_pixel_format_.toString(),
                          stride_);
}

// 🎯 v0.6.1: New FPS Getter
float SpiderCamera::get_last_series_fps() const {
    return last_calculated_fps_;
}

py::list SpiderCamera::get_burst_frames() {
    std::vector<std::vector<uint8_t>> frames_to_process;
    {
        std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
        frames_to_process = std::move(frame_data_buffer_);
        frame_data_buffer_.clear();
    }
    LOG_INFO("Processing " << (int)frames_to_process.size() << " buffered frames...");
    
    py::list frame_list;

    py::gil_scoped_release release_gil;
    
    py::gil_scoped_acquire acquire_gil;

    try {
        for (const auto& frame_data : frames_to_process) {
            // Create a numpy array of the exact size of our buffer (which includes padding)
            py::array_t<uint8_t> arr(frame_data.size());
            
            // Copy data
            std::memcpy(arr.request(true).ptr, frame_data.data(), frame_data.size());
            
            frame_list.append(arr);
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Failed during Python memory copy: " << e.what());
        return py::list();
    }
    
    LOG_INFO("Returned " << py::len(frame_list) << " frames (Raw Buffer with Stride).");
    return frame_list;
}

// --- Deprecated / Unused ---
int SpiderCamera::get_state() const { return state_; }
void SpiderCamera::set_frame_callback(std::function<void(py::array, double)> callback) { 
    std::lock_guard<std::mutex> lock(callback_mutex_);
    frame_callback_ = callback; 
}
void SpiderCamera::enable_debug(bool enable) { debug_enabled_ = enable; LOG_INFO("Debug: " << enable); }
py::array SpiderCamera::convert_to_numpy(libcamera::FrameBuffer *buffer) { (void)buffer; return py::array(); }
