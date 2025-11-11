/*
 * spider_camera.cpp
 *
 * Implementation file for the SpiderCamera library (v0.3.9.1 - Macro Fix).
 * v0.3.9.1: 1. Re-added the logging macros (LOG_INFO, etc.) that
 * were mistakenly omitted in the previous version.
 * 2. (Fix) Commented out FrameDurationLimits (Control 0x1c).
 * 3. (Fix) Implemented robust single mmap() logic for YUV planes.
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
#include <cstring> // <--- Додано для std::memcpy

// =======================================================
// 🎯 ВИРІШЕННЯ: Повертаємо макроси логування
// =======================================================
#define LOG_INFO(msg)   std::cout << "[INFO]  " << msg << std::endl
#define LOG_DEBUG(msg)  if (debug_enabled_) std::cout << "[DEBUG] " << msg << std::endl
#define LOG_WARN(msg)   std::cerr << "[WARN]  " << msg << std::endl
#define LOG_ERROR(msg)  std::cerr << "[ERROR] " << msg << std::endl
// =======================================================


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
    frame_stride_y_ = 0;
    frame_stride_uv_ = 0;
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

/**
 * @brief [v0.3.10: ЗМІНЕНО]
 * Імітуємо логіку picamera2: залишаємо AeEnable = true,
 * але примусово "перезаписуємо" її значення, передаючи
 * ExposureTime, AnalogueGain та DigitalGain у тому ж запиті.
 * Це змушує ISP застосувати DigitalGain.
 */
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
        for (auto &request : requests_) {
            libcamera::ControlList &controls = request->controls();
            
            // =======================================================
            // 🎯 ВИРІШЕННЯ: Ми *вмикаємо* авторежими,
            // але негайно "перезаписуємо" їх ручними значеннями.
            
            controls.set(libcamera::controls::AeEnable, true);
            controls.set(libcamera::controls::AwbEnable, true); // Також вмикаємо AWB для стабільності

            // Встановлюємо наші примусові ручні значення:
            controls.set(libcamera::controls::ExposureTime, 100);
            controls.set(libcamera::controls::AnalogueGain, 16.0f);
            controls.set(libcamera::controls::DigitalGain, 2.5f);
            // =======================================================
            
            camera_->queueRequest(request.get());
        }

        LOG_INFO("Queued " << requests_.size() << " requests: AE/AWB=ON (Manual Override), ISO 4000 (AG 16.0, DG 2.5), Exp 100us");
        
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


void SpiderCamera::handle_request_complete(libcamera::Request *request) {
    if (request->status() == libcamera::Request::RequestCancelled) {
        LOG_DEBUG("Request was cancelled");
        return;
    }

    if (request->status() != libcamera::Request::RequestComplete) {
        LOG_ERROR("Request failed (status: " << static_cast<int>(request->status()) << "), frame dropped.");
        error_count_++;
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return;
    }

    const std::map<const libcamera::Stream *, libcamera::FrameBuffer *> &buffers = 
        request->buffers();
    
    if (buffers.empty()) {
        LOG_ERROR("No buffers in completed request");
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return;
    }

    libcamera::FrameBuffer *buffer = buffers.begin()->second;
    frame_count_++;
    
    LOG_DEBUG("Frame #" << frame_count_ << ": " << frame_width_ << "x" << frame_height_);

    // --- TIMESTAMPS (З виправленням .id() та *) ---
    const libcamera::ControlList &metadata = request->metadata();
    double timestamp_us = 0.0;
    if (metadata.contains(libcamera::controls::SensorTimestamp.id())) {
        int64_t ts_ns = *metadata.get(libcamera::controls::SensorTimestamp);
        timestamp_us = ts_ns / 1000.0;
        LOG_DEBUG("  Timestamp: " << ts_ns << " ns (" << (timestamp_us / 1000.0) << " ms)");
    }

    // --- FPS logging ---
    if (frame_count_ % 10 == 0) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - fps_start_time_).count();
        if (elapsed > 0) {
            double fps = 10000.0 / elapsed;
            LOG_INFO("Frame rate: " << std::fixed << std::setprecision(1) << fps << " fps");
        }
        fps_start_time_ = now;
    }

    // --- v0.3.9: YUV420 Robust MMAP Handling ---
    if (buffer->planes().size() < 3) {
        LOG_ERROR("YUV: Expected 3 planes, but buffer has " << buffer->planes().size());
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return;
    }

    const libcamera::FrameBuffer::Plane &y_plane = buffer->planes()[0];
    const libcamera::FrameBuffer::Plane &u_plane = buffer->planes()[1];
    const libcamera::FrameBuffer::Plane &v_plane = buffer->planes()[2];

    size_t total_buffer_span = (v_plane.offset - y_plane.offset) + v_plane.length;
    
    void *buffer_base_ptr = mmap(nullptr, total_buffer_span, PROT_READ, MAP_SHARED, 
                                 y_plane.fd.get(), y_plane.offset);

    if (buffer_base_ptr == MAP_FAILED) {
        LOG_ERROR("Failed to mmap YUV buffer (errno " << errno << ")");
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return;
    }

    try {
        const uint8_t *base_ptr = static_cast<const uint8_t*>(buffer_base_ptr);
        
        const uint8_t *y_src_ptr = base_ptr; 
        const uint8_t *u_src_ptr = base_ptr + (u_plane.offset - y_plane.offset);
        const uint8_t *v_src_ptr = base_ptr + (v_plane.offset - y_plane.offset);

        size_t y_size = frame_width_ * frame_height_;
        size_t uv_width = frame_width_ / 2;
        size_t uv_height = frame_height_ / 2;
        size_t uv_size = uv_width * uv_height;
        size_t total_size = y_size + uv_size + uv_size;

        std::vector<uint8_t> frame_data(total_size);

        uint8_t *y_dst_ptr = frame_data.data();
        uint8_t *u_dst_ptr = y_dst_ptr + y_size;
        uint8_t *v_dst_ptr = u_dst_ptr + uv_size;

        // --- Y Plane Copy (stripping stride) ---
        if (frame_stride_y_ == frame_width_) {
            std::memcpy(y_dst_ptr, y_src_ptr, y_size);
        } else {
            LOG_DEBUG("Stripping Y stride: " << frame_stride_y_ << " -> " << frame_width_);
            for (uint32_t i = 0; i < frame_height_; ++i) {
                std::memcpy(y_dst_ptr, y_src_ptr, frame_width_);
                y_src_ptr += frame_stride_y_;
                y_dst_ptr += frame_width_;
            }
        }

        // --- U Plane Copy (stripping stride) ---
        if (frame_stride_uv_ == uv_width) {
            std::memcpy(u_dst_ptr, u_src_ptr, uv_size);
        } else {
            LOG_DEBUG("Stripping U stride: " << frame_stride_uv_ << " -> " << uv_width);
            for (uint32_t i = 0; i < uv_height; ++i) {
                std::memcpy(u_dst_ptr, u_src_ptr, uv_width);
                u_src_ptr += frame_stride_uv_;
                u_dst_ptr += uv_width;
            }
        }

        // --- V Plane Copy (stripping stride) ---
        if (frame_stride_uv_ == uv_width) {
            std::memcpy(v_dst_ptr, v_src_ptr, uv_size);
        } else {
            LOG_DEBUG("Stripping V stride: " << frame_stride_uv_ << " -> " << uv_width);
            for (uint32_t i = 0; i < uv_height; ++i) {
                std::memcpy(v_dst_ptr, v_src_ptr, uv_width);
                v_src_ptr += frame_stride_uv_;
                v_dst_ptr += uv_width;
            }
        }

        munmap(buffer_base_ptr, total_buffer_span);

        {
            std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
            frame_data_buffer_.push_back(std::move(frame_data));
        }

        LOG_DEBUG("YUV frame buffered for burst capture");
        error_count_ = 0; 

    } catch (const std::exception& e) {
        LOG_ERROR("Frame processing failed: " << e.what());
        munmap(buffer_base_ptr, total_buffer_span);
    }

    request->reuse(libcamera::Request::ReuseBuffers);
    camera_->queueRequest(request);
}


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
