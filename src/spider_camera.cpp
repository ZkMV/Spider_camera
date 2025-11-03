/*
 * spider_camera.cpp
 *
 * Implementation file for the SpiderCamera library (v0.2).
 * Contains the business logic for camera management and streaming.
 */

#include "spider_camera.hpp"
#include <libcamera/pixel_format.h>
#include <iostream>
#include <iomanip>
#include <sys/mman.h>
#include <unistd.h>
#include <cerrno>

// Logging macros
#define LOG_INFO(msg)   std::cout << "[INFO]  " << msg << std::endl
#define LOG_DEBUG(msg)  if (debug_enabled_) std::cout << "[DEBUG] " << msg << std::endl
#define LOG_WARN(msg)   std::cerr << "[WARN]  " << msg << std::endl
#define LOG_ERROR(msg)  std::cerr << "[ERROR] " << msg << std::endl

// Unpacking utilities - inline in anonymous namespace
namespace {

void unpack_raw10_csi2p(const uint8_t* packed, 
                        uint16_t* unpacked, 
                        size_t width, 
                        size_t height) {
    if (!packed || !unpacked) {
        throw std::invalid_argument("Null pointer passed to unpack_raw10_csi2p");
    }

    size_t pixel_count = width * height;

    // Process 4 pixels at a time (5 bytes)
    for (size_t i = 0; i < pixel_count; i += 4) {
        size_t byte_idx = (i * 5) / 4;

        // Extract 4 pixels from 5 bytes
        unpacked[i + 0] = (static_cast<uint16_t>(packed[byte_idx + 0]) << 2) | 
                         ((packed[byte_idx + 4] >> 0) & 0x3);
        
        unpacked[i + 1] = (static_cast<uint16_t>(packed[byte_idx + 1]) << 2) | 
                         ((packed[byte_idx + 4] >> 2) & 0x3);
        
        unpacked[i + 2] = (static_cast<uint16_t>(packed[byte_idx + 2]) << 2) | 
                         ((packed[byte_idx + 4] >> 4) & 0x3);
        
        unpacked[i + 3] = (static_cast<uint16_t>(packed[byte_idx + 3]) << 2) | 
                         ((packed[byte_idx + 4] >> 6) & 0x3);
    }
}

size_t calculate_raw10_packed_size(size_t width, size_t height) {
    size_t pixel_count = width * height;
    return (pixel_count * 10 + 7) / 8;
}

bool validate_raw10_buffer_size(size_t buffer_size, size_t width, size_t height) {
    size_t expected = calculate_raw10_packed_size(width, height);
    return buffer_size >= expected;
}

} // anonymous namespace


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
    
    // Stop streaming if active
    if (state_ == 2) {
        streaming_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Detach thread if exists
    if (stream_thread_) {
        if (stream_thread_->joinable()) {
            stream_thread_->detach();
        }
        stream_thread_.reset();
    }
    
    // Stop camera if running
    try {
        if (camera_ && state_ > 0) {
            camera_->stop();
        }
    } catch (...) {
        // Ignore errors during shutdown
    }
    
    // Release camera
    stop();
    
    // Stop camera manager
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
    // Stop streaming if active
    if (streaming_) {
        pause();
    }
    
    if (camera_) {
        if (state_ > 0) { // If ready or streaming
            try {
                // Free requests first
                requests_.clear();
                
                // Free allocator
                allocator_.reset();
                
                // Release camera
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
    
    // Stop streaming if active
    if (state_ == 2) {
        streaming_ = false;
        if (stream_thread_ && stream_thread_->joinable()) {
            stream_thread_->detach();
        }
        stream_thread_.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    // Try to stop camera with timeout protection
    try {
        // Launch stop in separate thread with timeout
        std::atomic<bool> stop_done{false};
        std::thread stop_thread([this, &stop_done]() {
            try {
                camera_->stop();
                stop_done = true;
            } catch (...) {}
        });
        
        // Wait max 500ms
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

    // Find suitable RAW pixel format
    const libcamera::StreamFormats &formats = stream_config.formats();
    const auto &pix_formats = formats.pixelformats();

    if (pix_formats.empty()) {
        throw std::runtime_error("No pixel formats available for RAW stream.");
    }

    // Use first available format
    libcamera::PixelFormat target_format = pix_formats[0];
    LOG_INFO("Using PixelFormat: " << target_format.toString());

    stream_config.pixelFormat = target_format;

    // Set Resolution
    const auto &sizes = stream_config.formats().sizes(target_format);
    if (sizes.empty()) {
        throw std::runtime_error("No sizes available for the selected format.");
    }
    
    stream_config.size = sizes[0];
    LOG_INFO("Set Resolution to: " << stream_config.size.width << "x" 
             << stream_config.size.height);

    // Validate configuration
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

    // Get the stream
    libcamera::Stream *stream = config_->at(0).stream();
    
    // Create allocator
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    
    // Allocate buffers for the stream
    int ret = allocator_->allocate(stream);
    if (ret < 0) {
        throw std::runtime_error("Failed to allocate buffers: " + std::to_string(ret));
    }
    
    LOG_INFO("Allocated " << ret << " buffers for stream");
    
    // Store frame dimensions
    const libcamera::StreamConfiguration &cfg = config_->at(0);
    frame_width_ = cfg.size.width;
    frame_height_ = cfg.size.height;
}

void SpiderCamera::create_requests() {
    if (!camera_ || !allocator_) {
        throw std::runtime_error("Camera not ready for request creation");
    }

    // Get the stream
    libcamera::Stream *stream = config_->at(0).stream();
    
    // Get allocated buffers
    const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = 
        allocator_->buffers(stream);
    
    // Create requests (up to 4 for queue)
    size_t request_count = std::min(buffers.size(), size_t(4));
    
    for (size_t i = 0; i < request_count; ++i) {
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request) {
            throw std::runtime_error("Failed to create request");
        }
        
        // Associate buffer with request
        int ret = request->addBuffer(stream, buffers[i].get());
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
        // 1. Acquire the camera
        if (camera_->acquire()) {
            throw std::runtime_error("Failed to acquire camera.");
        }
        LOG_INFO("Camera acquired: " << camera_->id());

        // 2. Create and set the RAW configuration
        config_ = create_raw_config();
        
        if (camera_->configure(config_.get()) < 0) {
            throw std::runtime_error("Failed to configure camera.");
        }
        LOG_INFO("Camera configured for RAW streaming");

        // 3. Allocate buffers (v0.2)
        allocate_buffers();

        // 4. Create requests (v0.2)
        create_requests();

        // 5. Connect requestCompleted signal (v0.2)
        camera_->requestCompleted.connect(this, &SpiderCamera::handle_request_complete);
        LOG_DEBUG("Connected requestCompleted signal");

        // 6. Set state to "ready"
        state_ = 1;
        LOG_INFO("Camera is READY (state 1)");

    } catch (const std::exception& e) {
        LOG_ERROR("Failed during be_ready(): " << e.what());
        state_ = 4; // Error state
        release_camera();
    }
}

void SpiderCamera::go() {
    if (state_ != 1) {
        throw std::runtime_error("Camera must be in ready state (1). Current state: " + 
                               std::to_string(state_.load()));
    }

    LOG_INFO("Starting streaming...");
    
    try {
        // DON'T stop camera - it causes deadlock
        // Camera will handle restart automatically
        
        // Reset requests
        for (auto &request : requests_) {
            try {
                request->reuse(libcamera::Request::ReuseBuffers);
            } catch (...) {
                // Ignore
            }
        }
        
        // Start camera (it handles restart internally)
        int ret = camera_->start();
        if (ret && ret != -EACCES) {  // Ignore "already running"
            throw std::runtime_error("Failed to start camera: " + std::to_string(ret));
        }

        // Queue all initial requests
        for (auto &request : requests_) {
            camera_->queueRequest(request.get());
        }
        
        // Start streaming flag and thread
        streaming_ = true;
        frame_count_ = 0;
        error_count_ = 0;
        fps_start_time_ = std::chrono::steady_clock::now();
        
        stream_thread_ = std::make_unique<std::thread>(&SpiderCamera::stream_loop, this);
        
        state_ = 2;
        LOG_INFO("Streaming started (state 2)");

    } catch (const std::exception& e) {
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
    
    streaming_ = false;
    
    if (stream_thread_) {
        if (stream_thread_->joinable()) {
            stream_thread_->detach();
        }
        stream_thread_.reset();
    }
    
    // Longer delay to let requests complete
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    state_ = 1;
    LOG_INFO("Streaming paused (state 1)");
}

void SpiderCamera::stream_loop() {
    LOG_DEBUG("Stream loop started");
    
    while (streaming_) {
        // Simple sleep-based loop
        // The actual work happens in handle_request_complete callback
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
        LOG_ERROR("Request failed: frame dropped");
        error_count_++;
        
        // Requeue request anyway
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return;
    }

    // Get the buffer
    const std::map<const libcamera::Stream *, libcamera::FrameBuffer *> &buffers = 
        request->buffers();
    
    if (buffers.empty()) {
        LOG_ERROR("No buffers in completed request");
        return;
    }

    libcamera::FrameBuffer *buffer = buffers.begin()->second;
    
    frame_count_++;
    
    LOG_DEBUG("Frame #" << frame_count_ << ": " << frame_width_ << "x" << frame_height_);

    // FPS logging every 10 frames
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

	// Convert to NumPy and call Python callback
    try {
        // Спочатку отримуємо RAW дані БЕЗ NumPy
        // (тут буде спрощена версія convert_to_numpy)
        
        // Отримуємо plane
        const libcamera::FrameBuffer::Plane &plane = buffer->planes()[0];
        
        // Map buffer
        void *data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, 
                         plane.fd.get(), 0);
        
        if (data == MAP_FAILED) {
            LOG_ERROR("Failed to mmap buffer");
            error_count_++;
            request->reuse(libcamera::Request::ReuseBuffers);
            camera_->queueRequest(request);
            return;
        }
        
        // Unpack RAW10 to vector
        size_t pixel_count = frame_width_ * frame_height_;
        std::vector<uint16_t> unpacked(pixel_count);
        
        const uint8_t* packed = static_cast<const uint8_t*>(data);
        size_t out_idx = 0;
        
        for (size_t i = 0; i < pixel_count && out_idx < pixel_count; i += 4) {
            size_t byte_idx = (i * 5) / 4;
            if (byte_idx + 4 >= plane.length) break;
            
            unpacked[out_idx++] = (static_cast<uint16_t>(packed[byte_idx + 0]) << 2) | 
                                 ((packed[byte_idx + 4] >> 0) & 0x3);
            if (out_idx >= pixel_count) break;
            unpacked[out_idx++] = (static_cast<uint16_t>(packed[byte_idx + 1]) << 2) | 
                                 ((packed[byte_idx + 4] >> 2) & 0x3);
            if (out_idx >= pixel_count) break;
            unpacked[out_idx++] = (static_cast<uint16_t>(packed[byte_idx + 2]) << 2) | 
                                 ((packed[byte_idx + 4] >> 4) & 0x3);
            if (out_idx >= pixel_count) break;
            unpacked[out_idx++] = (static_cast<uint16_t>(packed[byte_idx + 3]) << 2) | 
                                 ((packed[byte_idx + 4] >> 6) & 0x3);
        }
        
        munmap(data, plane.length);
        
        // Визначаємо реальні розміри
        size_t actual_height = out_idx / frame_width_;
        size_t actual_width = frame_width_;
        size_t actual_pixels = actual_height * actual_width;
        
        LOG_DEBUG("Unpacked " << actual_pixels << " pixels (" << actual_width << "x" << actual_height << ")");
        
        // ТЕПЕР захоплюємо GIL і створюємо NumPy
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (frame_callback_) {
                py::gil_scoped_acquire gil;
                
                // Створюємо NumPy array ТУТ, де є GIL
                py::array_t<uint16_t> arr(
                    {static_cast<py::ssize_t>(actual_height), 
                     static_cast<py::ssize_t>(actual_width)}
                );
                
                auto buf = arr.request();
                uint16_t* ptr = static_cast<uint16_t*>(buf.ptr);
                std::memcpy(ptr, unpacked.data(), actual_pixels * sizeof(uint16_t));
                
                // Викликаємо callback
                frame_callback_(arr);
                error_count_ = 0;
            }
        }
        
        LOG_DEBUG("Callback sent to Python");
        
    } catch (const py::error_already_set& e) {
        LOG_ERROR("Python callback failed: " << e.what());
        error_count_++;
        
        if (error_count_ >= 10) {
            LOG_ERROR("Too many callback errors (10+), pausing stream");
            streaming_ = false;
            state_ = 4;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Frame conversion failed: " << e.what());
    }

    // Requeue the request
    request->reuse(libcamera::Request::ReuseBuffers);
    camera_->queueRequest(request);
}

int SpiderCamera::get_state() const {
    return state_;
}

void SpiderCamera::set_frame_callback(std::function<void(py::array)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    frame_callback_ = callback;
    LOG_INFO("Python frame callback registered");
}

void SpiderCamera::enable_debug(bool enable) {
    debug_enabled_ = enable;
    LOG_INFO("Debug logging " << (enable ? "enabled" : "disabled"));
}
