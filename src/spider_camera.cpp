/*
 * spider_camera.cpp
 *
 * Implementation file for the SpiderCamera library (v0.1).
 * Contains the business logic for camera management.
 */

#include "spider_camera.hpp"
#include <libcamera/pixel_format.h>
#include <iostream>

// Anonymous namespace for internal logging helpers
namespace {
    void log_error(const std::string& msg) {
        std::cerr << "[SpiderCamera ERROR] " << msg << std::endl;
    }
    void log_info(const std::string& msg) {
        std::cout << "[SpiderCamera INFO] " << msg << std::endl;
    }
}

SpiderCamera::SpiderCamera() : state_(0), active_camera_id_(-1) {
    cam_mgr_ = std::make_shared<libcamera::CameraManager>();
    int ret = cam_mgr_->start();
    if (ret) {
        log_error("Failed to start CameraManager: " + std::to_string(ret));
        state_ = 4; // Error state
    } else {
        log_info("CameraManager started.");
        if (cam_mgr_->cameras().empty()) {
            log_error("No cameras found.");
            state_ = 4;
        }
    }
}

SpiderCamera::~SpiderCamera() {
    log_info("Shutting down...");
    stop(); // Ensure camera is released
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
    log_info("Camera selected: " + camera_->id());
}

int SpiderCamera::get_cam() const {
    return active_camera_id_;
}

void SpiderCamera::release_camera() {
    if (camera_) {
        if (state_ > 0) { // If ready or streaming
            try {
                camera_->release();
                log_info("Camera released: " + camera_->id());
            } catch (const std::exception& e) {
                log_error("Exception during camera release: " + std::string(e.what()));
            }
        }
        config_.reset(); // Delete the configuration object
        // camera_ is reset by set_cam or destructor, but not here
    }
}

void SpiderCamera::stop() {
    if (state_ == 0) return; // Already stopped

    log_info("Stopping camera...");
    release_camera();
    
    // Reset state
    camera_.reset(); // Release shared_ptr
    active_camera_id_ = -1;
    state_ = 0;
    log_info("Camera stopped and state reset to 0.");
}

std::unique_ptr<libcamera::CameraConfiguration> SpiderCamera::create_raw_config() {
    // Generate a configuration for the "Raw" stream role
    auto config = camera_->generateConfiguration({libcamera::StreamRole::Raw});
    if (!config) {
        throw std::runtime_error("Failed to generate configuration.");
    }

    // Get the first (and only) stream configuration
    libcamera::StreamConfiguration &stream_config = config->at(0);
    log_info("Generated base config for stream role 'Raw'.");

    // --- Find a suitable RAW pixel format ---
    // For v0.1, we use the first available RAW format
    const libcamera::StreamFormats &formats = stream_config.formats();
    const auto &pix_formats = formats.pixelformats();

    if (pix_formats.empty()) {
        throw std::runtime_error("No pixel formats available for RAW stream.");
    }

    // Use first available format (libcamera will suggest the best one)
    libcamera::PixelFormat target_format = pix_formats[0];
    log_info("Using PixelFormat: " + target_format.toString());

    stream_config.pixelFormat = target_format;

    // --- Set Resolution ---
    // For v0.1, we just use the first (default) resolution.
    // v0.3 will add set_resolution()
    const auto &sizes = stream_config.formats().sizes(target_format);
    if (sizes.empty()) {
        throw std::runtime_error("No sizes available for the selected format.");
    }
    
    stream_config.size = sizes[0];
    log_info("Set Resolution to: " + std::to_string(stream_config.size.width) + 
             "x" + std::to_string(stream_config.size.height));

    // --- Validate the configuration ---
    if (config->validate() == libcamera::CameraConfiguration::Invalid) {
        throw std::runtime_error("Failed to validate the created RAW configuration.");
    }
    log_info("Configuration validated successfully.");

    return config;
}


void SpiderCamera::be_ready() {
    if (!camera_) {
        throw std::runtime_error("No camera selected. Call set_cam() first.");
    }
    if (state_ != 0) {
        throw std::runtime_error("Camera is already active. Call stop() first.");
    }

    log_info("Moving to 'ready' state (be_ready)...");
    try {
        // 1. Acquire the camera
        if (camera_->acquire()) {
            throw std::runtime_error("Failed to acquire camera.");
        }
        log_info("Camera acquired: " + camera_->id());

        // 2. Create and set the RAW configuration
        config_ = create_raw_config();
        
        if (camera_->configure(config_.get()) < 0) {
            throw std::runtime_error("Failed to configure camera.");
        }
        log_info("Camera configured for RAW streaming.");

        // 3. Set state to "ready"
        state_ = 1;
        log_info("Camera is READY (state 1).");

    } catch (const std::exception& e) {
        log_error("Failed during be_ready(): " + std::string(e.what()));
        state_ = 4; // Error state
        release_camera(); // Clean up
    }
}

int SpiderCamera::get_state() const {
    return state_;
}

void SpiderCamera::set_frame_callback(std::function<void(py::array)> callback) {
    log_info("Python frame callback registered.");
    frame_callback_ = callback;
}
