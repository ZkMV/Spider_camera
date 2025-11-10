/*
 * spider_camera.hpp
 *
 * Header file for the SpiderCamera library (v0.2.8 - Burst Capture).
 * Defines the main class for camera control and state management.
 */

#ifndef SPIDER_CAMERA_HPP
#define SPIDER_CAMERA_HPP

#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/pixel_format.h>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

#include "pisp_decompress.hpp"
#include "frame_buffer.hpp"

#include <memory>
#include <functional>
#include <string>
#include <thread>
#include <atomic>
#include <mutex> // <--- v0.2.8: Додано
#include <chrono>
#include <vector> // <--- v0.2.8: Додано

namespace py = pybind11;

class SpiderCamera {
public:
    /**
     * @brief Constructor. Initializes the CameraManager.
     */
    SpiderCamera();

    /**
     * @brief Destructor. Ensures the camera is stopped and released.
     */
    ~SpiderCamera();

    // --- v0.1 Methods ---
    void set_cam(int cam_id);
    int get_cam() const;
    void be_ready();
    void stop();
    int get_state() const;

    // --- v0.2 Methods ---
    void go();
    void pause();
    
    /**
     * @brief [v0.2.8: НЕ ВИКОРИСТОВУЄТЬСЯ в режимі 'burst']
     * Registers the Python callback function to receive frames.
     */
    void set_frame_callback(std::function<void(py::array, double)> callback);
    
    void enable_debug(bool enable);

    // --- v0.2.8: Новий метод ---
    /**
     * @brief Processes all buffered frames, decompresses them,
     * and returns them as a list of NumPy arrays.
     * @return A py::list of NumPy arrays (uint16).
     */
    py::list get_burst_frames();

    // --- v0.3+ Methods (stubs) ---
    void set_iso(int iso) { /* Not implemented yet */ }
    int get_iso() const { return 0; }
    void set_exposure(int exposure_us) { /* Not implemented yet */ }
    int get_exposure() const { return 0; }
    void set_resolution(int w, int h) { /* Not implemented yet */ }
    py::tuple get_resolution() const { return py::make_tuple(0, 0); }
    void set_focus(int focus) { /* Not implemented yet */ }
    int get_focus() const { return 0; }

    // --- v0.4+ Methods (stubs) ---
    void set_spider_trigger(bool enable) { /* Not implemented yet */ }
    bool get_spider_trigger() const { return false; }
    void set_spider_gpio(int pin) { /* Not implemented yet */ }
    int get_spider_gpio() const { return -1; }

private:
    std::unique_ptr<libcamera::CameraConfiguration> create_raw_config();
    void release_camera();
    void allocate_buffers();
    void create_requests();
    
    /**
     * @brief [v0.2.8: Змінено] Signal handler. Now copies compressed 
     * frame data into the burst buffer instead of decompressing.
     */
    void handle_request_complete(libcamera::Request *request);
    
    void stream_loop();
    py::array convert_to_numpy(libcamera::FrameBuffer *buffer); // (Deprecated)

    // --- Member Variables ---
    std::atomic<int> state_{0};
    int active_camera_id_ = -1;

    std::shared_ptr<libcamera::CameraManager> cam_mgr_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;

    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    
    std::unique_ptr<std::thread> stream_thread_;
    std::atomic<bool> streaming_{false};
    std::atomic<bool> debug_enabled_{false};
    
    std::function<void(py::array, double)> frame_callback_;
    std::mutex callback_mutex_;
    
    std::atomic<int> error_count_{0};
    std::atomic<uint64_t> frame_count_{0};
    std::chrono::steady_clock::time_point fps_start_time_;
    
    uint32_t frame_width_ = 0;
    uint32_t frame_height_ = 0;
    libcamera::PixelFormat current_pixel_format_;

    // v0.2.3 -> v0.2.8: 
    // Буфер для розпаковки (використовується в get_burst_frames)
    std::vector<uint16_t> decompression_buffer_;

    // --- v0.2.8: Нові змінні для Burst Capture ---
    
    /**
     * @brief Buffer to store raw, compressed frame data
     * (e.g., PISP_COMP1 8-bit data) during 'go()'.
     */
    std::vector<std::vector<uint8_t>> compressed_frame_buffer_;
    
    /**
     * @brief Mutex to protect 'compressed_frame_buffer_' from 
     * race conditions (camera thread vs python thread).
     */
    std::mutex burst_buffer_mutex_;
};

#endif // SPIDER_CAMERA_HPP
