/*
 * spider_camera.hpp
 *
 * Header file for the SpiderCamera library (v0.2).
 * Defines the main class for camera control and state management.
 */

#ifndef SPIDER_CAMERA_HPP
#define SPIDER_CAMERA_HPP

#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

#include <memory>
#include <functional>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <vector>

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

    /**
     * @brief Selects the active camera by its index.
     * Must be called when state is 0 (off).
     * @param cam_id The index of the camera (e.g., 0).
     */
    void set_cam(int cam_id);

    /**
     * @brief Gets the ID of the currently selected camera.
     * @return Camera ID, or -1 if none is selected.
     */
    int get_cam() const;

    /**
     * @brief Prepares the camera for streaming (v0.1 state 1).
     * Acquires the camera, creates a RAW configuration, and validates it.
     */
    void be_ready();

    /**
     * @brief Stops all activity and releases the camera.
     * Returns the state to 0 (off).
     */
    void stop();

    /**
     * @brief Gets the current state of the camera.
     * 0 = off
     * 1 = ready
     * 2 = streaming (for v0.2)
     * 4 = error
     * @return The current state code.
     */
    int get_state() const;

    // --- v0.2 Methods ---

    /**
     * @brief Starts the frame capture stream (state: 1 → 2).
     */
    void go();

    /**
     * @brief Pauses the frame capture stream (state: 2 → 1).
     */
    void pause();

    /**
     * @brief Registers the Python callback function to receive frames.
     * @param callback A Python function (or callable) that accepts one
     * argument (the frame data as a numpy array).
     */
    void set_frame_callback(std::function<void(py::array)> callback);

    /**
     * @brief Enable or disable debug logging.
     * @param enable True to enable debug logs
     */
    void enable_debug(bool enable);

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
    /**
     * @brief Helper function to create a default RAW stream configuration.
     * @return A unique_ptr to the generated configuration.
     */
    std::unique_ptr<libcamera::CameraConfiguration> create_raw_config();

    /**
     * @brief Helper function to safely release the camera and config.
     */
    void release_camera();

    /**
     * @brief Allocates frame buffers for the configured stream.
     */
    void allocate_buffers();

    /**
     * @brief Creates Request objects and associates them with buffers.
     */
    void create_requests();

    /**
     * @brief Signal handler called when a request is completed.
     * @param request The completed request
     */
    void handle_request_complete(libcamera::Request *request);

    /**
     * @brief Thread function for streaming loop.
     */
    void stream_loop();

    /**
     * @brief Converts a RAW10 FrameBuffer to NumPy array.
     * @param buffer The frame buffer to convert
     * @return NumPy array with uint16 Bayer data
     */
    py::array convert_to_numpy(libcamera::FrameBuffer *buffer);

    // --- Member Variables ---

    std::atomic<int> state_{0}; // 0=off, 1=ready, 2=streaming, 4=error
    int active_camera_id_ = -1;

    std::shared_ptr<libcamera::CameraManager> cam_mgr_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;

    // v0.2 additions
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    
    std::unique_ptr<std::thread> stream_thread_;
    std::atomic<bool> streaming_{false};
    std::atomic<bool> debug_enabled_{false};
    
    std::function<void(py::array)> frame_callback_;
    std::mutex callback_mutex_;
    
    std::atomic<int> error_count_{0};
    std::atomic<uint64_t> frame_count_{0};
    std::chrono::steady_clock::time_point fps_start_time_;
    
    // Frame dimensions (set during configuration)
    uint32_t frame_width_ = 0;
    uint32_t frame_height_ = 0;
};

#endif // SPIDER_CAMERA_HPP
