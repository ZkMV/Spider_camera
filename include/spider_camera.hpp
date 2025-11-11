/*
 * spider_camera.hpp
 *
 * Header file for the SpiderCamera library (v0.3.7 - Stride Fix).
 */

#ifndef SPIDER_CAMERA_HPP
#define SPIDER_CAMERA_HPP

#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/pixel_format.h>
#include <libcamera/formats.h>

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
    SpiderCamera();
    ~SpiderCamera();

    void set_cam(int cam_id);
    int get_cam() const;
    void be_ready();
    void stop();
    int get_state() const;

    void go();
    void pause();
    py::list get_burst_frames();
    void set_frame_callback(std::function<void(py::array, double)> callback);
    void enable_debug(bool enable);

    // --- v0.3.7: Нова функція для Python ---
    /**
     * @brief Gets the configured stream properties.
     * @return tuple (width, height, pixel_format_str)
     */
    py::tuple get_frame_properties();

    // --- Stubs ---
    void set_iso(int iso) { /* Not implemented yet */ }
    int get_iso() const { return 0; }
    void set_exposure(int exposure_us) { /* Not implemented yet */ }
    int get_exposure() const { return 0; }
    void set_resolution(int w, int h) { /* Not implemented yet */ }
    py::tuple get_resolution() const { return py::make_tuple(0, 0); }
    void set_focus(int focus) { /* Not implemented yet */ }
    int get_focus() const { return 0; }

private:
    std::unique_ptr<libcamera::CameraConfiguration> create_capture_config();
    void release_camera();
    void allocate_buffers();
    void create_requests();
    void handle_request_complete(libcamera::Request *request);
    void stream_loop();
    py::array convert_to_numpy(libcamera::FrameBuffer *buffer); // (Deprecated)

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
    
    // --- v0.3.7: Змінні для Stride ---
    uint32_t frame_width_ = 0;
    uint32_t frame_height_ = 0;
    uint32_t frame_stride_y_ = 0;  // Stride для Y-площини
    uint32_t frame_stride_uv_ = 0; // Stride для U/V площин
    libcamera::PixelFormat current_pixel_format_;

    std::vector<std::vector<uint8_t>> frame_data_buffer_;
    std::mutex frame_buffer_mutex_;
};

#endif // SPIDER_CAMERA_HPP
