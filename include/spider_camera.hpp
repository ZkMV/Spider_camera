/*
 * spider_camera.hpp
 *
 * Header file for the SpiderCamera library (v0.4.3 - C-API GPIO Fix).
 * v0.4.3: Switched from gpiod.hpp (C++ wrapper) to gpiod.h (C API)
 * to fix "undefined symbol" linking errors.
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

// 🎯 v0.4.3: ПЕРЕМИКАЄМОСЬ НА C API
#include <gpiod.h>

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

    // --- Core Methods ---
    void set_cam(int cam_id);
    int get_cam() const;
    void be_ready();
    void stop();
    int get_state() const;

    // --- Streaming Methods ---
    void go();
    void pause();
    py::list get_burst_frames();
    void set_frame_callback(std::function<void(py::array, double)> callback);
    void enable_debug(bool enable);
    py::tuple get_frame_properties();

    // --- v0.3: Hot Parameters ---
    void set_iso(int iso);
    void set_exposure(int exposure_us);
    void set_focus(float focus_value);
    void set_resolution(int w, int h);
    
    // --- Stubs (Getters) ---
    int get_iso() const { return 0; } 
    int get_exposure() const { return 0; }
    float get_focus() const { return 0.0f; }
    py::tuple get_resolution() const { return py::make_tuple(0, 0); }
    
    // --- v0.4: GPIO Functions ---
    void set_frame_trigger_pin(int pin_num);
    void enable_frame_trigger(bool enable);


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
    
    uint32_t frame_width_ = 0;
    uint32_t frame_height_ = 0;
    uint32_t frame_stride_y_ = 0;
    uint32_t frame_stride_uv_ = 0;
    libcamera::PixelFormat current_pixel_format_;

    std::vector<std::vector<uint8_t>> frame_data_buffer_;
    std::mutex frame_buffer_mutex_;
    
    // --- v0.3 Variables ---
    int exposure_us_ = 100;
    float focus_value_ = 0.0f;
    float total_gain_ = 40.0f;
    uint32_t target_width_ = 0;
    uint32_t target_height_ = 0;
    const float BASE_ISO_ = 100.0f; 
    
    // =======================================================
    // 🎯 v0.4.3: ЗМІНЮЄМО ТИПИ ЗМІННИХ НА C-ВКАЗІВНИКИ
    // =======================================================
    std::atomic<bool> trigger_enabled_{false};
    int trigger_pin_num_ = -1;
    struct gpiod_chip *gpio_chip_ = nullptr;
    struct gpiod_line *gpio_trigger_line_ = nullptr;
};

#endif // SPIDER_CAMERA_HPP
