/*
 * spider_camera.hpp
 *
 * Header file for the SpiderCamera library (v0.1).
 * Defines the main class for camera control and state management.
 */

#ifndef SPIDER_CAMERA_HPP
#define SPIDER_CAMERA_HPP

#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
//#include <libcamera/configuration.h>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h> // For std::function callback
#include <pybind11/numpy.h>     // For pybind11::array

#include <memory>
#include <functional>
#include <string>

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

    // --- v0.2+ Methods (defined for API completeness) ---

    /**
     * @brief Registers the Python callback function to receive frames.
     * @param callback A Python function (or callable) that accepts one
     * argument (the frame data as a numpy array).
     */
    void set_frame_callback(std::function<void(py::array)> callback);


    // --- v0.3+ Methods (stubs for v0.1) ---
    // These will be implemented in v0.3
    void set_iso(int iso) { /* Not implemented in v0.1 */ }
    int get_iso() const { return 0; }
    void set_exposure(int exposure_us) { /* Not implemented in v0.1 */ }
    int get_exposure() const { return 0; }
    void set_resolution(int w, int h) { /* Not implemented in v0.1 */ }
    py::tuple get_resolution() const { return py::make_tuple(0, 0); }
    void set_focus(int focus) { /* Not implemented in v0.1 */ }
    int get_focus() const { return 0; }

    // --- v0.4+ Methods (stubs for v0.1) ---
    void set_spider_trigger(bool enable) { /* Not implemented in v0.1 */ }
    bool get_spider_trigger() const { return false; }
    void set_spider_gpio(int pin) { /* Not implemented in v0.1 */ }
    int get_spider_gpio() const { return -1; }

    // --- v0.2+ Methods (stubs for v0.1) ---
    void go() { /* Not implemented in v0.1 */ }
    void pause() { /* Not implemented in v0.1 */ }


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

    // --- Member Variables ---

    int state_ = 0; // 0=off, 1=ready, 2=streaming, 4=error
    int active_camera_id_ = -1;

    std::shared_ptr<libcamera::CameraManager> cam_mgr_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;

    std::function<void(py::array)> frame_callback_;
};

#endif // SPIDER_CAMERA_HPP
