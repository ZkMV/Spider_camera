/*
 * pybind_spider.cpp
 *
 * Pybind11 wrapper definitions for the SpiderCamera library.
 * v0.3.11: Updated bindings for implemented Hot Parameters.
 */

#include "spider_camera.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(spider_camera, m) {
    m.doc() = "SpiderCamera: A C++ libcamera wrapper for high-speed capture (v0.3.11 YUV)";

    py::class_<SpiderCamera>(m, "SpiderCamera")
        .def(py::init<>(), "Initializes the camera manager")

        // --- Core Methods ---
        .def("set_cam", &SpiderCamera::set_cam, py::arg("cam_id"), "Selects the active camera")
        .def("get_cam", &SpiderCamera::get_cam, "Gets the ID of the currently selected camera")
        .def("be_ready", &SpiderCamera::be_ready, "Acquires and configures the camera (state: 0 -> 1)")
        .def("stop", &SpiderCamera::stop, "Stops all activity and releases the camera (state -> 0)")
        .def("get_state", &SpiderCamera::get_state, "Gets the current camera state")

        // --- v0.2+ (Burst) Methods ---
        .def("go", &SpiderCamera::go, "Starts the frame capture stream (state: 1 -> 2)")
        .def("pause", &SpiderCamera::pause, "Pauses the frame capture stream (state: 2 -> 1)")
        .def("set_frame_callback", &SpiderCamera::set_frame_callback, "Register callback (not used in burst mode)")
        .def("enable_debug", &SpiderCamera::enable_debug, py::arg("enable"), "Enable or disable debug logging")
        .def("get_burst_frames", &SpiderCamera::get_burst_frames, "Get all buffered frames (YUV data)")
        .def("get_frame_properties", &SpiderCamera::get_frame_properties,
             "Returns (width, height, format_string) of the stream")

        // =======================================================
        // 🎯 v0.3: ОНОВЛЕНІ БІНДИНГИ
        // =======================================================
        .def("set_iso", &SpiderCamera::set_iso, 
             py::arg("iso"), "Sets the target ISO value")
        .def("get_iso", &SpiderCamera::get_iso, "Get current ISO value (stub)")
        
        .def("set_exposure", &SpiderCamera::set_exposure, 
             py::arg("exposure_us"), "Sets the exposure time in microseconds")
        .def("get_exposure", &SpiderCamera::get_exposure, "Get current exposure time (stub)")
        
        .def("set_focus", &SpiderCamera::set_focus, 
             py::arg("focus_value"), "Sets the manual focus value (0.0 = infinity)")
        .def("get_focus", &SpiderCamera::get_focus, "Get current focus value (stub)")

        // --- Stubs ---
        .def("set_resolution", &SpiderCamera::set_resolution, py::arg("width"), py::arg("height"), "Set frame resolution (stub)")
        .def("get_resolution", &SpiderCamera::get_resolution, "Get current frame resolution (stub)");
}
