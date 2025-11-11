/*
 * pybind_spider.cpp
 *
 * Pybind11 wrapper definitions for the SpiderCamera library.
 * v0.3.7: Added get_frame_properties() binding.
 */

#include "spider_camera.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(spider_camera, m) {
    m.doc() = "SpiderCamera: A C++ libcamera wrapper for high-speed capture (v0.3.7 YUV)";

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

        // --- v0.3.7: НОВА ПРИВ'ЯЗКА ---
        .def("get_frame_properties", &SpiderCamera::get_frame_properties,
             "Returns (width, height, format_string) of the stream")

        // --- v0.3 Methods (Stubs) ---
        .def("set_iso", &SpiderCamera::set_iso, py::arg("iso"), "Set ISO value (0-4000)")
        .def("get_iso", &SpiderCamera::get_iso, "Get current ISO value")
        .def("set_exposure", &SpiderCamera::set_exposure, py::arg("exposure_us"), "Set exposure time in microseconds")
        .def("get_exposure", &SpiderCamera::get_exposure, "Get current exposure time in microseconds")
        .def("set_resolution", &SpiderCamera::set_resolution, py::arg("width"), py::arg("height"), "Set frame resolution")
        .def("get_resolution", &SpiderCamera::get_resolution, "Get current frame resolution (width, height)")
        .def("set_focus", &SpiderCamera::set_focus, py::arg("focus"), "Set focus distance (0-20)")
        .def("get_focus", &SpiderCamera::get_focus, "Get current focus distance");
}
