/*
 * pybind_spider.cpp
 *
 * Pybind11 wrapper definitions for the SpiderCamera library.
 * This file creates the Python module 'spider_camera'.
 * v0.2.8: Added get_burst_frames() binding
 */

#include "spider_camera.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/functional.h> // For set_frame_callback
#include <pybind11/stl.h>       // For std::vector, std::string, py::list

namespace py = pybind11;

PYBIND11_MODULE(spider_camera, m) {
    m.doc() = "SpiderCamera: A C++ libcamera wrapper for high-speed RAW capture";

    py::class_<SpiderCamera>(m, "SpiderCamera")
        .def(py::init<>(), "Initializes the camera manager")

        // --- v0.1 Methods ---
        .def("set_cam", &SpiderCamera::set_cam, 
             py::arg("cam_id"), 
             "Selects the active camera by its index (e.g., 0)")
        
        .def("get_cam", &SpiderCamera::get_cam, 
             "Gets the ID of the currently selected camera")

        .def("be_ready", &SpiderCamera::be_ready, 
             "Acquires and configures the camera. Sets state to 1 (ready)")

        .def("stop", &SpiderCamera::stop, 
             "Stops all activity, releases the camera, and resets state to 0 (off)")

        .def("get_state", &SpiderCamera::get_state, 
             "Gets the current camera state (0=off, 1=ready, 2=streaming, 4=error)")

        // --- v0.2+ Methods ---
        .def("go", &SpiderCamera::go, 
             "Starts the frame capture stream")
        
        .def("pause", &SpiderCamera::pause, 
             "Pauses the frame capture stream")
		
        .def("set_frame_callback", &SpiderCamera::set_frame_callback,
			"Register callback (not used in burst mode)")
		
        .def("enable_debug", &SpiderCamera::enable_debug,
             py::arg("enable"),
             "Enable or disable debug logging")

        // --- v0.3 Methods (Stubs) ---
        .def("set_iso", &SpiderCamera::set_iso, py::arg("iso"), "Set ISO value (0-4000)")
        .def("get_iso", &SpiderCamera::get_iso, "Get current ISO value")
        .def("set_exposure", &SpiderCamera::set_exposure, py::arg("exposure_us"), "Set exposure time in microseconds")
        .def("get_exposure", &SpiderCamera::get_exposure, "Get current exposure time in microseconds")
        .def("set_resolution", &SpiderCamera::set_resolution, py::arg("width"), py::arg("height"), "Set frame resolution")
        .def("get_resolution", &SpiderCamera::get_resolution, "Get current frame resolution (width, height)")
        .def("set_focus", &SpiderCamera::set_focus, py::arg("focus"), "Set focus distance (0-20)")
        .def("get_focus", &SpiderCamera::get_focus, "Get current focus distance")

        // --- v0.4 Methods (Stubs) ---
        .def("set_spider_trigger", &SpiderCamera::set_spider_trigger, py::arg("enable"), "Enable or disable the GPIO trigger mode")
        .def("get_spider_trigger", &SpiderCamera::get_spider_trigger, "Get current GPIO trigger mode state")
        .def("set_spider_gpio", &SpiderCamera::set_spider_gpio, py::arg("pin"), "Set the GPIO pin number for the trigger")
        .def("get_spider_gpio", &SpiderCamera::get_spider_gpio, "Get the current GPIO pin number")

        // --- v0.2.8: НОВА ПРИВ'ЯЗКА ---
        .def("get_burst_frames", &SpiderCamera::get_burst_frames,
             "Get all buffered frames and decompress them (v0.2.8)");
}
