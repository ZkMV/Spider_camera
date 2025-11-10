# SpiderCamera 🕷️📷

**High-performance RAW camera capture library for Raspberry Pi**

SpiderCamera is a C++ wrapper around libcamera designed for high-speed RAW frame capture with Python bindings via pybind11. It targets Raspberry Pi boards and provides in-memory burst capture with planned support for hot parameter updates and GPIO triggers.

## Features

* 🎥 RAW format streaming (10-bit / 12-bit Bayer)
* 🐍 Python interface via pybind11
* 🚀 **~14 fps burst capture performance** at 4608×2592
* 📈 **PISP_COMP1 8-bit RAW decompression** support
* 💾 In-memory "burst" frame buffering via `get_burst_frames()`
* 📋 Hot parameter changes (ISO, exposure, resolution, focus) — **planned for v0.3**
* 🔌 GPIO hardware trigger support — **planned for v0.4**

## Requirements

* Raspberry Pi with libcamera support
* Python 3.11+
* `libcamera-dev`
* `libgpiod-dev` (for v0.4+)
* `pybind11`
* `opencv-python-headless` (for processing/saving frames in Python)

## Quick Start

```bash
# Clone repository
git clone https://github.com/YOUR_USERNAME/SpiderCamera.git
cd SpiderCamera

# Setup virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install pybind11 numpy opencv-python-headless

# Build
chmod +x build.sh
./build.sh

# Test
python3 examples/demo_python.py
```

## Usage Example (v0.2 Burst Capture)

```python
import time
import spider_camera

# Initialize
cam = spider_camera.SpiderCamera()
cam.set_cam(0)

# Prepare camera (e.g. 4056×3040 @ ~14 fps)
cam.be_ready()

# Start capturing
cam.go()

# Capture for 2 seconds
time.sleep(2.0)

# Pause capturing (keep camera ready)
cam.pause()

# Get all captured frames
# This also performs C++-side decompression
frame_list = cam.get_burst_frames()
print(f"Captured {len(frame_list)} frames.")

# TODO: process/save frames in Python (e.g. via OpenCV)

# Stop and release resources
cam.stop()
```

## API Reference

### Core Methods (v0.1)

* `set_cam(id)` — select camera by index.
* `get_cam()` — get current camera ID.
* `be_ready()` — prepare camera (state transition: `0 → 1`).
* `stop()` — stop and release camera (state transition: `* → 0`).
* `get_state()` — get current state:

  * `0` — off
  * `1` — ready
  * `2` — streaming
  * `4` — error

### Streaming Methods (v0.2+)

* `go()` — start RAW frame capture (state: `1 → 2`).
* `pause()` — pause streaming, keep camera configured (state: `2 → 1`).
* `get_burst_frames() -> list[np.ndarray]` — **(NEW in v0.2.8)** return all buffered frames as a list of NumPy arrays; performs decompression in C++ before passing frames to Python.
* `set_frame_callback(callback)` — **deprecated in v0.2.8**; not recommended for high-speed capture. Prefer `get_burst_frames()`.

### Hot Parameters (v0.3+, planned)

*Planned for v0.3 — not yet implemented.*

* `set_iso(value)` / `get_iso()` — control sensor gain.
* `set_exposure(us)` / `get_exposure()` — shutter time in microseconds.
* `set_resolution(w, h)` / `get_resolution()` — change capture resolution.
* `set_focus(mm)` / `get_focus()` — lens focus position (in millimetres or device units).

### GPIO Trigger (v0.4+, planned)

*Planned for v0.4 — not yet implemented.*

* `set_spider_gpio(pin)` / `get_spider_gpio()` — configure GPIO pin used for SpiderCamera trigger.
* `set_spider_trigger(enabled: bool)` / `get_spider_trigger()` — hardware trigger mode:

  * HIGH = streaming
  * LOW  = paused

## Project Structure

```text
SpiderCamera/
├── include/
│   ├── spider_camera.hpp        # C++ API interface
│   └── pisp_decompress.hpp      # Decompressor header
├── src/
│   ├── spider_camera.cpp        # Core implementation
│   ├── pisp_decompress.cpp      # PISP_COMP1 decompressor
│   ├── frame_buffer.cpp         # Frame buffer handling (CSI2P)
│   └── ...                      # Other internal sources
├── bindings/
│   └── pybind_spider.cpp        # Python bindings (pybind11)
├── examples/
│   └── demo_python.py           # Test script (burst capture)
├── build.sh                     # Build script
└── Specification.md             # Technical specification
```

## Git History

### v0.1 — Basic Initialization (2025-10-31)

**Status:** ✅ Complete and tested

**Features:**

* Camera manager initialization.
* Camera selection via `set_cam()`.
* Basic RAW configuration and state management.

### v0.2 — Burst Frame Streaming (2025-11-10)

**Status:** ✅ Complete and tested

**Features:**

* `go()` / `pause()` methods for streaming control.
* New architecture: burst capture mode.
* `get_burst_frames()` — C++-side decompression and delivery of all buffered frames to Python as a list of NumPy arrays.
* PISP_COMP1 8-bit compressed RAW decompression via LUT.
* Increased buffer count (`-bufferCount 8`) to prevent pipeline stalls.
* Performance: stable ~13–14 fps at 4608×2592 (12 MP) resolution.
* Hard-coded capture settings in C++ loop (14 fps, ISO 4000, 100 µs exposure).

### v0.3 — Hot Parameters (planned)

**Status:** 📋 Not started

**Planned features:**

* Implement Python setters (`set_iso`, `set_exposure`, etc.).
* Pass parameters from Python to C++ controls inside the `go()` loop.
* Allow changing parameters "live" while in `pause()` state.

### v0.4 — GPIO Trigger (planned)

**Status:** 📋 Not started

**Planned features:**

* GPIO pin configuration for trigger.
* Hardware trigger mode (HIGH = streaming, LOW = paused).
* `libgpiod` integration.

### v1.0 — Full Release (planned)

To be defined.

## Performance

* **Target:** ~14 fps RAW capture at high resolution.
* **Result (v0.2):** ~13–14 fps at 4608×2592 (12 MP).
* Latency: minimal; frames are copied in-memory in C++ during capture.
* Decompression time: ~0.28 s for 13 × 12 MP frames (post-capture, on reference hardware).
* Format: PISP_COMP1 (8-bit compressed) → 10-bit Bayer RAW.

## Limitations (current v0.2)

* ❌ No hot parameters: capture settings (ISO, exposure, etc.) are hard-coded in C++.
* ❌ No GPIO trigger support yet.
* ❌ RAW format only (no JPEG / H.264 encoding).
* ❌ No single-shot capture mode.
* ❌ Frames are delivered only to Python memory (no disk writes in C++ layer).

## Roadmap

* [x] v0.1 — Basic initialization
* [x] v0.2 — Frame streaming (burst capture)
* [ ] v0.3 — Hot parameters
* [ ] v0.4 — GPIO trigger
* [ ] v0.5 — Performance optimization (CSI2P support)
* [ ] v1.0 — Production release

## License



## Author



## Acknowledgments


