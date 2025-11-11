# SpiderCamera 🕷️📷

**High-performance RAW camera capture library for Raspberry Pi**

SpiderCamera is a C++ wrapper around libcamera designed for high-speed RAW frame capture with Python bindings via pybind11. It targets Raspberry Pi boards and provides in-memory burst capture with planned support for hot parameter updates and GPIO triggers.

## Features

* 🎥 **High-speed YUV420 streaming** (for real-time analysis, bypassing slow RAW decompression).
* 🐍 Python interface via pybind11.
* 🚀 **~14 fps burst capture performance** at **3840x2400**.
* 💾 In-memory "burst" frame buffering via `get_burst_frames()`.
* ✅ **Hot parameter changes** (ISO, exposure, resolution, focus) via Python setters.
* 🔌 **Per-frame GPIO hardware trigger** support (for flash/strobe sync) — **planned for v0.4**.
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


## Usage Example (v0.3 Hot Parameters)

```python
import time
import spider_camera
import json

# --- 1. Load Configuration ---
# (e.g., from examples/demo_config.json)
with open("examples/demo_config.json", 'r') as f:
    config = json.load(f)

# --- 2. Initialize & Configure ---
cam = spider_camera.SpiderCamera()
cam.set_cam(0)

# Set parameters BEFORE calling be_ready()
cam.set_iso(config['iso'])
cam.set_exposure(config['exposure_us'])
cam.set_focus(config['focus_value'])
w, h = map(int, config['resolution'].split('x'))
cam.set_resolution(w, h)

# Prepare camera. This will now use the settings above.
cam.be_ready()

# --- 3. Capture ---
cam.go()
time.sleep(1.0) # Capture for 1 second
cam.pause()

# --- 4. Retrieve ---
# Frames are YUV420 format (e.g., 3840x2400)
frame_list = cam.get_burst_frames()
print(f"Captured {len(frame_list)} frames.")

# --- 5. Process (see Application Pipeline) ---
# (User code for OpenCV conversion, saving, or analysis)

# --- 6. Shutdown ---
cam.stop()

## Application Pipeline: Super-Resolution & Photometry

The primary goal of this library is to enable advanced surface analysis by combining high-speed burst capture with synchronized lighting.

The intended pipeline consists of two stages:

1.  **Capture Stage (Real-time, ~1 sec):**
    * The camera captures a high-speed burst (~14 fps) of `YUV 3840x2400` frames while in motion.
    * (v0.4) A per-frame GPIO trigger fires *exactly* in sync with each frame's exposure, activating external photometric lighting.

2.  **Processing Stage (Offline, ~5-10 sec):**
    * The captured burst of ~14 frames is passed to Python for analysis.
    * **Task A (Reference Frame):** A subset of frames (e.g., 4-6) are fed into a **Multi-Frame Super-Resolution (MFSR)** algorithm. The slight (sub-pixel) motion between frames provides the necessary data to reconstruct a single, high-detail reference image (e.g., `~7680x4800`).
    * **Task B (Photometry):** The full set of synchronized frames is fed into a neural network, which analyzes the precise light reflections (photometry) to detect surface inequalities and defects.


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

### Hot Parameters (v0.3)

*Implementation: These setters store values which are applied during `be_ready()` and `go()`.*

* `set_iso(iso: int)` — Sets target ISO (e.g., 4000). C++ calculates `AnalogueGain`.
* `set_exposure(us: int)` — Sets shutter time in microseconds (e.g., 100).
* `set_resolution(w: int, h: int)` — Sets target resolution (e.g., 3840, 2400).
* `set_focus(value: float)` — Sets manual focus value (e.g., 0.0 for infinity).

### GPIO Trigger (v0.4+, planned)

**Status:** 📋 Not started

**Planned features:**

* `set_frame_trigger_pin(pin)` / `get_frame_trigger_pin()` — configure GPIO pin for per-frame trigger.
* `enable_frame_trigger(enabled: bool)` / `get_frame_trigger()` — enable/disable the per-frame trigger.
* **Trigger Logic:** When enabled, the specified GPIO pin will be pulsed (HIGH/LOW) for each individual frame captured.
* **Use Case:** This allows for precise, microsecond-level synchronization of external lighting (strobes, flashes) with the camera's sensor exposure.
* **Implementation:** This will be achieved by connecting to `libcamera`'s internal signals:
  * `requestIssued` signal → Set GPIO **HIGH** (just before exposure starts)
  * `requestCompleted` signal → Set GPIO **LOW** (just after frame is captured)

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

### v0.3 — Hot Parameters (2025-11-11)

**Status:** ✅ Complete and tested

**Features:**
* Implemented Python setters: `set_iso`, `set_exposure`, `set_focus`, `set_resolution`.
* Parameters are now passed from Python (e.g., read from a JSON config) to C++.
* Refactored `go()` to use these variables instead of hard-coded values.
* Corrected gain logic: now uses `AeEnable=false` with full `AnalogueGain` (e.g., 40.0) and `DigitalGain=1.0`, successfully "exorcising" the auto-exposure "poltergeist".
* Confirmed stable manual exposure at `100us` (verified with/without external light).

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

## Limitations (current v0.3)

* ❌ No GPIO trigger support yet (this is v0.4).
* ❌ Getters (`get_iso`, etc.) are still stubs.

## Roadmap

* [x] v0.1 — Basic initialization
* [x] v0.2 — Frame streaming (burst capture)
* [x] v0.3 — Hot parameters
* [ ] v0.4 — GPIO trigger
* [ ] v0.5 — Performance optimization (CSI2P support)
* [ ] v1.0 — Production release

## License



## Author



## Acknowledgments


