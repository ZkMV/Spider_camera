# SpiderCamera 🕷️📷

**High-performance RAW camera capture library for Raspberry Pi**

SpiderCamera is a C++ wrapper around libcamera designed for high-speed RAW frame capture with Python bindings via pybind11. It targets Raspberry Pi boards and provides in-memory burst capture with planned support for hot parameter updates and GPIO triggers.

## Features

* 🎥 **High-speed YUV420 streaming** (for real-time analysis, bypassing slow RAW decompression).
* 🐍 Python interface via pybind11.
* 🚀 **~14 fps burst capture performance** at **3840x2400**.
* 💾 In-memory "burst" frame buffering via `get_burst_frames()`.
* ✅ **Hot parameter changes** (ISO, exposure, resolution, focus) via Python setters.
* ✅ **Per-frame GPIO hardware trigger** support (for flash/strobe sync)..
## Requirements

* Raspberry Pi with libcamera support
* Python 3.11+
* `libcamera-dev`
* `libgpiod-dev` (for v0.4+)
* `pybind11`
* `opencv-python-headless` (for processing/saving frames in Python)

## Quick Start
SpiderCamera/
├── include/
│   └── spider_camera.hpp        # C++ API interface
├── src/
│   ├── spider_camera.cpp        # Core implementation
│   ├── pisp_decompress.cpp      # (DEPRECATED: Old v0.2 RAW logic)
│   └── frame_buffer.cpp         # (DEPRECATED: Old v0.2 RAW logic)
├── bindings/
│   └── pybind_spider.cpp        # Python bindings (pybind11)
├── examples/
│   ├── demo_python.py           # Test script for v0.3 (YUV Burst)
│   └── demo_config.json         # v0.3 config file (ISO, Exp, Res, Focus)
├── build.sh                     # Build script
└── Specification.md             # Technical specification
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
    * A per-frame GPIO trigger fires *exactly* in sync with each frame's exposure, activating external photometric lighting.

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

* `go()` — start YUV frame capture (state: `1 → 2`).
* `pause()` — pause streaming, keep camera configured (state: `2 → 1`).
* `get_burst_frames() -> list[np.ndarray]` — **(NEW in v0.2.8)** return all buffered frames as a list of NumPy arrays; performs decompression in C++ before passing frames to Python.
* `set_frame_callback(callback)` — **deprecated in v0.2.8**; not recommended for high-speed capture. Prefer `get_burst_frames()`.

### Hot Parameters (v0.3)

*Implementation: These setters store values which are applied during `be_ready()` and `go()`.*

* `set_iso(iso: int)` — Sets target ISO (e.g., 4000). C++ calculates `AnalogueGain`.
* `set_exposure(us: int)` — Sets shutter time in microseconds (e.g., 100).
* `set_resolution(w: int, h: int)` — Sets target resolution (e.g., 3840, 2400).
* `set_focus(value: float)` — Sets manual focus value (e.g., 0.0 for infinity).

### GPIO Trigger (v0.4)

**Status:** ✅ Complete and tested

**Features:**
* `set_frame_trigger_pin(pin: int)`: Configures the BCM GPIO pin for output (e.g., pin 21 on `gpiochip4` for RPi 5).
* `enable_frame_trigger(enabled: bool)`: Enables/disables the trigger logic.
* **Trigger Logic:** Generates a precise HIGH/LOW pulse for each frame.
    * **GPIO LOW:** Встановлюється на *початку* `handle_request_complete` (кінець експозиції кадру N).
    * **GPIO HIGH:** Встановлюється в *кінці* `handle_request_complete` (безпосередньо перед постановкою в чергу кадру N+1).
* **Implementation:** Uses the stable C-API (`gpiod.h`) for `libgpiod` to ensure compatibility and avoid "undefined symbol" linking errors.

## Project Structure

```text
SpiderCamera/
├── include/
│   └── spider_camera.hpp        # C++ API interface
├── src/
│   ├── spider_camera.cpp        # Core implementation
│   ├── pisp_decompress.cpp      # (DEPRECATED: Old v0.2 RAW logic)
│   └── frame_buffer.cpp         # (DEPRECATED: Old v0.2 RAW logic)
├── bindings/
│   └── pybind_spider.cpp        # Python bindings (pybind11)
├── examples/
│   ├── demo_python.py           # CLI Burst Capture (Headless)
│   ├── demo_python_preview.py   # Headless Preview (saves to temp/spider_preview.jpg)
│   ├── spider_preview_gui.py    # GUI Control (PyQt5) with real-time preview
│   └── demo_config.json         # Shared configuration file
├── build.sh                     # Build script
└── Specification.md             # Technical specification
```

## Demo & Testing Tools

The library includes three Python scripts to verify functionality and performance:

1.  **`demo_python.py` (Burst Benchmark)**
    * **Use case:** Performance testing and headless capture.
    * **Action:** Loads config, captures a high-speed burst (e.g., 1.5s), calculates actual FPS, and saves frames to `temp/`.
    * **Run:** `python3 examples/demo_python.py`

2.  **`demo_python_preview.py` (Headless Preview)**
    * **Use case:** Monitoring the feed without a display attached.
    * **Action:** Continuously fetches frames and updates a single JPEG file (`temp/spider_preview.jpg`). Useful for checking focus/framing via a file browser or web server.
    * **Run:** `python3 examples/demo_python_preview.py`

3.  **`spider_preview_gui.py` (GUI Control Panel)**
    * **Use case:** Interactive configuration and real-time feedback.
    * **Features:**
        * Live viewfinder (~2 FPS visualization).
        * **Hot-swappable parameters:** Sliders for ISO, Exposure, and Focus.
        * Resolution switcher (triggers soft-restart).
        * Saves current settings to `demo_config.json`.
    * **Run:** `python3 examples/spider_preview_gui.py`
    * *Requires: `python3-pyqt5`*


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

**Status:** ✅ Complete and tested

**Features:**
* set_frame_trigger_pin() and enable_frame_trigger() implementation.
* Hardware sync: GPIO HIGH on frame start, LOW on frame end.
* Switched to C-API (libgpiod) for stability on RPi 5.

### v0.5 — GUI & Stability (2025-11-20)
**Status:** ✅ Complete and tested

**Features:**
* PyQt5 GUI: Created spider_preview_gui.py for real-time parameter tuning.
* Stability Fix: Fixed race conditions in stop() method to prevent library crashes during rapid parameter changes.
* Headless Preview: Added demo_python_preview.py for continuous monitoring.
* YUV Pipeline: Fully transitioned to YUV420 for high-speed burst capture (replacing raw decompression).

### v1.0 — Full Release (planned)

To be defined.

## Performance

* **Target:** ~14 fps RAW capture at high resolution.
* **Result (v0.2):** ~13–14 fps at 4608×2592 (12 MP).
* Latency: minimal; frames are copied in-memory in C++ during capture.
* Decompression time: ~0.28 s for 13 × 12 MP frames (post-capture, on reference hardware).
* Format: PISP_COMP1 (8-bit compressed) → 10-bit Bayer RAW.

## Limitations (current v0.4)

* ❌ Getters (`get_iso`, etc.) are still stubs.

## Roadmap

* [x] v0.1 — Basic initialization
* [x] v0.2 — Frame streaming (burst capture)
* [x] v0.3 — Hot parameters
* [x] v0.4 — GPIO trigger
* [x] v0.5 — GUI Preview & Stability Optimization. WIP: Needs better synchronization debugging in real operating conditions. 
	Considering on-the-fly software adjustment of phase shift and extending light frames by several camera frames.
* [ ] v1.0 — Production release


## Installation & Requirements
```
Since PyQt5 on Raspberry Pi (ARM) is best installed via the system package manager to avoid compilation errors, the installation is split into two steps.
```
### Step 1: System Dependencies (via apt)
Install core C++ libraries and Qt5 system packages:
```
sudo apt update
sudo apt install -y libcamera-dev libgpiod-dev python3-pyqt5 python3-pyqt5.qtquick libqt5gui5 libqt5widgets5
```

**Important:** To allow your virtual environment to use the system-installed PyQt5, you must enable access to system site-packages:
1. Open your venv configuration: nano .venv/pyvenv.cfg
2. Change include-system-site-packages = false to:

```
include-system-site-packages = true
```
**Step 2:** Python Dependencies (via pip)
Install the remaining Python packages (NumPy, OpenCV headless, pybind11) using the provided requirements file:
```
source .venv/bin/activate
pip install -r requirements.txt
```

**Step 3:** Build the Library
Run the build script to compile the C++ code. This will generate the spider_camera.cpython-*.so module directly in the project root directory.

```
chmod +x build.sh
./build.sh
```
	**Note:** You do not need to move the generated .so file. The scripts in the examples/ folder are pre-configured to find the spider_camera module in the project root directory automatically.

## License



## Author



## Acknowledgments


