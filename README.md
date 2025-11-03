# SpiderCamera 🕷️📷

**High-performance RAW camera capture library for Raspberry Pi**

SpiderCamera is a C++ wrapper around libcamera designed for high-speed RAW frame capture with Python bindings via pybind11. Built specifically for Raspberry Pi with GPIO trigger support.

## Features

- 🎥 RAW format streaming (10-bit/12-bit Bayer)
- 🐍 Python interface via pybind11
- ⚡ Hot parameter changes (ISO, exposure, resolution, focus)
- 🔌 GPIO hardware trigger support
- 🚀 ~14 fps capture performance
- 💾 In-memory frame delivery (no disk writes)

## Requirements

- Raspberry Pi with libcamera support
- Python 3.11+
- libcamera-dev
- libgpiod-dev (for v0.4+)
- pybind11

## Quick Start
```bash
# Clone repository
git clone https://github.com/YOUR_USERNAME/SpiderCamera.git
cd SpiderCamera

# Setup virtual environment
python3 -m venv .venv
source .venv/bin/activate
pip install pybind11 numpy

# Build
chmod +x build.sh
./build.sh

# Test
python3 examples/demo_python.py
```

## Usage Example
```python
import spider_camera

# Initialize
cam = spider_camera.SpiderCamera()
cam.set_cam(0)
cam.be_ready()

# Configure (v0.3+)
cam.set_iso(1600)
cam.set_exposure(10000)  # microseconds
cam.set_resolution(1920, 1080)

# Start streaming (v0.2+)
def on_frame(frame):
    print(f"Frame: {frame.shape}")

cam.set_frame_callback(on_frame)
cam.go()

# Stop
cam.pause()
cam.stop()
```

## API Reference

### Core Methods (v0.1)
- `set_cam(id)` — Select camera by index
- `get_cam()` — Get current camera ID
- `be_ready()` — Prepare camera (state: 0 → 1)
- `stop()` — Stop and release camera (state → 0)
- `get_state()` — Get current state (0=off, 1=ready, 2=streaming, 4=error)

### Streaming Methods (v0.2+)
- `go()` — Start RAW frame capture
- `pause()` — Pause streaming
- `set_frame_callback(callback)` — Register Python callback for frames

### Hot Parameters (v0.3+)
- `set_iso(value)` / `get_iso()` — ISO 0-4000
- `set_exposure(us)` / `get_exposure()` — Exposure time in microseconds
- `set_resolution(w, h)` / `get_resolution()` — Frame resolution
- `set_focus(mm)` / `get_focus()` — Focus distance 0-20mm

### GPIO Trigger (v0.4+)
- `set_spider_gpio(pin)` / `get_spider_gpio()` — Set GPIO pin number
- `set_spider_trigger(bool)` / `get_spider_trigger()` — Enable/disable trigger
  - HIGH = streaming active
  - LOW = paused/stopped

## Project Structure
```
SpiderCamera/
├── include/
│   └── spider_camera.hpp        # C++ API interface
├── src/
│   ├── spider_camera.cpp        # Core implementation
│   ├── camera_controller.cpp    # libcamera management
│   ├── gpio_controller.cpp      # GPIO trigger logic
│   └── frame_buffer.cpp         # Frame buffer handling
├── bindings/
│   └── pybind_spider.cpp        # Python bindings
├── examples/
│   └── demo_python.py           # Test scripts
├── build.sh                     # Build script
└── Specification.md             # Technical specification
```

## Git History

### v0.1 — Basic Initialization (2025-10-31)

**Status:** ✅ Complete and tested

**Features:**
- Camera manager initialization
- Camera selection (`set_cam()`)
- RAW configuration (SRGGB10_CSI2P format)
- State management (0=off, 1=ready, 4=error)
- Resource cleanup (`stop()`)
- Pybind11 Python bindings
- Test suite

**Tested on:**
- Raspberry Pi 5 / BCM2712_D0
- IMX708 camera sensor
- Resolution: 1536x864 @ 10-bit RAW
- libcamera v0.5.2

**Changes:**
```
+ spider_camera.hpp - Core class definition
+ spider_camera.cpp - Implementation
+ pybind_spider.cpp - Python bindings
+ demo_python.py - Test suite
+ build.sh - Compilation script
```

---

### v0.2 — Frame Streaming (planned)

**Status:** 🚧 In development

**Planned Features:**
- `go()` / `pause()` methods
- Request queue management
- Event loop in separate thread
- RAW frame capture to memory
- NumPy array delivery to Python
- Frame callback mechanism
- State 2 (streaming)

---

### v0.3 — Hot Parameters (planned)

**Status:** 📋 Not started

**Planned Features:**
- Live ISO adjustment (0-4000)
- Live exposure control (microseconds)
- Live resolution changes
- Live focus adjustment (0-20mm)
- No camera restart required

---

### v0.4 — GPIO Trigger (planned)

**Status:** 📋 Not started

**Planned Features:**
- GPIO pin configuration
- Hardware trigger mode
- HIGH = streaming active
- LOW = paused/stopped
- libgpiod integration

---

### v1.0 — Full Release (planned)

**Status:** 📋 Not started

**Goals:**
- All features complete
- Full test coverage
- Performance optimization (~14 fps)
- Documentation complete
- Production ready

---

## Development

### Building from Source
```bash
# Install dependencies
sudo apt install libcamera-dev libgpiod-dev python3-dev

# Setup
python3 -m venv .venv
source .venv/bin/activate
pip install pybind11

# Compile
./build.sh

# Run tests
python3 examples/demo_python.py
```

### Geany IDE Setup

**Build command (F9):**
```bash
bash /home/admin/projects/Spider_camera/build.sh
```

**Execute command (F5):**
```bash
cd /home/admin/projects/Spider_camera && source .venv/bin/activate && python3 examples/demo_python.py
```

## Performance

- **Target:** ~14 fps RAW capture
- **Latency:** Minimal (in-memory transfer)
- **Format:** 10-bit/12-bit Bayer RAW
- **Memory:** Zero-copy where possible

## Limitations (v0.1)

- ❌ RAW format only (no JPEG/H.264)
- ❌ No single-shot capture mode
- ❌ No encoding/compression
- ❌ Memory-only delivery (no disk writes)
- ✅ Geany-based compilation (no CMake required)

## License

[Specify your license here]

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Add tests for new features
4. Submit a pull request

## Roadmap

- [x] v0.1 - Basic initialization
- [x] v0.2 - Frame streaming
- [ ] v0.3 - Hot parameters
- [ ] v0.4 - GPIO trigger
- [ ] v0.5 - Performance optimization
- [ ] v1.0 - Production release

## Author

[Your name]

## Acknowledgments

- libcamera team for the excellent camera framework
- pybind11 project for C++/Python bindings
- Raspberry Pi Foundation
