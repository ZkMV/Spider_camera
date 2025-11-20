#!/usr/bin/env python3
# spider_preview_gui.py
"""
SpiderCamera GUI Preview Application.

Features:
- Live preview (~2 FPS) from SpiderCamera.
- Real-time parameter adjustment (ISO, Exposure, Focus, Resolution).
- Multithreaded architecture (GUI + CameraWorker).
- Saves preview frames to temp/spider_preview.jpg.
- Loads/Saves configuration to demo_config.json.
"""

import sys
import os
import json
import time
import cv2
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QSpinBox, QDoubleSpinBox, QComboBox,
    QCheckBox, QPushButton, QGroupBox, QGridLayout, QMessageBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtGui import QImage, QPixmap

# -----------------------------------------------------------------------------
# Path Setup
# -----------------------------------------------------------------------------
# Determine project root to import spider_camera
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR) if os.path.basename(CURRENT_DIR) == "examples" else CURRENT_DIR

if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

try:
    import spider_camera
except ImportError:
    print(f"❌ Error: Could not import 'spider_camera'. Ensure the library is compiled in {PROJECT_ROOT}")
    sys.exit(1)

# -----------------------------------------------------------------------------
# Configuration Constants
# -----------------------------------------------------------------------------
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_config.json")
PREVIEW_POLL_SEC = 0.5
TEMP_DIR = os.path.join(PROJECT_ROOT, "temp")
PREVIEW_FILE = os.path.join(TEMP_DIR, "spider_preview.jpg")

# Ensure temp directory exists
os.makedirs(TEMP_DIR, exist_ok=True)

# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------
def load_config():
    default_config = {
        "iso": 100,
        "exposure_us": 1000,
        "focus_value": 1.0,
        "resolution": "3840x2400"
    }
    if not os.path.exists(CONFIG_FILE):
        return default_config
    try:
        with open(CONFIG_FILE, "r") as f:
            data = json.load(f)
            # Merge with defaults to ensure all keys exist
            for key, value in default_config.items():
                if key not in data:
                    data[key] = value
            return data
    except Exception as e:
        print(f"⚠️ Error loading config: {e}")
        return default_config

def save_config(config_data):
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(config_data, f, indent=4)
        print(f"✓ Configuration saved to {CONFIG_FILE}")
    except Exception as e:
        print(f"❌ Error saving config: {e}")

# -----------------------------------------------------------------------------
# Worker Thread
# -----------------------------------------------------------------------------
class CameraWorker(QThread):
    """
    Handles SpiderCamera interactions in a separate thread to keep GUI responsive.
    Manages the state machine: Ready -> Streaming -> Stop -> Ready.
    """
    # Signals to GUI
    new_frame = pyqtSignal(QImage)
    status_message = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, initial_config):
        super().__init__()
        self.running = True
        self.preview_active = False

        # Internal state
        self.cam = None
        self.cam_state = 0  # 0: Off, 1: Ready, 2: Streaming

        # Parameters
        self.params = {
            "iso": initial_config.get("iso", 100),
            "exposure_us": initial_config.get("exposure_us", 1000),
            "focus": initial_config.get("focus_value", 1.0),
            "resolution": initial_config.get("resolution", "3840x2400")
        }

        # Flags for update logic
        self.params_dirty = True  # Force initial apply
        self.restart_needed = True  # Force initial setup

    def run(self):
        """Main thread loop."""
        try:
            self.cam = spider_camera.SpiderCamera()
            self.cam.set_cam(0)
            # Disable debug to reduce console spam
            self.cam.enable_debug(False)

            # Initial setup
            self._apply_params_to_camera_object()
            self.cam.be_ready()
            self.cam_state = 1
            self.status_message.emit("Camera Ready")

        except Exception as e:
            self.error_occurred.emit(f"Init failed: {str(e)}")
            return

        while self.running:
            # 1. Check for Parameter Updates / Restart Logic
            if self.params_dirty:
                self.status_message.emit("Applying settings...")

                # Need to stop stream to change resolution or re-apply 'be_ready' logic
                was_streaming = (self.cam_state == 2)

                if was_streaming:
                    self.cam.stop()
                    self.cam_state = 0

                # Apply parameters to C++ object
                self._apply_params_to_camera_object()

                # Re-initialize
                if self.cam_state == 0:
                    # FIX: Re-select camera because stop() resets it
                    self.cam.set_cam(0)
                    self.cam.be_ready()
                    self.cam_state = 1

                # Resume stream if it was active or if preview is requested
                if self.preview_active and self.cam_state == 1:
                    self.cam.go()
                    self.cam_state = 2

                self.params_dirty = False
                self.status_message.emit(f"Settings applied. State: {self.cam_state}")

            # 2. Preview Loop logic
            if self.preview_active:
                if self.cam_state == 1:  # If ready but not streaming
                    self.cam.go()
                    self.cam_state = 2

                if self.cam_state == 2:
                    self._fetch_and_process_frame()

                # Sleep to control FPS
                time.sleep(PREVIEW_POLL_SEC)
            else:
                # If preview disabled but camera streaming, stop it to save resources
                if self.cam_state == 2:
                    self.cam.pause()  # or stop()
                    self.cam_state = 1
                    self.status_message.emit("Preview Paused")
                time.sleep(0.1)

        # Cleanup on exit
        if self.cam:
            self.cam.stop()

    def _apply_params_to_camera_object(self):
        """Calls C++ setters based on current self.params dict."""
        try:
            # Resolution parsing
            w, h = map(int, self.params["resolution"].split('x'))
            self.cam.set_resolution(w, h)

            self.cam.set_iso(int(self.params["iso"]))
            self.cam.set_exposure(int(self.params["exposure_us"]))
            self.cam.set_focus(float(self.params["focus"]))

        except Exception as e:
            print(f"⚠️ Error applying params: {e}")

    def _fetch_and_process_frame(self):
        """Fetches frames from C++, converts YUV->RGB, emits signal."""
        try:
            # Get burst frames (list of numpy arrays)
            frames = self.cam.get_burst_frames()

            if not frames:
                return

            # Take the last frame (most recent)
            raw_frame = frames[-1]

            # Get properties to know how to decode
            width, height, fmt = self.cam.get_frame_properties()

            # Determine YUV format
            yuv_height = 0
            code = None

            if fmt == "YUV420":  # I420
                yuv_height = int(height * 1.5)
                code = cv2.COLOR_YUV2BGR_I420
            elif fmt == "NV12":
                yuv_height = int(height * 1.5)
                code = cv2.COLOR_YUV2BGR_NV12
            else:
                return  # Unsupported for preview

            expected_size = width * yuv_height

            if raw_frame.size != expected_size:
                return  # Corrupted or partial frame

            # 1. Reshape
            yuv_img = raw_frame.reshape((yuv_height, width))

            # 2. Convert to BGR (for OpenCV saving)
            bgr_img = cv2.cvtColor(yuv_img, code)

            # 3. Save to disk (Requirement)
            cv2.imwrite(PREVIEW_FILE, bgr_img)

            # 4. Convert to RGB (for Qt)
            rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

            # 5. Create QImage
            # MUST copy data, otherwise underlying buffer might be freed
            h, w, ch = rgb_img.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()

            self.new_frame.emit(qt_image)

        except Exception as e:
            print(f"Frame process error: {e}")

    # --- Public Slots called from GUI ---

    @pyqtSlot(dict)
    def update_params(self, new_params):
        """Receives updated parameters from GUI."""
        self.params = new_params
        self.params_dirty = True  # Trigger restart logic in loop

    @pyqtSlot(bool)
    def set_preview_active(self, active):
        self.preview_active = active

    def stop_thread(self):
        self.running = False
        self.wait()

# -----------------------------------------------------------------------------
# Main GUI Window
# -----------------------------------------------------------------------------
class SpiderPreviewGUI(QMainWindow):
    # Signal to worker to update parameters
    request_param_update = pyqtSignal(dict)

    def __init__(self):
        super().__init__()

        self.setWindowTitle("SpiderCamera Preview & Control")
        self.resize(1000, 800)

        # Load initial config
        self.config = load_config()

        # Thread setup
        self.worker = CameraWorker(self.config)
        self.worker.new_frame.connect(self.update_image_display)
        self.worker.status_message.connect(self.update_status)
        self.worker.error_occurred.connect(self.show_error)
        self.request_param_update.connect(self.worker.update_params)

        # Debounce Timer (prevents restarting camera on every slider tick)
        self.debounce_timer = QTimer()
        self.debounce_timer.setSingleShot(True)
        self.debounce_timer.setInterval(600)  # 600ms delay
        self.debounce_timer.timeout.connect(self.commit_params)

        self.init_ui()

        # Start Worker
        self.worker.start()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main Layout
        layout = QVBoxLayout(central_widget)

        # --- 1. Preview Area ---
        self.image_label = QLabel("Waiting for preview...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: #2b2b2b; color: #888;")
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setSizePolicy(self.image_label.sizePolicy().Expanding,
                                     self.image_label.sizePolicy().Expanding)
        layout.addWidget(self.image_label, stretch=2)

        # --- 2. Controls Area ---
        controls_group = QGroupBox("Camera Controls")
        controls_layout = QGridLayout()

        # Row 0: Preview Toggle & Resolution
        self.chk_preview = QCheckBox("Enable Live Preview")
        self.chk_preview.stateChanged.connect(self.toggle_preview)
        self.chk_preview.setStyleSheet("font-weight: bold; font-size: 14px;")

        self.combo_res = QComboBox()
        resolutions = ["3840x2400", "2304x1296", "1920x1080", "1280x720", "640x480"]
        self.combo_res.addItems(resolutions)
        # Set current index based on config
        idx = self.combo_res.findText(self.config.get("resolution", "3840x2400"))
        if idx >= 0: self.combo_res.setCurrentIndex(idx)
        self.combo_res.currentTextChanged.connect(self.on_param_change)

        controls_layout.addWidget(self.chk_preview, 0, 0)
        controls_layout.addWidget(QLabel("Resolution:"), 0, 1)
        controls_layout.addWidget(self.combo_res, 0, 2)

        # Row 1: ISO
        controls_layout.addWidget(QLabel("ISO:"), 1, 0)
        self.slider_iso = QSlider(Qt.Horizontal)
        self.slider_iso.setRange(1, 4000)
        self.spin_iso = QSpinBox()
        self.spin_iso.setRange(1, 4000)
        self.setup_control_pair(self.slider_iso, self.spin_iso, "iso")
        controls_layout.addWidget(self.slider_iso, 1, 1)
        controls_layout.addWidget(self.spin_iso, 1, 2)

        # Row 2: Exposure
        controls_layout.addWidget(QLabel("Exposure (µs):"), 2, 0)
        self.slider_exp = QSlider(Qt.Horizontal)
        self.slider_exp.setRange(100, 10000)
        self.spin_exp = QSpinBox()
        self.spin_exp.setRange(100, 10000)
        self.setup_control_pair(self.slider_exp, self.spin_exp, "exposure_us")
        controls_layout.addWidget(self.slider_exp, 2, 1)
        controls_layout.addWidget(self.spin_exp, 2, 2)

        # Row 3: Focus (Float handling)
        controls_layout.addWidget(QLabel("Focus (Lens Pos):"), 3, 0)
        self.slider_focus = QSlider(Qt.Horizontal)
        self.slider_focus.setRange(0, 200)  # 0.0 to 20.0 * 10
        self.spin_focus = QDoubleSpinBox()
        self.spin_focus.setRange(0.0, 20.0)
        self.spin_focus.setSingleStep(0.1)
        self.setup_focus_pair(self.slider_focus, self.spin_focus)
        controls_layout.addWidget(self.slider_focus, 3, 1)
        controls_layout.addWidget(self.spin_focus, 3, 2)

        # Add to main layout
        controls_layout.setColumnStretch(1, 1)  # Sliders expand
        controls_group.setLayout(controls_layout)
        layout.addWidget(controls_group)

        # --- 3. Bottom Buttons ---
        btn_layout = QHBoxLayout()
        btn_save = QPushButton("Save Config to JSON")
        btn_save.clicked.connect(self.save_current_config)
        btn_layout.addStretch()
        btn_layout.addWidget(btn_save)
        layout.addLayout(btn_layout)

        # --- 4. Status Bar ---
        self.statusbar = self.statusBar()
        self.statusbar.showMessage("Ready")

    def setup_control_pair(self, slider, spinbox, config_key):
        """Links a Slider and Spinbox together."""
        val = int(self.config.get(config_key, slider.minimum()))
        slider.setValue(val)
        spinbox.setValue(val)

        # Link signals
        slider.valueChanged.connect(spinbox.setValue)
        spinbox.valueChanged.connect(slider.setValue)

        # Link to debounce logic
        slider.valueChanged.connect(self.on_param_change)
        # spinbox valueChanged is triggered by slider, so we don't need to link it explicitly to debounce
        # unless typed manually. To be safe:
        spinbox.editingFinished.connect(self.on_param_change)

    def setup_focus_pair(self, slider, spinbox):
        """Specific linking for Float focus values."""
        val = float(self.config.get("focus_value", 1.0))
        slider.setValue(int(val * 10))
        spinbox.setValue(val)

        def slider_to_spin(v):
            spinbox.blockSignals(True)
            spinbox.setValue(v / 10.0)
            spinbox.blockSignals(False)
            self.on_param_change()

        def spin_to_slider(v):
            slider.blockSignals(True)
            slider.setValue(int(v * 10))
            slider.blockSignals(False)
            self.on_param_change()

        slider.valueChanged.connect(slider_to_spin)
        spinbox.valueChanged.connect(spin_to_slider)

    # --- Logic ---

    def toggle_preview(self, state):
        is_active = (state == Qt.Checked)
        self.worker.set_preview_active(is_active)

    def on_param_change(self):
        """Called when any GUI control changes. Starts Debounce timer."""
        self.debounce_timer.start()

    def commit_params(self):
        """Executed after timer expires. Collects data and sends to worker."""
        new_params = {
            "iso": self.spin_iso.value(),
            "exposure_us": self.spin_exp.value(),
            "focus": self.spin_focus.value(),
            "resolution": self.combo_res.currentText()
        }
        self.request_param_update.emit(new_params)
        self.statusbar.showMessage("Updating camera parameters...")

    def save_current_config(self):
        cfg = {
            "iso": self.spin_iso.value(),
            "exposure_us": self.spin_exp.value(),
            "focus_value": self.spin_focus.value(),
            "resolution": self.combo_res.currentText()
        }
        save_config(cfg)
        self.statusbar.showMessage(f"Config saved to {CONFIG_FILE}", 3000)

    @pyqtSlot(str)
    def update_status(self, msg):
        self.statusbar.showMessage(msg)

    @pyqtSlot(str)
    def show_error(self, msg):
        QMessageBox.critical(self, "Error", msg)

    @pyqtSlot(QImage)
    def update_image_display(self, qt_img):
        """Scales and displays the QImage."""
        # Scale maintaining aspect ratio
        pixmap = QPixmap.fromImage(qt_img)
        scaled_pix = pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image_label.setPixmap(scaled_pix)

    def closeEvent(self, event):
        """Handle app closure."""
        self.worker.stop_thread()
        super().closeEvent(event)

# -----------------------------------------------------------------------------
# Main Entry Point
# -----------------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)

    # Dark Theme Styling (Optional)
    app.setStyle("Fusion")

    window = SpiderPreviewGUI()
    window.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
