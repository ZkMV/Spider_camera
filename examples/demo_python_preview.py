#!/usr/bin/env python3
# examples/demo_python_preview.py
"""
demo_python_preview.py – headless preview for SpiderCamera (v0.6 Stride Aware).

Purpose:
- Reads config from demo_config.json.
- Initializes SpiderCamera (be_ready once).
- Starts stream (go) once.
- Loop:
    * Fetches burst frames via get_burst_frames().
    * Takes the last frame.
    * HANDLES STRIDE/PADDING properly (v0.6 fix).
    * Converts YUV -> BGR.
    * Saves to <root>/temp/spider_preview.jpg.

Note: Uses headless OpenCV (no imshow).
"""

import sys
import os
import time
import json

import cv2
import numpy as np

# -----------------------------
# Settings
# -----------------------------

PREVIEW_POLL_SEC = 0.5  # ~2 FPS for preview updates
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_config.json")

# -----------------------------
# Helpers
# -----------------------------

def load_config():
    """Reads demo_config.json."""
    print("=== SpiderCamera Preview (headless) ===\n")
    print(f"Config file: {CONFIG_FILE}")

    try:
        with open(CONFIG_FILE, "r") as f:
            config = json.load(f)
    except Exception as e:
        print(f"❌ Error loading {CONFIG_FILE}: {e}")
        sys.exit(1)

    print("✓ Loaded config:")
    print(f"  ISO:        {config.get('iso')}")
    print(f"  Exposure:   {config.get('exposure_us')} us")
    print(f"  Resolution: {config.get('resolution')}\n")
    return config


def apply_camera_settings(cam, config):
    """Applies ISO, Exposure, Focus, Resolution to the camera object."""
    cam.set_iso(int(config.get("iso", 100)))
    cam.set_exposure(int(config.get("exposure_us", 1000)))
    cam.set_focus(float(config.get("focus_value", 1.0)))

    res_str = config.get("resolution", "")
    try:
        w_str, h_str = res_str.split("x")
        cam.set_resolution(int(w_str), int(h_str))
    except Exception:
        print(f"⚠️ Invalid resolution '{res_str}', using defaults.")


def prepare_opencv_format(cam):
    """
    Calls get_frame_properties() (v0.6 signature).
    Returns:
        width, height, stride, yuv_height, color_cvt_code
    """
    # 🎯 v0.6 Change: unpack 4 values
    width, height, pixel_format_str, stride = cam.get_frame_properties()
    
    print(f"Got Frame Properties:")
    print(f"  Dims:   {width}x{height}")
    print(f"  Stride: {stride} bytes (Hardware Padding)")
    print(f"  Format: {pixel_format_str}")

    color_cvt_code = None
    # For YUV420/NV12, total vertical height is 1.5x
    yuv_height = int(height * 1.5)

    if pixel_format_str == "YUV420":
        color_cvt_code = cv2.COLOR_YUV2BGR_I420
        print("✓ Using I420 (Planar) decoder.")
    elif pixel_format_str == "NV12":
        color_cvt_code = cv2.COLOR_YUV2BGR_NV12
        print("✓ Using NV12 (Semi-planar) decoder.")
    else:
        print(f"⚠️ Unsupported pixel format: {pixel_format_str}")

    return width, height, stride, yuv_height, color_cvt_code


def save_preview_frame(bgr_image, path):
    """Saves BGR frame to JPEG."""
    try:
        success, encoded = cv2.imencode(".jpg", bgr_image)
        if success:
            with open(path, "wb") as f:
                f.write(encoded.tobytes())
    except Exception as e:
        print(f"⚠️ Error saving preview: {e}")


# -----------------------------
# Main Loop
# -----------------------------

def show_preview():
    config = load_config()
    cam = None

    # Setup Project Root
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if project_root not in sys.path:
        sys.path.append(project_root)

    preview_dir = os.path.join(project_root, "temp")
    os.makedirs(preview_dir, exist_ok=True)
    preview_path = os.path.join(preview_dir, "spider_preview.jpg")

    try:
        import spider_camera
    except ImportError as e:
        print(f"❌ Error importing spider_camera: {e}")
        sys.exit(1)

    try:
        cam = spider_camera.SpiderCamera()
        cam.enable_debug(False)
        cam.set_cam(0)

        apply_camera_settings(cam, config)

        print("Starting camera (be_ready)...")
        cam.be_ready()

        # 🎯 v0.6: Get stride info
        width, height, stride, yuv_height, color_cvt_code = prepare_opencv_format(cam)
        
        if color_cvt_code is None:
            return

        print("Starting streaming (go)...")
        cam.go()

        print(f"Preview Loop Running. Image: {preview_path}")
        print("Ctrl+C to stop.\n")

        while True:
            loop_start = time.time()

            # Fetch latest frames
            frame_list = cam.get_burst_frames()

            if frame_list:
                # Take the last (most recent) frame
                flat_frame = frame_list[-1]

                # 🎯 v0.6 Stride Handling Logic
                try:
                    # 1. Calculate how many rows fit in this buffer given the hardware stride
                    rows_in_buffer = flat_frame.size // stride
                    
                    # 2. Reshape to (Rows, Stride). 
                    # We limit data to rows_in_buffer * stride to avoid size mismatch errors
                    view_2d = flat_frame[:rows_in_buffer*stride].reshape((rows_in_buffer, stride))

                    # 3. Crop the actual image data (remove padding on right, remove extra rows)
                    # We need 'yuv_height' rows and 'width' columns
                    if rows_in_buffer >= yuv_height:
                         # np.ascontiguousarray is CRITICAL for OpenCV to work correctly with a sliced view
                        yuv_image = np.ascontiguousarray(view_2d[:yuv_height, :width])

                        # 4. Convert YUV -> BGR
                        bgr_image = cv2.cvtColor(yuv_image, color_cvt_code)
                        
                        if bgr_image is not None:
                            save_preview_frame(bgr_image, preview_path)
                    else:
                        print(f"⚠️ Incomplete frame: got {rows_in_buffer} rows, needed {yuv_height}")

                except Exception as e:
                    print(f"⚠️ Processing error: {e}")

            # FPS Control
            loop_time = time.time() - loop_start
            sleep_time = max(0.0, PREVIEW_POLL_SEC - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n⚠️ Interrupted.")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if cam is not None:
                cam.stop()
        except:
            pass
        print("Preview finished.")

if __name__ == "__main__":
    show_preview()
