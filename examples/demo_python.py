#!/usr/bin/env python3
"""
demo_python.py - v0.3.9 (Burst Test)

Tests the full burst capture pipeline.
- Captures for a set duration.
- Saves all frames to a specified directory.
"""

import sys
import os
import time
import cv2
import numpy as np

if 'spider_camera' in sys.modules:
    del sys.modules['spider_camera']

# Визначаємо корінь проєкту, щоб додати його до шляху Python
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.append(project_root)

try:
    import spider_camera
except ImportError as e:
    print(f"Error importing spider_camera: {e}")
    print(f"Project root (added to path): {project_root}")
    sys.exit(1)

# ============================================
# НАЛАШТУВАННЯ
# ============================================

# Тривалість захоплення в секундах
CAPTURE_DURATION_SEC = 2.0 

# 🎯 ВИПРАВЛЕНО: Шлях для збереження кадрів
SAVE_PATH = os.path.join(project_root, "temp") # Зберігаємо в папку /temp всередині проєкту

# ============================================

def main():
    print(f"=== SpiderCamera Burst Test (v0.3.9) ===\n")
    print(f"Capture Duration: {CAPTURE_DURATION_SEC:.1f} seconds")
    print(f"Will save images to: {SAVE_PATH}")
    
    # Створюємо папку для збереження одразу
    try:
        os.makedirs(SAVE_PATH, exist_ok=True)
        print(f"✓ Created output directory: {SAVE_PATH}")
    except PermissionError:
        print(f"❌ PERMISSION ERROR: Cannot create directory {SAVE_PATH}.")
        print(f"  Please check permissions or choose a different path (e.g., in your home dir).")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error creating directory: {e}")
        sys.exit(1)

    
    cam = None
    try:
        cam = spider_camera.SpiderCamera()
        
        # Вмикаємо debug-логі, щоб бачити пропуск кадрів
        cam.enable_debug(True) 
        
        cam.set_cam(0)
        
        print("\nStarting camera (be_ready)...")
        cam.be_ready() 
        print(f"State: {cam.get_state()} (Ready)\n")
        
        # Отримуємо властивості кадру
        width, height, pixel_format_str = cam.get_frame_properties()
        print(f"Got Frame Properties: {width}x{height}, Format: {pixel_format_str}\n")
        
        # Визначаємо формат для OpenCV
        if pixel_format_str == "YUV420":
            yuv_height = int(height * 1.5)
            color_cvt_code = cv2.COLOR_YUV2BGR_I420
            print("✓ Using I420 (Planar) decoder.")
            expected_size = width * yuv_height
        elif pixel_format_str == "NV12":
            yuv_height = int(height * 1.5)
            color_cvt_code = cv2.COLOR_YUV2BGR_NV12
            print("✓ Using NV12 (Semi-Planar) decoder.")
            expected_size = width * yuv_height
        else:
            print(f"❌ Unknown format: {pixel_format_str}. Cannot save frames.")
            color_cvt_code = None
            expected_size = 0
            
        print(f"\nCapturing burst for {CAPTURE_DURATION_SEC:.1f} seconds...")
        
        cam.go()
        print(f"State: {cam.get_state()} (Streaming)")
        
        start_time = time.time()
        time.sleep(CAPTURE_DURATION_SEC)
        
        print("Pausing stream...")
        cam.pause()
        end_time = time.time()
        
        print("Retrieving frame data from C++...")
        start_copy = time.time()
        frame_list = cam.get_burst_frames()
        end_copy = time.time()
        
        total_time = end_time - start_time
        frame_count = len(frame_list)
        fps = frame_count / total_time if total_time > 0 else 0
        
        print(f"\n=== Capture Results ===")
        print(f"Frames Captured: {frame_count}")
        print(f"Actual Capture Time: {total_time:.2f}s")
        print(f"C++ to Python Copy Time: {end_copy - start_copy:.3f}s")
        print(f"**Average Capture FPS: {fps:.2f}**")
        
        # Очікувана кількість кадрів = (Час - Час_розігріву) * FPS
        # (час розігріву ~5 кадрів / 14 FPS = ~0.35s)
        expected_frames = (total_time - 0.35) * 14 
        print(f"(Expected ~{expected_frames:.0f} frames)")
        
        
        # Збереження кадрів
        if frame_list and color_cvt_code is not None:
            print(f"\nSaving {len(frame_list)} frames to {SAVE_PATH}...")
            
            save_start_time = time.time()
            
            for i, flat_frame in enumerate(frame_list):
                try:
                    # Перевірка розміру
                    if flat_frame.size != expected_size:
                        print(f"⚠️  Frame {i+1}: size mismatch! Got {flat_frame.size}, expected {expected_size}")
                        continue
                    
                    # Reshape YUV data
                    yuv_image = flat_frame.reshape((yuv_height, width))
                    
                    # Convert YUV → BGR
                    bgr_image = cv2.cvtColor(yuv_image, color_cvt_code)
                    
                    # Save
                    filename = f"frame_{i + 1:03d}.jpg"
                    filepath = os.path.join(SAVE_PATH, filename)
                    cv2.imwrite(filepath, bgr_image)
                    
                    if i == 0:
                        print(f"✓ First frame saved: {filepath}")
                        print(f"  YUV shape: {yuv_image.shape}, dtype: {yuv_image.dtype}")
                        print(f"  BGR shape: {bgr_image.shape}, range: [{bgr_image.min()}, {bgr_image.max()}]")
                
                except Exception as e:
                    print(f"❌ Error saving frame {i+1}: {e}")

            save_end_time = time.time()
            print(f"\n✓ Processed and saved {len(frame_list)} frames in {save_end_time - save_start_time:.2f}s")
        
        elif not frame_list:
            print("❌ No frames were captured!")
        else:
            print("❌ Cannot save frames (unknown format)")
        
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if cam and cam.get_state() > 0:
            try:
                if cam.get_state() == 2:
                    cam.pause()
                cam.stop()
            except Exception as e:
                print(f"Error during shutdown: {e}")
        print("\nTest complete.")

if __name__ == "__main__":
    main()
