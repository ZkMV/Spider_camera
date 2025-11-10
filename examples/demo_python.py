#!/usr/bin/env python3
"""
demo_python.py - v0.2.14 (Burst Capture + JPEG Save Fix)

Fixes save path to be relative to the project root.
"""

import sys
import os
import time
import cv2
import numpy as np

# Clear module cache
if 'spider_camera' in sys.modules:
    del sys.modules['spider_camera']

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

try:
    import spider_camera
except ImportError as e:
    print(f"Error importing spider_camera: {e}")
    sys.exit(1)

# Configuration
CAPTURE_DURATION_SEC = 2.0 
# === v0.2.14: FIX: Створюємо 'temp' всередині папки проєкту ===
SAVE_PATH = os.path.join(project_root, "temp")
# ========================================================
g_frame_count = 0

def main():
    print(f"=== SpiderCamera Burst Test (v0.2.14) ===\n")
    print(f"Will save images to: {SAVE_PATH}") # <-- Додатковий лог
    
    cam = None
    bayer_pattern_code = None

    try:
        cam = spider_camera.SpiderCamera()
        cam.enable_debug(True) 
        cam.set_cam(0)
        
        print("Starting camera...")
        cam.be_ready() 
        print(f"State: {cam.get_state()} (Ready)\n")
        
        # Визначаємо код дебаєризації (з попередніх логів це BGGR)
        bayer_pattern_code = cv2.COLOR_BAYER_BG2RGB

        print(f"Capturing burst for {CAPTURE_DURATION_SEC:.1f} seconds...")
        
        cam.go()
        print(f"State: {cam.get_state()} (Streaming)\n")
        
        start_time = time.time()
        time.sleep(CAPTURE_DURATION_SEC)
        
        print("Pausing stream...")
        cam.pause()
        end_time = time.time()
        
        print("Capture complete. Decompressing frames...")
        start_decomp = time.time()
        
        frame_list = cam.get_burst_frames()
        
        end_decomp = time.time()
        
        total_time = end_time - start_time
        g_frame_count = len(frame_list)
        fps = g_frame_count / total_time if total_time > 0 else 0
        
        # --- ЕТАП ЗБЕРЕЖЕННЯ КАДРІВ ---
        if frame_list:
            print(f"\nSaving {len(frame_list)} frames to {SAVE_PATH}...")
            
            # Цей виклик тепер спрацює, оскільки він у вашій домашній директорії
            os.makedirs(SAVE_PATH, exist_ok=True) 
            
            save_start_time = time.time()
            
            for i, bayer_frame in enumerate(frame_list):
                # 1. Конвертуємо 10-бітний (0-1023) в 8-бітний (0-255)
                bayer_8bit = (bayer_frame >> 2).astype('uint8')
                
                # 2. Дебаєризація
                rgb_image = cv2.cvtColor(bayer_8bit, bayer_pattern_code)
                
                # 3. Створюємо ім'я файлу (01.jpg, 02.jpg, ...)
                filename = f"{i + 1:02d}.jpg"
                filepath = os.path.join(SAVE_PATH, filename)
                
                # 4. Зберігаємо JPEG
                cv2.imwrite(filepath, rgb_image)

            save_end_time = time.time()
            print(f"Successfully saved frames in {save_end_time - save_start_time:.2f}s")
        # ------------------------------------

        print(f"\n=== Results ===")
        print(f"Frames Captured: {g_frame_count}")
        print(f"Capture Time: {total_time:.2f}s")
        print(f"Decompression Time: {end_decomp - start_decomp:.2f}s")
        print(f"**Actual Average FPS: {fps:.2f}**")

        if frame_list:
            print(f"First frame shape: {frame_list[0].shape} (dtype: {frame_list[0].dtype})")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
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
        print("\nTest complete")

if __name__ == "__main__":
    main()
