#!/usr/bin/env python3
"""
demo_python.py - v0.6 (Stride Aware)

Цей скрипт демонструє використання SpiderCamera v0.6 з підтримкою
апаратного вирівнювання (padding/stride).

Основні кроки:
1. Ініціалізація камери.
2. Отримання stride (кроку рядка) через get_frame_properties().
3. Захоплення серії кадрів ("сирі" дані з padding).
4. Правильний reshape та crop (обрізка сміття) перед збереженням.
"""

import sys
import os
import time
import cv2
import numpy as np
import json 

# Видаляємо старий модуль з пам'яті, якщо він там є
if 'spider_camera' in sys.modules:
    del sys.modules['spider_camera']

# Додаємо корінь проєкту в path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.append(project_root)

try:
    import spider_camera
except ImportError as e:
    print(f"❌ Error importing spider_camera: {e}")
    print(f"   Make sure 'spider_camera.so' is in: {project_root}")
    sys.exit(1)

# ============================================
# НАЛАШТУВАННЯ
# ============================================

CAPTURE_DURATION_SEC = 1.5 
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_config.json")
SAVE_PATH = os.path.join(project_root, "temp")

# ============================================

def main():
    print(f"=== SpiderCamera Burst Test (v0.6 - Stride Fixed) ===\n")
    
    # 1. Load Config
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
    except Exception as e:
        print(f"❌ Error loading config: {e}")
        sys.exit(1)
    
    os.makedirs(SAVE_PATH, exist_ok=True)

    cam = None
    try:
        cam = spider_camera.SpiderCamera()
        cam.enable_debug(True) # Бачимо логи C++
        cam.set_cam(0)

        # 2. Apply Settings
        print(f"Configuring: ISO {config['iso']}, Exp {config['exposure_us']}us, {config['resolution']}")
        cam.set_iso(config['iso'])
        cam.set_exposure(config['exposure_us'])
        cam.set_focus(config['focus_value'])
        
        try:
            w, h = map(int, config['resolution'].split('x'))
            cam.set_resolution(w, h)
        except:
            pass

        # GPIO Setup
        try:
            cam.set_frame_trigger_pin(21)
            cam.enable_frame_trigger(True)
        except Exception as e:
            print(f"⚠️ GPIO Warning: {e}")

        print("\nInitializing (be_ready)...")
        cam.be_ready() 
        
        # 🎯 v0.6: ОТРИМАННЯ STRIDE (КРОК РЯДКА)
        # Тепер метод повертає 4 значення!
        width, height, pixel_format_str, stride = cam.get_frame_properties()
        
        print(f"\n✅ CAMERA PROPERTIES (Hardware):")
        print(f"   Width:  {width}")
        print(f"   Height: {height}")
        print(f"   Stride: {stride} bytes (Row Width + Padding)")
        print(f"   Format: {pixel_format_str}")
        
        # Визначаємо висоту YUV буфера (Y + UV)
        # Для YUV420/NV12 загальна висота = height * 1.5
        yuv_height = int(height * 1.5)
        
        # Вибір конвертера OpenCV
        if pixel_format_str == "YUV420":
            color_cvt_code = cv2.COLOR_YUV2BGR_I420
            print("   Decoder: I420 (Planar)")
        elif pixel_format_str == "NV12":
            color_cvt_code = cv2.COLOR_YUV2BGR_NV12
            print("   Decoder: NV12 (Semi-Planar)")
        else:
            print(f"❌ Unknown format: {pixel_format_str}")
            return

        print(f"\nCapturing for {CAPTURE_DURATION_SEC}s...")
        cam.go()
        time.sleep(CAPTURE_DURATION_SEC)
        cam.pause()
        
        print("Fetching frames...")
        frame_list = cam.get_burst_frames()
        print(f"Captured {len(frame_list)} frames.")
        
        if not frame_list:
            print("❌ No frames captured!")
            return

        # 🎯 v0.6: ОБРОБКА ДАНИХ З УРАХУВАННЯМ STRIDE
        print(f"\nSaving frames to {SAVE_PATH}...")
        
        for i, flat_frame in enumerate(frame_list):
            try:
                # 1. Перевіряємо, чи ділиться буфер на stride без залишку
                # Інколи буфер може мати ще додатковий паддінг в самому кінці
                rows_in_buffer = flat_frame.size // stride
                
                # 2. Інтерпретуємо як 2D масив (Rows x Stride)
                # Обрізаємо хвіст, якщо він не повний рядок
                view_2d = flat_frame[:rows_in_buffer*stride].reshape((rows_in_buffer, stride))
                
                # 3. CROP: Відрізаємо "сміття" (padding) справа
                # Беремо тільки перші 'width' колонок
                # Також беремо тільки 'yuv_height' рядків (на випадок зайвих рядків знизу)
                image_data_cropped = view_2d[:yuv_height, :width]
                
                # 4. Конвертація YUV -> BGR (вже на "чистих" даних)
                # Важливо: OpenCV очікує contiguous array, reshape може створити view.
                # np.ascontiguousarray гарантує правильну пам'ять для C++ OpenCV
                bgr_image = cv2.cvtColor(np.ascontiguousarray(image_data_cropped), color_cvt_code)
                
                # Збереження
                filename = f"frame_{i + 1:03d}.jpg"
                cv2.imwrite(os.path.join(SAVE_PATH, filename), bgr_image)
                
                if i == 0:
                    print(f"✓ Frame 1 saved. Raw size: {flat_frame.size} bytes.")
                    print(f"  Reshaped to ({rows_in_buffer}, {stride}) -> Cropped to ({yuv_height}, {width})")

            except Exception as e:
                print(f"❌ Error processing frame {i}: {e}")
                import traceback
                traceback.print_exc()
                break # Stop on error to avoid spam

    except Exception as e:
        print(f"Global Error: {e}")
    finally:
        if cam: cam.stop()

if __name__ == "__main__":
    main()
