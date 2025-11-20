#!/usr/bin/env python3
# examples/demo_python_preview.py
"""
demo_python_preview.py – headless preview для SpiderCamera.

Призначення:
- Читає налаштування з demo_config.json.
- Ініціалізує SpiderCamera (be_ready один раз).
- Запускає стрім (go) один раз.
- У циклі:
    * періодично забирає кадри через get_burst_frames() без зупинки стріму;
    * бере останній кадр у серії;
    * конвертує YUV → BGR;
    * зберігає кадр у JPEG-файл:
        <root>/temp/spider_preview.jpg
      де <root> – корінь репозиторію Spider_camera.

Через те, що у поточному venv використовується headless OpenCV,
cv2.namedWindow / cv2.imshow недоступні. Тому прев’ю реалізоване як
«живий» JPEG-файл, що постійно оновлюється.
"""

import sys
import os
import time
import json

import cv2
import numpy as np

# -----------------------------
# Налаштування preview
# -----------------------------

# Періодичність опитування кадрів у секундах (орієнтовний інтервал між прев’ю)
PREVIEW_POLL_SEC = 0.5  # ~2 кадри на секунду в прев’ю (кожні 0.5 s забираємо буфер)

# Шлях до конфігу (лежить поруч зі скриптом)
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_config.json")


# -----------------------------
# Допоміжні функції
# -----------------------------


def load_config():
    """Читає demo_config.json і повертає dict з налаштуваннями."""
    print("=== SpiderCamera Preview (headless) ===\n")
    print(f"Config file: {CONFIG_FILE}")

    try:
        with open(CONFIG_FILE, "r") as f:
            config = json.load(f)
    except Exception as e:
        print(f"❌ Error loading {CONFIG_FILE}: {e}")
        print("   Перевір, що demo_config.json лежить поруч зі скриптом (у папці examples).")
        sys.exit(1)

    print("✓ Loaded config:")
    print(f"  ISO:        {config.get('iso')}")
    print(f"  Exposure:   {config.get('exposure_us')} us")
    print(f"  Focus:      {config.get('focus_value')}")
    print(f"  Resolution: {config.get('resolution')}\n")
    return config


def apply_camera_settings(cam, config):
    """
    Застосовує ISO, витримку, фокус і роздільну здатність до обʼєкта SpiderCamera.

    Ці значення реально використовуються всередині go(), коли
    камера заповнює controls перед запуском стріму.
    """
    # ISO
    iso = int(config.get("iso", 100))
    cam.set_iso(iso)

    # Витримка (uс)
    exposure_us = int(config.get("exposure_us", 1000))
    cam.set_exposure(exposure_us)

    # Фокус (float, як у твоїй бібліотеці)
    focus_value = float(config.get("focus_value", 1.0))
    cam.set_focus(focus_value)

    # Роздільна здатність "WxH"
    res_str = config.get("resolution", "")
    try:
        w_str, h_str = res_str.split("x")
        w = int(w_str)
        h = int(h_str)
        cam.set_resolution(w, h)
    except Exception as e:
        print(f"⚠️  Invalid resolution format '{res_str}'. Using C++ default.")
        print(f"    (Error: {e})")


def prepare_opencv_format(cam):
    """
    Викликає get_frame_properties() і визначає параметри для OpenCV-конверсії.
    Повертає:
        width, height, yuv_height, expected_size, color_cvt_code
    """
    width, height, pixel_format_str = cam.get_frame_properties()
    print(f"Got Frame Properties: {width}x{height}, Format: {pixel_format_str}")

    color_cvt_code = None
    if pixel_format_str == "YUV420":
        # У SpiderCamera flatten-буфер: Y plane (H×W) + U plane + V plane, загальна висота 1.5*H
        yuv_height = int(height * 1.5)
        color_cvt_code = cv2.COLOR_YUV2BGR_I420
        print("✓ Using I420 (Planar) decoder.")
    elif pixel_format_str == "NV12":
        yuv_height = int(height * 1.5)
        color_cvt_code = cv2.COLOR_YUV2BGR_NV12
        print("✓ Using NV12 (Semi-planar) decoder.")
    else:
        yuv_height = height
        print(f"⚠️ Unsupported pixel format for preview: {pixel_format_str}")
        print("   Preview frames may not decode correctly.")

    expected_size = width * yuv_height
    print(f"  YUV buffer size per frame: {expected_size} bytes\n")
    return width, height, yuv_height, expected_size, color_cvt_code


def save_preview_frame(bgr_image, path):
    """
    Зберігає BGR-кадр у JPEG-файл.

    Використовує cv2.imencode, щоб не залежати від GUI-функцій OpenCV.
    """
    try:
        success, encoded = cv2.imencode(".jpg", bgr_image)
        if not success:
            print("⚠️  Failed to encode frame to JPEG.")
            return
        # Записуємо байти у файл
        with open(path, "wb") as f:
            f.write(encoded.tobytes())
    except Exception as e:
        print(f"⚠️  Error saving preview frame to {path}: {e}")


# -----------------------------
# Основний preview-цикл
# -----------------------------


def show_preview():
    """
    Основний Workflow:
    - читаємо конфіг
    - додаємо PROJECT_ROOT до sys.path (для spider_camera)
    - створюємо SpiderCamera
    - be_ready()
    - визначаємо локальний шлях для превʼю-файла (<root>/temp/spider_preview.jpg)
    - запускаємо стрім один раз (go)
    - у циклі:
        * get_burst_frames() → беремо останній кадр → YUV → BGR → JPEG
        * sleep(PREVIEW_POLL_SEC)
    """
    config = load_config()
    cam = None

    # -------------------------
    # Визначаємо корінь проєкту
    # -------------------------
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if project_root not in sys.path:
        sys.path.append(project_root)

    # Локальна директорія для превʼю
    preview_dir = os.path.join(project_root, "temp")
    os.makedirs(preview_dir, exist_ok=True)
    preview_path = os.path.join(preview_dir, "spider_preview.jpg")

    try:
        import spider_camera
    except ImportError as e:
        print(f"❌ Error importing spider_camera: {e}")
        print(f"   Project root (added to path): {project_root}")
        sys.exit(1)

    try:
        cam = spider_camera.SpiderCamera()

        # Вимикаємо debug, щоб не засмічувати термінал
        cam.enable_debug(False)

        # Беремо першу камеру (як у demo_python.py)
        cam.set_cam(0)

        # Початкові налаштування з demo_config.json
        apply_camera_settings(cam, config)

        print("Starting camera (be_ready)...")
        cam.be_ready()
        print(f"State after be_ready: {cam.get_state()} (1 = Ready)\n")

        # Параметри кадру для конвертації
        width, height, yuv_height, expected_size, color_cvt_code = prepare_opencv_format(cam)
        if color_cvt_code is None:
            print("❌ Cannot determine YUV→BGR conversion code. Exiting.")
            return

        print("Starting streaming (go)...")
        cam.go()
        print(f"State after go: {cam.get_state()} (2 = Streaming)\n")

        print("Entering preview loop (headless, continuous streaming).")
        print(f"Preview frames will be written to: {preview_path}")
        print("Відкрий цей файл у переглядачі або браузері й оновлюй, щоб бачити останній кадр.")
        print("Натисни Ctrl+C у терміналі, щоб зупинити.\n")

        # Preview-цикл: стрім безперервний, ми лише періодично
        # забираємо накопичені кадри з буфера.
        while True:
            loop_start = time.time()

            # Забираємо всі кадри, що накопичились з останнього виклику
            frame_list = cam.get_burst_frames()

            if frame_list:
                flat_frame = frame_list[-1]

                if flat_frame.size != expected_size:
                    print(
                        f"⚠️  Frame size mismatch: got {flat_frame.size}, "
                        f"expected {expected_size}"
                    )
                else:
                    # Відновлюємо YUV-матрицю (H*1.5 x W)
                    yuv_image = flat_frame.reshape((yuv_height, width))

                    # YUV → BGR
                    try:
                        bgr_image = cv2.cvtColor(yuv_image, color_cvt_code)
                    except Exception as e:
                        print(f"⚠️  Error converting YUV to BGR: {e}")
                        bgr_image = None

                    if bgr_image is not None:
                        save_preview_frame(bgr_image, preview_path)
            # else: просто нічого не робимо, наступний цикл знову забере кадри

            # Обмежуємо частоту оновлення превʼю
            loop_time = time.time() - loop_start
            sleep_time = max(0.0, PREVIEW_POLL_SEC - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user (Ctrl+C).")
    except Exception as e:
        print(f"❌ Error in preview loop: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # Акуратне вимкнення камери
        try:
            if cam is not None and cam.get_state() > 0:
                # Ми запускали тільки go(), без pause() в циклі,
                # тому достатньо просто stop()
                cam.stop()
        except Exception as e:
            print(f"Error during shutdown: {e}")

        print("Preview finished.\n")


if __name__ == "__main__":
    show_preview()
