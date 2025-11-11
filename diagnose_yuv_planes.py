#!/usr/bin/env python3
"""
diagnose_yuv_planes.py - Детальний аналіз Y/U/V площин

Запустіть:
  python3 diagnose_yuv_planes.py
"""

import sys
import os
import numpy as np

if 'spider_camera' in sys.modules:
    del sys.modules['spider_camera']

project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

try:
    import spider_camera
except ImportError as e:
    print(f"❌ Cannot import: {e}")
    sys.exit(1)

print("=" * 70)
print("YUV PLANE-BY-PLANE ANALYSIS")
print("=" * 70)

cam = None
try:
    cam = spider_camera.SpiderCamera()
    cam.enable_debug(False)
    cam.set_cam(0)
    cam.be_ready()
    
    width, height, pixel_format = cam.get_frame_properties()
    print(f"\nFormat: {pixel_format}, Size: {width}x{height}")
    
    # Capture
    cam.go()
    import time
    time.sleep(0.2)
    cam.pause()
    
    frames = cam.get_burst_frames()
    if not frames:
        print("❌ No frames!")
        sys.exit(1)
    
    frame = frames[0]
    print(f"Frame size: {frame.size:,} bytes")
    
    # I420 format: Y + U + V
    y_size = width * height
    uv_size = (width // 2) * (height // 2)
    
    # Розділити площини
    y_plane = frame[:y_size]
    u_plane = frame[y_size:y_size + uv_size]
    v_plane = frame[y_size + uv_size:y_size + uv_size * 2]
    
    print("\n" + "=" * 70)
    print("Y PLANE (Luminance):")
    print("=" * 70)
    print(f"  Size: {y_plane.size:,} bytes")
    print(f"  Min: {y_plane.min()}, Max: {y_plane.max()}, Mean: {y_plane.mean():.2f}")
    print(f"  Std: {y_plane.std():.2f}")
    print(f"  First 64 bytes: {y_plane[:64]}")
    
    # Гістограма Y
    hist_y, bins = np.histogram(y_plane, bins=16, range=(0, 256))
    print(f"\n  Y Histogram:")
    max_y = hist_y.max()
    for i, count in enumerate(hist_y):
        bar = '█' * int((count / max_y) * 40) if max_y > 0 else ''
        pct = (count / y_plane.size) * 100
        print(f"    {int(bins[i]):3d}-{int(bins[i+1]):3d}: {bar} {pct:5.1f}%")
    
    print("\n" + "=" * 70)
    print("U PLANE (Chroma Blue):")
    print("=" * 70)
    print(f"  Size: {u_plane.size:,} bytes")
    print(f"  Min: {u_plane.min()}, Max: {u_plane.max()}, Mean: {u_plane.mean():.2f}")
    print(f"  Std: {u_plane.std():.2f}")
    print(f"  First 64 bytes: {u_plane[:64]}")
    
    print("\n" + "=" * 70)
    print("V PLANE (Chroma Red):")
    print("=" * 70)
    print(f"  Size: {v_plane.size:,} bytes")
    print(f"  Min: {v_plane.min()}, Max: {v_plane.max()}, Mean: {v_plane.mean():.2f}")
    print(f"  Std: {v_plane.std():.2f}")
    print(f"  First 64 bytes: {v_plane[:64]}")
    
    print("\n" + "=" * 70)
    print("ANALYSIS:")
    print("=" * 70)
    
    # Нормальні значення YUV
    print("\n✓ Expected values for normal image:")
    print("  Y: ~100-150 (luminance)")
    print("  U: ~110-140 (chroma, centered at 128)")
    print("  V: ~110-140 (chroma, centered at 128)")
    
    print("\n→ Actual values:")
    print(f"  Y: {y_plane.mean():.1f} ", end='')
    if 80 < y_plane.mean() < 180:
        print("✓ OK")
    else:
        print("❌ TOO LOW!")
    
    print(f"  U: {u_plane.mean():.1f} ", end='')
    if 100 < u_plane.mean() < 160:
        print("✓ OK")
    else:
        print("⚠️  SUSPICIOUS")
    
    print(f"  V: {v_plane.mean():.1f} ", end='')
    if 100 < v_plane.mean() < 160:
        print("✓ OK")
    else:
        print("⚠️  SUSPICIOUS")
    
    # Перевірка чи площини різні
    if np.array_equal(u_plane, v_plane):
        print("\n❌ WARNING: U and V planes are IDENTICAL!")
        print("   This suggests wrong plane splitting or format!")
    
    # Перевірка чи площини мають дані
    if u_plane.std() < 1.0 or v_plane.std() < 1.0:
        print("\n❌ WARNING: U or V plane has very low variance!")
        print("   This suggests empty/constant data!")
    
    # Спроба manual конвертації
    print("\n" + "=" * 70)
    print("MANUAL YUV→RGB TEST:")
    print("=" * 70)
    
    try:
        import cv2
        
        # Reshape Y, U, V
        y_2d = y_plane.reshape((height, width))
        u_2d = u_plane.reshape((height // 2, width // 2))
        v_2d = v_plane.reshape((height // 2, width // 2))
        
        print(f"  Y shape: {y_2d.shape}")
        print(f"  U shape: {u_2d.shape}")
        print(f"  V shape: {v_2d.shape}")
        
        # Manual I420 assembly
        yuv_height = int(height * 1.5)
        yuv = frame.reshape((yuv_height, width))
        
        rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_I420)
        
        print(f"\n  RGB shape: {rgb.shape}")
        print(f"  RGB channels:")
        print(f"    R: min={rgb[:,:,0].min()}, max={rgb[:,:,0].max()}, mean={rgb[:,:,0].mean():.1f}")
        print(f"    G: min={rgb[:,:,1].min()}, max={rgb[:,:,1].max()}, mean={rgb[:,:,1].mean():.1f}")
        print(f"    B: min={rgb[:,:,2].min()}, max={rgb[:,:,2].max()}, mean={rgb[:,:,2].mean():.1f}")
        
        # Зберегти
        cv2.imwrite("diagnostic_planes.jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        print(f"\n✓ Saved: diagnostic_planes.jpg")
        
        # Також зберегти окремі площини для візуалізації
        cv2.imwrite("diagnostic_y_plane.jpg", y_2d)
        print(f"✓ Saved: diagnostic_y_plane.jpg (Y plane as grayscale)")
        
    except Exception as e:
        print(f"❌ Conversion failed: {e}")
    
    print("\n" + "=" * 70)
    print("RECOMMENDATION:")
    print("=" * 70)
    
    if y_plane.mean() < 50:
        print("❌ Y plane is TOO DARK!")
        print("   → Increase Exposure or ISO in C++ code")
        print("   → Currently: Exp=100us, ISO=4000")
        print("   → Try: Exp=10000us (10ms), ISO=2000")
    elif u_plane.std() < 5 and v_plane.std() < 5:
        print("❌ U/V planes have no data!")
        print("   → Check C++ stride handling or plane copying")
    else:
        print("⚠️  Data looks reasonable but colors wrong")
        print("   → Check YUV format (I420 vs NV12)")
        print("   → Verify plane order in C++ code")

except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
finally:
    if cam and cam.get_state() > 0:
        try:
            cam.stop()
        except:
            pass

print("\n" + "=" * 70)
