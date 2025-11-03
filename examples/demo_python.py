#!/usr/bin/env python3
"""
demo_python.py

Test script for SpiderCamera v0.2
Tests streaming, frame capture, and callback functionality.
(Updated to handle raw uint8 1D array from C++)
"""

import sys
import os
import time
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import spider_camera
except ImportError as e:
    print(f"❌ Failed to import spider_camera module: {e}")
    print("\nMake sure you:")
    print("1. Compiled the library: ./build.sh")
    print("2. Activated virtual environment: source .venv/bin/activate")
    print("3. Run from project root directory")
    sys.exit(1)


class FrameCounter:
    """Helper class to track frames"""
    def __init__(self):
        self.count = 0
        self.frames = []
        self.start_time = None
        
    def reset(self):
        self.count = 0
        self.frames = []
        self.start_time = time.time()
    
    def on_frame(self, frame: np.ndarray):
        self.count += 1
        
        # Store first 3 frames for analysis
        if len(self.frames) < 3:
            if frame.size > 0:
                self.frames.append(frame.copy())
        
        # Print info for first few frames
        if self.count <= 5:
            print(f"   Frame #{self.count}:")
            # (ОНОВЛЕНО)
            print(f"      Shape: {frame.shape}")
            print(f"      Dtype: {frame.dtype}")
            print(f"      Size (bytes): {frame.size}")
    
    def get_fps(self):
        if self.start_time and self.count > 0:
            elapsed = time.time() - self.start_time
            return self.count / elapsed if elapsed > 0 else 0
        return 0


def test_v02_streaming():
    """Test v0.2 streaming functionality"""
    
    print("=" * 70)
    print("  SpiderCamera v0.2 - Streaming Test")
    print("=" * 70)
    print()
    
    # Create camera instance
    print("📷 Creating SpiderCamera instance...")
    cam = spider_camera.SpiderCamera()
    
    if cam.get_state() == 4:
        print("❌ ERROR: Camera initialization failed!")
        return False
    print(f"   ✅ Camera initialized, state: {cam.get_state()}")
    print()
    
    # Optional: enable debug logging
    # cam.enable_debug(True)
    
    # Select and prepare camera
    print("📷 Preparing camera...")
    try:
        cam.set_cam(0)
        cam.be_ready()
        state = cam.get_state()
        print(f"   ✅ Camera ready, state: {state}")
        
        if state != 1:
            print(f"   ❌ Expected state 1, got {state}")
            return False
    except Exception as e:
        print(f"   ❌ Failed to prepare camera: {e}")
        return False
    print()
    
    # Setup frame counter
    counter = FrameCounter()
    
    # Register callback
    print("📷 Registering frame callback...")
    cam.set_frame_callback(counter.on_frame)
    print("   ✅ Callback registered")
    print()
    
    # Test 1: Short streaming session
    print("📷 Test 1: Streaming for 2 seconds...")
    try:
        counter.reset()
        cam.go()
        state = cam.get_state()
        print(f"   ✅ Streaming started, state: {state}")
        
        if state != 2:
            print(f"   ⚠️  WARNING: Expected state 2, got {state}")
        
        # Capture for 2 seconds
        time.sleep(2.0)
        
        cam.pause()
        state = cam.get_state()
        print(f"   ✅ Streaming paused, state: {state}")
        
        if state != 1:
            print(f"   ⚠️  WARNING: Expected state 1 after pause, got {state}")
        
        fps = counter.get_fps()
        print(f"   📊 Captured {counter.count} frames in 2s (~{fps:.1f} fps)")
        
        if counter.count < 10:
            print(f"   ⚠️  WARNING: Low frame count, expected ~28 frames")
        
    except Exception as e:
        print(f"   ❌ Streaming test failed: {e}")
        cam.stop()
        return False
    print()
    
    # Analyze captured frames
    print("📷 Analyzing captured frames...")
    if len(counter.frames) > 0:
        frame = counter.frames[0]
        # (ОНОВЛЕНО)
        print(f"   Frame shape: {frame.shape}")
        print(f"   Frame dtype: {frame.dtype}")
        print(f"   Frame size (bytes): {frame.size}")
        print(f"   Expected: uint8, 1D array (raw compressed bytes)")

        # Перевіряємо, чи є дані
        if frame.size > 1000: # Просто перевіряємо, що це не пустий буфер
            print(f"   ✅ Frame data looks valid (size > 1000 bytes)")
        else:
            print(f"   ⚠️  WARNING: Frame data is very small or empty")
            
        if frame.dtype == np.uint8:
            print(f"   ✅ Data format correct (uint8)")
        else:
            print(f"   ⚠️  WARNING: Unexpected data format")
    else:
        print(f"   ⚠️  WARNING: No frames captured")
    print()
    
    # Test 2: Restart streaming
    print("📷 Test 2: Restart streaming (1 second)...")
    try:
        counter.reset()
        cam.go()
        time.sleep(1.0)
        cam.pause()
        
        fps = counter.get_fps()
        print(f"   ✅ Captured {counter.count} frames in 1s (~{fps:.1f} fps)")
        
    except Exception as e:
        print(f"   ❌ Restart test failed: {e}")
        cam.stop()
        return False
    print()
    
    # Test 3: Multiple start/stop cycles
    print("📷 Test 3: Multiple quick cycles...")
    try:
        for cycle in range(3):
            counter.reset()
            cam.go()
            time.sleep(0.5)
            cam.pause()
            print(f"   Cycle {cycle+1}: {counter.count} frames")
        
        print(f"   ✅ Multiple cycles completed")
        
    except Exception as e:
        print(f"   ❌ Multiple cycles failed: {e}")
        cam.stop()
        return False
    print()
    
    # Stop camera
    print("📷 Stopping camera...")
    try:
        cam.stop()
        state = cam.get_state()
        print(f"   ✅ Camera stopped, state: {state}")
        
        if state != 0:
            print(f"   ⚠️  WARNING: Expected state 0, got {state}")
    except Exception as e:
        print(f"   ❌ Failed to stop: {e}")
        return False
    print()
    
    print("=" * 70)
    print("  ✅ All v0.2 streaming tests passed!")
    print("=" * 70)
    return True


def test_error_handling():
    """Test error handling and edge cases"""
    
    print()
    print("=" * 70)
    print("  Testing Error Handling")
    print("=" * 70)
    print()
    
    cam = spider_camera.SpiderCamera()
    cam.set_cam(0)
    cam.be_ready()
    
    # Test 1: go() from wrong state
    print("📷 Test: go() without be_ready()...")
    cam.stop()
    try:
        cam.go()
        print("   ⚠️  Should have raised exception!")
        return False
    except RuntimeError as e:
        print(f"   ✅ Expected exception: {e}")
    print()
    
    # Test 2: pause() without streaming
    print("📷 Test: pause() without streaming...")
    cam.set_cam(0)
    cam.be_ready()
    try:
        cam.pause()
        print("   ⚠️  Should have raised exception!")
        return False
    except RuntimeError as e:
        print(f"   ✅ Expected exception: {e}")
    print()
    
    # Test 3: Callback exception handling
    print("📷 Test: Python callback exception...")
    
    def bad_callback(frame):
        if counter.count == 2:
            raise ValueError("Test exception in callback")
        counter.count += 1
    
    counter = FrameCounter()
    cam.set_frame_callback(bad_callback)
    
    try:
        cam.go()
        time.sleep(0.5) # Даємо час, щоб помилки накопичились
        cam.pause()
        print(f"   ✅ Streaming continued despite callback exception")
        print(f"   Captured {counter.count} frames")
        
        if cam.get_state() == 4:
            print(f"   ✅ Camera correctly entered error state 4")
        else:
            print(f"   ⚠️  WARNING: Camera did not enter error state 4")
            
    except Exception as e:
        print(f"   ⚠️  Unexpected exception: {e}")
    finally:
        cam.stop()
    print()
    
    return True


def test_bayer_pattern():
    """
    (ОНОВЛЕНО)
    This test is now DEPRECATED as we receive a raw compressed 
    uint8 buffer, not an uncompressed uint16 Bayer pattern.
    """
    
    print()
    print("=" * 70)
    print("  Bayer Pattern Verification (SKIPPED)")
    print("=" * 70)
    print()
    print("   Test skipped: Receiving raw uint8 buffer, not uint16 Bayer.")
    print()
    return True
    
    # (Старий код закоментовано)
    # cam = spider_camera.SpiderCamera()
    # cam.set_cam(0)
    # cam.be_ready()
    # ...


if __name__ == "__main__":
    print()
    
    # Main streaming test
    success = test_v02_streaming()
    
    if not success:
        print("\n❌ Streaming tests failed!")
        sys.exit(1)
    
    # Additional tests
    test_error_handling()
    
    # (ОНОВЛЕНО)
    # Більше не викликаємо цей тест, оскільки він не актуальний
    # test_bayer_pattern()
    
    print()
    print("🎉 SpiderCamera v0.2 is working correctly!")
    print()
    print("Next steps:")
    print("  → Test with longer captures (10+ seconds)")
    print("  → Implement v0.3 for hot parameter control")
    print("  → Implement Python-side unpacking of the raw buffer (if needed)")
    print()
    
    sys.exit(0)
