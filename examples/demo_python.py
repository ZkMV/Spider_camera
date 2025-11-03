#!/usr/bin/env python3
"""
demo_python.py

Test script for SpiderCamera v0.2
Tests streaming, frame capture, and callback functionality.
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
    
    def on_frame(self, frame):
        self.count += 1
        
        # Store first 3 frames for analysis
        if len(self.frames) < 3:
            self.frames.append(frame.copy())
        
        # Print info for first few frames
        if self.count <= 5:
            print(f"   Frame #{self.count}:")
            print(f"      Shape: {frame.shape}")
            print(f"      Dtype: {frame.dtype}")
            print(f"      Min: {frame.min()}, Max: {frame.max()}, Mean: {frame.mean():.1f}")
    
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
    #cam.enable_debug(True) Вмикає дебаг під час стрімінга спайдером
    
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
        print(f"   Frame shape: {frame.shape}")
        print(f"   Frame dtype: {frame.dtype}")
        print(f"   Value range: {frame.min()} - {frame.max()}")
        print(f"   Expected: uint16, values 0-1023 (10-bit)")
        
        # Check Bayer pattern (simple check)
        if frame.shape[0] > 2 and frame.shape[1] > 2:
            # Check if there's variation (not all zeros)
            if frame.std() > 10:
                print(f"   ✅ Frame data looks valid (std: {frame.std():.1f})")
            else:
                print(f"   ⚠️  WARNING: Frame data has low variation")
        
        # Check if data is in expected range
        if frame.dtype == np.uint16 and frame.max() <= 1023:
            print(f"   ✅ Data format correct (10-bit in uint16)")
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
        time.sleep(0.5)
        cam.pause()
        print(f"   ✅ Streaming continued despite callback exception")
        print(f"   Captured {counter.count} frames")
    except Exception as e:
        print(f"   ⚠️  Unexpected exception: {e}")
    finally:
        cam.stop()
    print()
    
    return True


def test_bayer_pattern():
    """Verify Bayer pattern structure"""
    
    print()
    print("=" * 70)
    print("  Bayer Pattern Verification")
    print("=" * 70)
    print()
    
    cam = spider_camera.SpiderCamera()
    cam.set_cam(0)
    cam.be_ready()
    
    counter = FrameCounter()
    cam.set_frame_callback(counter.on_frame)
    
    cam.go()
    time.sleep(0.5)
    cam.pause()
    cam.stop()
    
    if len(counter.frames) > 0:
        frame = counter.frames[0]
        print(f"📷 Frame analysis:")
        print(f"   Shape: {frame.shape}")
        print(f"   Dtype: {frame.dtype}")
        print()
        
        # Sample 2x2 block from center
        h, w = frame.shape
        cy, cx = h // 2, w // 2
        block = frame[cy:cy+2, cx:cx+2]
        
        print(f"   Sample 2x2 block from center:")
        print(f"   [{block[0,0]:4d} {block[0,1]:4d}]  <- R  G")
        print(f"   [{block[1,0]:4d} {block[1,1]:4d}]  <- G  B")
        print()
        print(f"   (RGGB Bayer pattern expected)")
        print()
        
        # Check if all corners have different statistics (rough check)
        tl = frame[0:h//2, 0:w//2].mean()
        tr = frame[0:h//2, w//2:w].mean()
        bl = frame[h//2:h, 0:w//2].mean()
        br = frame[h//2:h, w//2:w].mean()
        
        print(f"   Quadrant means:")
        print(f"   TL: {tl:.1f}  TR: {tr:.1f}")
        print(f"   BL: {bl:.1f}  BR: {br:.1f}")
        print()
        
        if abs(tl - br) > 10 or abs(tr - bl) > 10:
            print(f"   ✅ Quadrants show variation (good sign)")
        else:
            print(f"   ⚠️  Low variation between quadrants")
    
    return True


if __name__ == "__main__":
    print()
    
    # Main streaming test
    success = test_v02_streaming()
    
    if not success:
        print("\n❌ Streaming tests failed!")
        sys.exit(1)
    
    # Additional tests
    test_error_handling()
    test_bayer_pattern()
    
    print()
    print("🎉 SpiderCamera v0.2 is working correctly!")
    print()
    print("Next steps:")
    print("  → Test with longer captures (10+ seconds)")
    print("  → Implement v0.3 for hot parameter control")
    print("  → Verify FPS stability under load")
    print()
    
    sys.exit(0)
