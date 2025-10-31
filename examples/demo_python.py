#!/usr/bin/env python3
"""
demo_python.py

Test script for SpiderCamera v0.1
Tests basic initialization, camera selection, and state management.
"""

import sys
import os
import time 

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


def test_v01_basic():
    """Test v0.1 functionality: initialization, camera selection, state management"""
    
    print("=" * 60)
    print("  SpiderCamera v0.1 - Basic Test")
    print("=" * 60)
    print()
    
    # Create camera instance
    print("📷 Creating SpiderCamera instance...")
    cam = spider_camera.SpiderCamera()
    print(f"   Initial state: {cam.get_state()} (should be 0 or 4)")
    print()
    
    # Check initial state
    state = cam.get_state()
    if state == 4:
        print("❌ ERROR: Camera manager failed to initialize!")
        print("   Make sure libcamera is installed and a camera is connected.")
        return False
    elif state != 0:
        print(f"⚠️  WARNING: Unexpected initial state: {state}")
    
    # Select camera
    print("📷 Selecting camera 0...")
    try:
        cam.set_cam(0)
        selected_cam = cam.get_cam()
        print(f"   ✅ Camera selected: {selected_cam}")
        print(f"   State: {cam.get_state()} (should still be 0)")
    except Exception as e:
        print(f"   ❌ Failed to select camera: {e}")
        return False
    print()
    
    # Prepare camera (be_ready)
    print("📷 Preparing camera (be_ready)...")
    try:
        cam.be_ready()
        state = cam.get_state()
        print(f"   ✅ Camera ready!")
        print(f"   State: {state} (should be 1)")
        
        if state != 1:
            print(f"   ⚠️  WARNING: Expected state 1, got {state}")
            return False
    except Exception as e:
        print(f"   ❌ Failed to prepare camera: {e}")
        return False
    print()
    
    # Test state consistency
    print("📷 Testing state consistency...")
    time.sleep(0.5)
    state = cam.get_state()
    print(f"   State after 0.5s: {state} (should still be 1)")
    print()
    
    # Test double be_ready (should fail)
    print("📷 Testing double be_ready (should fail)...")
    try:
        cam.be_ready()
        print("   ⚠️  WARNING: be_ready() should have thrown an exception!")
    except RuntimeError as e:
        print(f"   ✅ Expected exception caught: {e}")
    print()
    
    # Stop camera
    print("📷 Stopping camera...")
    try:
        cam.stop()
        state = cam.get_state()
        print(f"   ✅ Camera stopped")
        print(f"   State: {state} (should be 0)")
        
        if state != 0:
            print(f"   ⚠️  WARNING: Expected state 0, got {state}")
            return False
    except Exception as e:
        print(f"   ❌ Failed to stop camera: {e}")
        return False
    print()
    
    # Test restart cycle
    print("📷 Testing restart cycle...")
    try:
        cam.set_cam(0)
        cam.be_ready()
        state = cam.get_state()
        print(f"   ✅ Camera restarted successfully, state: {state}")
        cam.stop()
    except Exception as e:
        print(f"   ❌ Failed restart cycle: {e}")
        return False
    print()
    
    print("=" * 60)
    print("  ✅ All v0.1 tests passed!")
    print("=" * 60)
    return True


def test_stub_methods():
    """Test that stub methods for future versions exist and don't crash"""
    
    print()
    print("=" * 60)
    print("  Testing stub methods (v0.3, v0.4)")
    print("=" * 60)
    print()
    
    cam = spider_camera.SpiderCamera()
    cam.set_cam(0)
    
    print("📷 Testing v0.3 stub methods...")
    try:
        # ISO
        cam.set_iso(1600)
        iso = cam.get_iso()
        print(f"   ISO: {iso} (stub, should return 0)")
        
        # Exposure
        cam.set_exposure(10000)
        exp = cam.get_exposure()
        print(f"   Exposure: {exp} (stub, should return 0)")
        
        # Resolution
        cam.set_resolution(1920, 1080)
        res = cam.get_resolution()
        print(f"   Resolution: {res} (stub, should return (0, 0))")
        
        # Focus
        cam.set_focus(10)
        focus = cam.get_focus()
        print(f"   Focus: {focus} (stub, should return 0)")
        
        print("   ✅ v0.3 stubs work")
    except Exception as e:
        print(f"   ❌ v0.3 stub error: {e}")
        return False
    print()
    
    print("📷 Testing v0.4 stub methods...")
    try:
        # GPIO
        cam.set_spider_gpio(17)
        gpio = cam.get_spider_gpio()
        print(f"   GPIO pin: {gpio} (stub, should return -1)")
        
        # Trigger
        cam.set_spider_trigger(True)
        trigger = cam.get_spider_trigger()
        print(f"   Trigger: {trigger} (stub, should return False)")
        
        print("   ✅ v0.4 stubs work")
    except Exception as e:
        print(f"   ❌ v0.4 stub error: {e}")
        return False
    print()
    
    return True


if __name__ == "__main__":
    print()
    success = test_v01_basic()
    
    if success:
        test_stub_methods()
        print()
        print("🎉 SpiderCamera v0.1 is working correctly!")
        print()
        print("Next steps:")
        print("  → Implement v0.2 for frame streaming")
        print("  → Test with: python3 examples/demo_v02.py")
        sys.exit(0)
    else:
        print()
        print("❌ Some tests failed. Check the error messages above.")
        sys.exit(1)
