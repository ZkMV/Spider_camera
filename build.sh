#!/bin/bash
# build.sh - Compilation script for SpiderCamera library (v0.3.0)

set -e  # Exit on error

echo "===================================="
echo "  SpiderCamera Build Script (v0.3.0)"
echo "===================================="

# Configuration
PYTHON_VERSION="3.11"
VENV_PATH=".venv"
PYBIND_INCLUDE="$VENV_PATH/lib/python$PYTHON_VERSION/site-packages/pybind11/include"
PYTHON_INCLUDE="/usr/include/python$PYTHON_VERSION"

# Check if pybind11 is installed
if [ ! -d "$PYBIND_INCLUDE" ]; then
    echo "ERROR: pybind11 not found in virtual environment!"
    echo "Run: source .venv/bin/activate && pip install pybind11"
    exit 1
fi

# Compiler flags
CXX="g++"
CXXFLAGS="-std=c++17 -fPIC -O2 -Wall -Wextra -Wno-unused-parameter"
LIBCAMERA_CFLAGS="$(pkg-config --cflags libcamera)"
INCLUDES="-I./include -I$PYBIND_INCLUDE -I$PYTHON_INCLUDE $LIBCAMERA_CFLAGS"
LIBCAMERA_LIBS="$(pkg-config --libs libcamera)"
LIBS="$LIBCAMERA_LIBS -lpthread"
LDFLAGS="-shared"

# Output file
OUTPUT="spider_camera$(python3-config --extension-suffix)"

echo ""
echo "Compiler: $CXX"
echo "Output: $OUTPUT"
echo ""

# Compile
echo "Compiling SpiderCamera..."
$CXX $CXXFLAGS $INCLUDES \
    src/spider_camera.cpp \
    bindings/pybind_spider.cpp \
    $LDFLAGS $LIBS \
    -o $OUTPUT

# v0.3.0: Видалено src/pisp_decompress.cpp та src/frame_buffer.cpp

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Build successful!"
    echo "   Output: $OUTPUT"
    echo ""
    echo "To test, run:"
    echo "   source .venv/bin/activate"
    echo "   python3 examples/demo_python.py"
else
    echo ""
    echo "❌ Build failed!"
    exit 1
fi
