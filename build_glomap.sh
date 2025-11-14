#!/bin/bash
# Build script for GLOMAP on Linux/WSL
# This script configures and builds GLOMAP with COLMAP and PoseLib

echo "========================================"
echo "Building GLOMAP with COLMAP and PoseLib"
echo "========================================"

# Create build directory if it doesn't exist
mkdir -p build
cd build

echo ""
echo "Configuring CMake..."
cmake .. -GNinja \
    -DCUDA_ENABLED=OFF \
    -DFETCH_COLMAP=ON \
    -DFETCH_POSELIB=ON \
    -DCMAKE_TOOLCHAIN_FILE="${PWD}/../../../../vcpkg/scripts/buildsystems/vcpkg.cmake" \
    -DCGAL_DIR="${PWD}/../../../../vcpkg/installed/x64-linux/share/cgal" \
    -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo "ERROR: CMake configuration failed"
    exit 1
fi

echo ""
echo "Building with Ninja..."
ninja

if [ $? -ne 0 ]; then
    echo "ERROR: Build failed"
    exit 1
fi

echo ""
echo "========================================"
echo "Build completed successfully!"
echo "========================================"
echo ""
echo "Executables can be found in:"
echo "  $(pwd)/glomap/"
echo ""

cd ..
