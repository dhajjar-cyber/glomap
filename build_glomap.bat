@echo off
REM Build script for GLOMAP on Windows
REM This script configures and builds GLOMAP with COLMAP and PoseLib

echo ========================================
echo Building GLOMAP with COLMAP and PoseLib
echo ========================================

REM Activate conda environment
call conda activate D:\conda_envs\VIS
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to activate conda environment VIS
    exit /b 1
)

REM Activate Visual Studio Developer Shell
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\Launch-VsDevShell.ps1" -Arch amd64
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to activate Visual Studio Developer Shell
    exit /b 1
)

REM Create build directory if it doesn't exist
if not exist "build" mkdir build
cd build

echo.
echo Configuring CMake...
cmake .. -GNinja ^
    -DCUDA_ENABLED=OFF ^
    -DFETCH_COLMAP=ON ^
    -DFETCH_POSELIB=ON ^
    -DCMAKE_TOOLCHAIN_FILE=C:\workspace\3DCD\vcpkg\scripts\buildsystems\vcpkg.cmake ^
    -DCGAL_DIR="C:\workspace\3DCD\vcpkg\installed\x64-windows\share\cgal" ^
    -DCMAKE_CXX_FLAGS="/bigobj"

if %ERRORLEVEL% NEQ 0 (
    echo ERROR: CMake configuration failed
    exit /b 1
)

echo.
echo Building with Ninja...
ninja

if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Build failed
    exit /b 1
)

echo.
echo ========================================
echo Build completed successfully!
echo ========================================
echo.
echo Executables can be found in:
echo   %CD%\glomap\
echo.

cd ..
