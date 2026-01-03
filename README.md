# Android RGBD-SLAM

Real-time RGBD SLAM system for Android using Intel RealSense D435 camera.

## Overview

This project brings the capabilities of your ROS2 RGBD-to-PC-Realtime system to Android devices. It captures RGB-D data from an Intel RealSense D435 camera connected via USB OTG and performs real-time SLAM to generate 3D point cloud maps.

## Hardware Requirements

- **Android Device**: Phone or tablet with USB OTG/Host Mode support (Android 7.0+)
- **Intel RealSense D435** depth camera
- **USB OTG Cable/Adapter**: USB-C to USB-A or appropriate adapter
- **Powered USB Hub** (recommended): To provide external power to the camera
- **Minimum 4GB RAM** recommended

## Current Status

### ✅ Phase 1: Minimal Project (COMPLETED)
- [x] Android project structure created
- [x] Basic UI with SurfaceView for camera display
- [x] Permission handling (Camera, Storage, USB)
- [x] USB device filtering for RealSense
- [x] MainActivity template with lifecycle management

### 🔧 Next Steps
See [ROADMAP.md](ROADMAP.md) for the complete implementation plan.

## Project Structure

```
Android-RGBD-SLAM/
├── app/
│   ├── src/main/
│   │   ├── kotlin/com/rgbd/slam/
│   │   │   └── MainActivity.kt          # Main activity (Phase 1)
│   │   ├── res/
│   │   │   ├── layout/
│   │   │   │   └── activity_main.xml    # UI layout
│   │   │   ├── values/
│   │   │   │   └── strings.xml
│   │   │   └── xml/
│   │   │       └── device_filter.xml    # USB device filter
│   │   └── AndroidManifest.xml
│   ├── build.gradle
│   └── libs/                            # RealSense AAR will go here
├── build.gradle
├── settings.gradle
├── gradle.properties
├── README.md                            # This file
└── ROADMAP.md                           # Implementation roadmap
```

## Building the Project

### Prerequisites

1. **Android Studio** (Latest version recommended)
2. **Android SDK** (API Level 24+)
3. **Kotlin** support
4. **Intel RealSense SDK AAR** (we'll build this)

### Setup Steps

1. Clone this repository
2. Open Android Studio
3. Open this project: `File -> Open -> [path to Android-RGBD-SLAM]`
4. Build the RealSense SDK AAR (see below)
5. Copy AAR to `app/libs/`
6. Sync Gradle
7. Build and run on your Android device

### Building RealSense SDK for Android

```bash
cd /path/to/librealsense/wrappers/android
./gradlew assembleRelease

# Copy the generated AAR
cp librealsense/build/outputs/aar/librealsense-release.aar \
   /path/to/Android-RGBD-SLAM/app/libs/
```

## Usage

1. Connect Intel RealSense D435 to your Android device via USB OTG
2. Grant camera and storage permissions when prompted
3. Tap "Start Capture" to begin streaming
4. (Future phases will add SLAM, point cloud accumulation, and export)

## Comparison to ROS2 System

This Android implementation mirrors your ROS2 setup:

| Component | ROS2 (RGBD-to-PC-Realtime) | Android (This Project) |
|-----------|---------------------------|------------------------|
| Camera Interface | `realsense2_camera_node` | RealSense Android SDK |
| RGB-D Capture | ROS topics | Direct SDK calls |
| SLAM | RTABMap | ORB-SLAM3 (planned) |
| Odometry | `rgbd_odometry` | Visual-Inertial fusion |
| IMU | External (optional) | Phone sensors |
| Point Cloud | PCL + ROS | Custom accumulator |
| Visualization | RViz2 | OpenGL ES |
| Export | PCD/PLY service | File export |

## Development Roadmap

See [ROADMAP.md](ROADMAP.md) for detailed implementation phases.

**Summary:**
- **Phase 1**: Basic RealSense capture ✅ (Current)
- **Phase 2**: Point cloud generation and display
- **Phase 3**: Point cloud accumulation with voxel filtering
- **Phase 4**: Simple visual odometry
- **Phase 5**: ORB-SLAM3 integration
- **Phase 6**: IMU fusion
- **Phase 7**: Export and optimization

## License

See [LICENSE](LICENSE)

## Acknowledgments

- Intel RealSense SDK: https://github.com/IntelRealSense/librealsense
- ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
- Inspired by RGBD-to-PC-Realtime ROS2 system
