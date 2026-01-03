# Setup Guide

Quick start guide for getting the Android RGBD-SLAM project running.

## Prerequisites

### Software
- **Android Studio** (Hedgehog 2023.1.1 or later)
- **Android SDK** API Level 24+ (Android 7.0+)
- **Android NDK** (for Phase 6+ when adding ORB-SLAM3)
- **Git**

### Hardware
- **Android Device** with:
  - USB OTG/Host support
  - Android 7.0+ (API 24+)
  - 4GB+ RAM recommended
  - ARM64 processor recommended
- **Intel RealSense D435** camera
- **USB OTG cable/adapter** (USB-C to USB-A or appropriate)
- **Powered USB Hub** (highly recommended for sustained use)

### Checking USB OTG Support
Download and install "USB OTG Checker" from Google Play Store to verify your device supports USB host mode.

## Step 1: Clone the Repository

```bash
git clone https://github.com/MLeggiero/Android-RGBD-SLAM.git
cd Android-RGBD-SLAM
```

## Step 2: Build RealSense SDK for Android

The Intel RealSense SDK needs to be compiled as an Android AAR library.

### Option A: Build from Source (Recommended)

```bash
# Clone RealSense SDK if you haven't already
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense/wrappers/android

# Build the AAR
./gradlew assembleRelease

# The AAR will be generated at:
# librealsense/build/outputs/aar/librealsense-release.aar
```

### Option B: Download Pre-built AAR

Check Intel's releases page: https://github.com/IntelRealSense/librealsense/releases

## Step 3: Add RealSense AAR to Project

```bash
# Create libs directory if it doesn't exist
mkdir -p Android-RGBD-SLAM/app/libs

# Copy the AAR file
cp ~/librealsense/wrappers/android/librealsense/build/outputs/aar/librealsense-release.aar \
   Android-RGBD-SLAM/app/libs/

# Or if you downloaded it:
cp ~/Downloads/librealsense-release.aar Android-RGBD-SLAM/app/libs/
```

## Step 4: Uncomment RealSense Dependency

Edit `app/build.gradle` and uncomment the RealSense dependency:

```gradle
dependencies {
    // ... other dependencies ...

    // RealSense SDK - Uncomment this line:
    implementation files('libs/librealsense-release.aar')
}
```

## Step 5: Open Project in Android Studio

1. Launch Android Studio
2. Click "Open" (not "New Project")
3. Navigate to `Android-RGBD-SLAM` folder
4. Click "OK"
5. Wait for Gradle sync to complete

## Step 6: Build the Project

1. In Android Studio, click **Build → Make Project**
2. Wait for build to complete
3. Fix any errors (should build cleanly if AAR is in place)

## Step 7: Connect Hardware

### Without Powered Hub (Testing Only)
```
[RealSense D435] ──USB Cable──> [USB-C Adapter] ──> [Android Phone]
```

⚠️ **Warning**: This will drain phone battery quickly! Only for short tests.

### With Powered Hub (Recommended)
```
[Power Adapter]
      │
      ↓
[Powered USB-C Hub] ──USB-C──> [Android Phone]
      │
      ↓ USB-A
[RealSense D435]
```

## Step 8: Enable Developer Options on Android

1. Go to **Settings → About Phone**
2. Tap "Build Number" 7 times
3. Go back to **Settings → Developer Options**
4. Enable **USB Debugging**
5. Enable **Stay Awake** (keeps screen on while charging)

## Step 9: Deploy to Device

1. Connect Android phone to computer via USB
2. Accept USB debugging prompt on phone
3. In Android Studio, select your device from dropdown
4. Click **Run** (green play button) or press **Shift+F10**

## Step 10: Grant Permissions

When the app launches:
1. Grant **Camera** permission
2. Grant **Storage** permissions (Read and Write)
3. Connect RealSense camera to phone via USB OTG

## Step 11: Test RealSense Connection

1. When RealSense connects, you should see "Camera Connected" in the app
2. Tap "Start Capture"
3. You should see the depth stream displayed (colorized depth visualization)

## Troubleshooting

### Camera Not Detected

**Check USB connection:**
```bash
# On computer with phone connected via ADB
adb shell lsusb
# Look for Intel vendor ID: 8086
```

**Grant USB permissions:**
- When you connect RealSense, Android should show a dialog asking for USB device permission
- Check "Always allow for this device" and tap OK

### App Crashes on Start

**Check logs:**
```bash
adb logcat | grep -i realsense
```

**Common issues:**
- RealSense AAR not in `app/libs/`
- Missing permissions in AndroidManifest.xml
- USB OTG not supported on device

### Low FPS or Lag

**Initial settings use 640x480 @ 30fps**

If still laggy:
1. Reduce resolution to 480x360
2. Lower frame rate to 15fps
3. Disable RGB stream (depth only)

Edit in MainActivity.kt:
```kotlin
config.enableStream(StreamType.DEPTH, 480, 360, 15)
// config.enableStream(StreamType.COLOR, 480, 360, 15)  // Comment out for depth-only
```

### Battery Draining Fast

**This is normal!** RealSense D435 draws ~1.5W.

Solutions:
1. Use powered USB hub (highly recommended)
2. Keep phone plugged into power while using app
3. Reduce frame rate and resolution

## Next Steps

Once you have Phase 2 working (RealSense streaming):
- Read [ROADMAP.md](ROADMAP.md) for implementation phases
- Start with Phase 3: Point Cloud Generation
- Each phase builds on the previous one

## Useful Commands

```bash
# View Android logs
adb logcat | grep "RGBD-SLAM"

# Check if camera is connected
adb shell lsusb

# Install APK manually
adb install app/build/outputs/apk/debug/app-debug.apk

# Clear app data (reset permissions)
adb shell pm clear com.rgbd.slam

# Record screen while testing
adb shell screenrecord /sdcard/test.mp4
```

## Development Tips

### Testing Without RealSense

For development without hardware, you can:
1. Use dummy data (random depth values)
2. Load pre-recorded depth images from assets
3. Test UI and other components

### Debugging Native Code (Phase 6+)

When ORB-SLAM3 is integrated:
```bash
# Attach debugger to native code
lldb
```

### Performance Monitoring

Android Studio Profiler:
1. **View → Tool Windows → Profiler**
2. Monitor CPU, Memory, Battery usage
3. Identify bottlenecks

## Resources

- [RealSense SDK Documentation](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/android)
- [Android USB Host Documentation](https://developer.android.com/guide/topics/connectivity/usb/host)
- [OpenGL ES for Android](https://developer.android.com/guide/topics/graphics/opengl)

## Getting Help

If you encounter issues:
1. Check existing Issues on GitHub
2. Review Android Studio build logs
3. Enable verbose logging in RealSense SDK
4. Compare with RealSense Android examples
