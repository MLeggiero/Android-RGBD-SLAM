# Android RGBD-SLAM Implementation Roadmap

This document outlines the phased approach to building a complete RGBD-SLAM system on Android, starting from the minimal project and building up to full SLAM with loop closure.

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────────┐
│                    Final Architecture                       │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  [RealSense D435] ←USB→ [Android Phone]                   │
│         │                        │                         │
│         │                   [IMU Sensors]                  │
│         ↓                        ↓                         │
│  [RealSense SDK]          [SensorManager]                 │
│         │                        │                         │
│         ├─→ RGB Stream           │                         │
│         ├─→ Depth Stream         │                         │
│         └─→ Point Cloud          │                         │
│                 ↓                ↓                         │
│         [Preprocessing & Filtering]                        │
│                      ↓                                     │
│              [ORB-SLAM3 Engine]                           │
│                      ↓                                     │
│         ┌────────────┴────────────┐                       │
│         ↓                         ↓                        │
│  [Visual Odometry]        [Loop Closure]                  │
│         ↓                         ↓                        │
│     [Pose Graph] ←───────────────┘                        │
│         ↓                                                  │
│  [Point Cloud Accumulator]                                │
│         │                                                  │
│         ├─→ Voxel Grid Filter                            │
│         ├─→ Noise Removal                                │
│         └─→ Transform to World Frame                     │
│                      ↓                                     │
│         ┌────────────┴────────────┐                       │
│         ↓                         ↓                        │
│  [OpenGL Renderer]        [PLY/PCD Exporter]              │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Minimal RealSense Capture ✅ COMPLETED

**Goal**: Create a working Android app that connects to RealSense D435 and displays the camera stream.

### What We Built
- ✅ Android project structure
- ✅ Gradle build configuration
- ✅ USB OTG device filtering
- ✅ Permission handling
- ✅ Basic UI layout
- ✅ MainActivity template

### What's Missing
- RealSense SDK AAR integration (need to build it)
- Actual camera initialization code
- Frame capture and display

### Next Action
Build the RealSense SDK and integrate it.

---

## Phase 2: RealSense Integration & Display

**Goal**: Display live RGB and Depth streams from RealSense D435.

### Components to Add

#### 2.1 RealSense SDK Setup
```kotlin
// Files to modify: MainActivity.kt

class MainActivity : AppCompatActivity() {
    private var rsContext: RsContext? = null
    private var pipeline: Pipeline? = null
    private var colorizer: Colorizer? = null

    private fun initializeRealSense() {
        RsContext.init(applicationContext)
        rsContext = RsContext()
        pipeline = Pipeline()
        colorizer = Colorizer()

        // Set up device listener
        rsContext?.setDevicesChangedCallback(deviceListener)
    }
}
```

#### 2.2 Stream Configuration
```kotlin
// Match your ROS2 camera_params.yaml settings

private fun configureStreams() {
    val config = Config()

    // RGB: 640x480 @ 30fps (starting with lower res for testing)
    config.enableStream(StreamType.COLOR, 640, 480, 30)

    // Depth: 640x480 @ 30fps
    config.enableStream(StreamType.DEPTH, 640, 480, 30)

    pipeline?.start(config)
}
```

#### 2.3 Frame Capture Loop
```kotlin
private val captureRunnable = object : Runnable {
    override fun run() {
        try {
            val frames = pipeline?.waitForFrames()
            frames?.let {
                // Get depth frame
                val depthFrame = it.first(StreamType.DEPTH)

                // Get color frame
                val colorFrame = it.first(StreamType.COLOR)

                // Display on surface view
                displayFrame(colorFrame, depthFrame)

                it.close()
            }

            if (isStreaming) {
                handler.post(this)
            }
        } catch (e: Exception) {
            Log.e(TAG, "Capture error: ${e.message}")
        }
    }
}
```

### Files to Create
- `app/libs/librealsense.aar` (built from Intel SDK)

### Expected Output
- Live depth visualization (colorized)
- Live RGB camera feed
- FPS counter showing ~30fps

---

## Phase 3: Point Cloud Generation

**Goal**: Generate and display 3D point clouds from depth + RGB data.

### Components to Add

#### 3.1 Point Cloud Generator
```kotlin
// New file: PointCloudGenerator.kt

class PointCloudGenerator(
    private val width: Int,
    private val height: Int,
    private val intrinsics: Intrinsics
) {
    fun generatePointCloud(
        depthData: ShortArray,
        colorData: ByteArray
    ): FloatArray {
        val points = mutableListOf<Float>()

        for (y in 0 until height step 2) {  // Subsample for performance
            for (x in 0 until width step 2) {
                val depthIdx = y * width + x
                val depthMm = depthData[depthIdx].toInt() and 0xFFFF
                val depthMeters = depthMm / 1000.0f

                if (depthMeters > 0.3f && depthMeters < 6.0f) {
                    // Unproject 2D pixel to 3D point
                    val point3D = unproject(x, y, depthMeters)

                    // Get RGB color
                    val colorIdx = (y * width + x) * 3
                    val r = colorData[colorIdx].toInt() and 0xFF
                    val g = colorData[colorIdx + 1].toInt() and 0xFF
                    val b = colorData[colorIdx + 2].toInt() and 0xFF

                    // Add to point cloud: X, Y, Z, R, G, B
                    points.addAll(listOf(
                        point3D[0], point3D[1], point3D[2],
                        r / 255f, g / 255f, b / 255f
                    ))
                }
            }
        }

        return points.toFloatArray()
    }

    private fun unproject(x: Int, y: Int, depth: Float): FloatArray {
        // Camera intrinsics unprojection
        val px = (x - intrinsics.ppx) / intrinsics.fx
        val py = (y - intrinsics.ppy) / intrinsics.fy

        return floatArrayOf(
            depth * px,
            depth * py,
            depth
        )
    }
}
```

#### 3.2 OpenGL Renderer
```kotlin
// New file: PointCloudRenderer.kt

class PointCloudRenderer : GLSurfaceView.Renderer {
    private var pointCloud: FloatArray = floatArrayOf()
    private lateinit var viewMatrix: FloatArray
    private lateinit var projectionMatrix: FloatArray

    fun updatePointCloud(points: FloatArray) {
        pointCloud = points
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

        // Render point cloud
        drawPointCloud(pointCloud)
    }

    // ... OpenGL setup code
}
```

### Expected Output
- Real-time 3D point cloud visualization
- Touch controls for rotation/zoom
- ~15-20 FPS with OpenGL rendering

---

## Phase 4: Point Cloud Accumulation

**Goal**: Accumulate point clouds over time with voxel filtering (like RTABMap does).

### Components to Add

#### 4.1 Point Cloud Accumulator
```kotlin
// New file: PointCloudAccumulator.kt

class PointCloudAccumulator(
    private val voxelSize: Float = 0.005f  // 5mm like RTABMap
) {
    private val voxelGrid = ConcurrentHashMap<VoxelKey, ColoredPoint>()
    private val transformMatrix = FloatArray(16)

    data class VoxelKey(val x: Int, val y: Int, val z: Int)
    data class ColoredPoint(
        val x: Float, val y: Float, val z: Float,
        val r: Float, val g: Float, val b: Float
    )

    fun addPointCloud(points: FloatArray, cameraPose: Pose) {
        Matrix.setIdentityM(transformMatrix, 0)
        // Set transform from cameraPose

        for (i in points.indices step 6) {
            val x = points[i]
            val y = points[i + 1]
            val z = points[i + 2]
            val r = points[i + 3]
            val g = points[i + 4]
            val b = points[i + 5]

            // Transform point to world coordinates
            val worldPoint = transformPoint(x, y, z, transformMatrix)

            // Add to voxel grid
            val voxelKey = getVoxelKey(worldPoint)
            voxelGrid[voxelKey] = ColoredPoint(
                worldPoint[0], worldPoint[1], worldPoint[2],
                r, g, b
            )
        }
    }

    fun getAccumulatedCloud(): FloatArray {
        // Return all points in voxel grid
        return voxelGrid.values.flatMap {
            listOf(it.x, it.y, it.z, it.r, it.g, it.b)
        }.toFloatArray()
    }

    private fun getVoxelKey(point: FloatArray): VoxelKey {
        return VoxelKey(
            (point[0] / voxelSize).toInt(),
            (point[1] / voxelSize).toInt(),
            (point[2] / voxelSize).toInt()
        )
    }
}
```

### Expected Output
- Growing point cloud map as camera moves
- Consistent 5mm voxel resolution
- Memory-efficient storage

---

## Phase 5: Simple Visual Odometry

**Goal**: Track camera movement using feature matching (pre-SLAM).

### Components to Add

#### 5.1 Feature Detector
```kotlin
// New file: VisualOdometry.kt
// Using OpenCV for Android

class SimpleVisualOdometry {
    private val orb = ORB.create(1000)  // 1000 features like RTABMap
    private var prevFrame: Mat? = null
    private var prevKeypoints: MatOfKeyPoint? = null
    private var prevDescriptors: Mat? = null

    data class Pose(
        val translation: FloatArray,
        val rotation: FloatArray
    )

    fun estimatePose(currentFrame: Mat, depthFrame: Mat): Pose? {
        // Detect ORB features
        val keypoints = MatOfKeyPoint()
        val descriptors = Mat()
        orb.detectAndCompute(currentFrame, Mat(), keypoints, descriptors)

        if (prevFrame != null) {
            // Match features
            val matches = matchFeatures(prevDescriptors!!, descriptors)

            // Estimate motion from matches
            val pose = estimateMotion(
                prevKeypoints!!, keypoints,
                matches, depthFrame
            )

            // Update previous frame
            prevFrame = currentFrame.clone()
            prevKeypoints = keypoints
            prevDescriptors = descriptors

            return pose
        } else {
            prevFrame = currentFrame.clone()
            prevKeypoints = keypoints
            prevDescriptors = descriptors
            return null
        }
    }
}
```

### Expected Output
- Camera pose estimation (x, y, z, roll, pitch, yaw)
- ~2 FPS processing (matching RTABMap detection_rate)
- Point clouds transformed to consistent world frame

---

## Phase 6: ORB-SLAM3 Integration

**Goal**: Replace simple odometry with full SLAM including loop closure.

### Components to Add

#### 6.1 ORB-SLAM3 NDK Build
```cmake
# New file: app/src/main/cpp/CMakeLists.txt

cmake_minimum_required(VERSION 3.18)
project(orbslam3_wrapper)

# Include ORB-SLAM3
add_subdirectory(ORB_SLAM3)

# Create JNI wrapper
add_library(orbslam3_jni SHARED
    orbslam3_wrapper.cpp
)

target_link_libraries(orbslam3_jni
    ORB_SLAM3
    opencv_java4
)
```

#### 6.2 JNI Wrapper
```cpp
// New file: app/src/main/cpp/orbslam3_wrapper.cpp

#include <jni.h>
#include "System.h"

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_rgbd_slam_ORBSLAM3_nativeCreate(
    JNIEnv* env,
    jobject thiz,
    jstring vocab_path,
    jstring settings_path
) {
    const char* vocabPath = env->GetStringUTFChars(vocab_path, nullptr);
    const char* settingsPath = env->GetStringUTFChars(settings_path, nullptr);

    ORB_SLAM3::System* slam = new ORB_SLAM3::System(
        vocabPath,
        settingsPath,
        ORB_SLAM3::System::RGBD,
        true  // Use viewer
    );

    env->ReleaseStringUTFChars(vocab_path, vocabPath);
    env->ReleaseStringUTFChars(settings_path, settingsPath);

    return reinterpret_cast<jlong>(slam);
}

JNIEXPORT jfloatArray JNICALL
Java_com_rgbd_slam_ORBSLAM3_nativeTrackRGBD(
    JNIEnv* env,
    jobject thiz,
    jlong slam_ptr,
    jbyteArray rgb_data,
    jshortArray depth_data,
    jdouble timestamp
) {
    auto* slam = reinterpret_cast<ORB_SLAM3::System*>(slam_ptr);

    // Convert Java arrays to cv::Mat
    // ...

    // Track frame
    cv::Mat pose = slam->TrackRGBD(rgbMat, depthMat, timestamp);

    // Convert pose to jfloatArray
    // ...

    return poseArray;
}

} // extern "C"
```

#### 6.3 Kotlin Wrapper
```kotlin
// New file: ORBSLAM3.kt

class ORBSLAM3(
    vocabPath: String,
    settingsPath: String
) {
    private val nativePtr: Long

    init {
        System.loadLibrary("orbslam3_jni")
        nativePtr = nativeCreate(vocabPath, settingsPath)
    }

    fun trackRGBD(
        rgbData: ByteArray,
        depthData: ShortArray,
        timestamp: Double
    ): FloatArray {
        return nativeTrackRGBD(nativePtr, rgbData, depthData, timestamp)
    }

    private external fun nativeCreate(
        vocabPath: String,
        settingsPath: String
    ): Long

    private external fun nativeTrackRGBD(
        slamPtr: Long,
        rgbData: ByteArray,
        depthData: ShortArray,
        timestamp: Double
    ): FloatArray
}
```

### Expected Output
- Full SLAM with loop closure
- Drift-corrected trajectory
- Optimized pose graph
- Map point cloud

---

## Phase 7: IMU Sensor Fusion

**Goal**: Fuse Android phone's IMU with visual odometry for better tracking.

### Components to Add

#### 7.1 IMU Manager
```kotlin
// New file: IMUManager.kt

class IMUManager(context: Context) : SensorEventListener {
    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

    private val imuBuffer = mutableListOf<IMUData>()

    data class IMUData(
        val timestamp: Long,
        val accel: FloatArray,
        val gyro: FloatArray
    )

    override fun onSensorChanged(event: SensorEvent?) {
        event?.let {
            when (it.sensor.type) {
                Sensor.TYPE_ACCELEROMETER -> {
                    // Store accelerometer data
                }
                Sensor.TYPE_GYROSCOPE -> {
                    // Store gyroscope data
                }
            }
        }
    }

    fun getIMUDataBetween(startTime: Long, endTime: Long): List<IMUData> {
        return imuBuffer.filter { it.timestamp in startTime..endTime }
    }
}
```

### Expected Output
- IMU + Visual fusion
- Better odometry in low-texture areas
- Reduced drift

---

## Phase 8: Export & Optimization

**Goal**: Export point clouds and optimize for battery/performance.

### Components to Add

#### 8.1 PLY Exporter
```kotlin
// New file: PointCloudExporter.kt

class PointCloudExporter {
    fun exportToPLY(points: FloatArray, filePath: String) {
        val numPoints = points.size / 6

        val header = """
            ply
            format ascii 1.0
            element vertex $numPoints
            property float x
            property float y
            property float z
            property uchar red
            property uchar green
            property uchar blue
            end_header
        """.trimIndent()

        File(filePath).bufferedWriter().use { writer ->
            writer.write(header)
            writer.newLine()

            for (i in points.indices step 6) {
                val x = points[i]
                val y = points[i + 1]
                val z = points[i + 2]
                val r = (points[i + 3] * 255).toInt()
                val g = (points[i + 4] * 255).toInt()
                val b = (points[i + 5] * 255).toInt()

                writer.write("$x $y $z $r $g $b")
                writer.newLine()
            }
        }
    }
}
```

### Expected Output
- Export to PLY/PCD
- Compatible with CloudCompare, MeshLab
- Share or upload to PC

---

## Implementation Timeline

| Phase | Complexity | Estimated Effort | Dependencies |
|-------|-----------|------------------|--------------|
| Phase 1 ✅ | Low | 2 hours | None |
| Phase 2 | Low | 4 hours | RealSense AAR |
| Phase 3 | Medium | 6 hours | Phase 2, OpenGL |
| Phase 4 | Medium | 4 hours | Phase 3 |
| Phase 5 | High | 8 hours | Phase 4, OpenCV |
| Phase 6 | Very High | 16 hours | Phase 5, ORB-SLAM3 |
| Phase 7 | Medium | 6 hours | Phase 6 |
| Phase 8 | Low | 4 hours | Phase 7 |

**Total**: ~50 hours from minimal to full SLAM system

---

## Next Immediate Steps

1. **Build RealSense SDK AAR**
   ```bash
   cd /path/to/librealsense/wrappers/android
   ./gradlew assembleRelease
   cp librealsense/build/outputs/aar/*.aar /path/to/Android-RGBD-SLAM/app/libs/
   ```

2. **Integrate RealSense in MainActivity**
   - Uncomment RealSense SDK code
   - Test camera connection
   - Display depth stream

3. **Test on Device**
   - Connect RealSense D435 via USB OTG
   - Verify depth/color streams work
   - Check FPS and performance

---

## Configuration Comparison

Matching your ROS2 `camera_params.yaml` and `rtabmap_params.yaml`:

| Parameter | ROS2 Value | Android Equivalent |
|-----------|-----------|-------------------|
| Resolution | 1280x720@30 | Start with 640x480, scale up |
| Voxel Size | 0.005 (5mm) | `voxelSize = 0.005f` |
| Max Depth | 6.0m | `maxDepth = 6.0f` |
| Min Depth | 0.3m | `minDepth = 0.3f` |
| Features | 1000 (GFTT/ORB) | `ORB.create(1000)` |
| Detection Rate | 2.0 Hz | Process every 500ms |
| Cloud Decimation | 1 (no decimation) | Step by 1 pixel |

This ensures Android produces same quality as ROS2!
