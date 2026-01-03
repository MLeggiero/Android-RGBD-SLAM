package com.rgbd.slam

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.SurfaceView
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat

/**
 * Minimal Android RGBD-SLAM Application
 *
 * Phase 1: Basic RealSense capture and display
 * This is the foundation - we'll build SLAM on top of this
 */
class MainActivity : AppCompatActivity() {

    // UI Components
    private lateinit var surfaceView: SurfaceView
    private lateinit var statusText: TextView
    private lateinit var startButton: Button

    // RealSense components (to be implemented with SDK)
    // private var rsContext: RsContext? = null
    // private var pipeline: Pipeline? = null

    private var isStreaming = false
    private val handler = Handler(Looper.getMainLooper())

    companion object {
        private const val PERMISSION_REQUEST_CODE = 100
        private val REQUIRED_PERMISSIONS = arrayOf(
            Manifest.permission.CAMERA,
            Manifest.permission.WRITE_EXTERNAL_STORAGE,
            Manifest.permission.READ_EXTERNAL_STORAGE
        )
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Initialize UI
        surfaceView = findViewById(R.id.surfaceView)
        statusText = findViewById(R.id.statusText)
        startButton = findViewById(R.id.startButton)

        // Check permissions
        if (!hasPermissions()) {
            requestPermissions()
        } else {
            initializeCamera()
        }

        // Button click listener
        startButton.setOnClickListener {
            if (isStreaming) {
                stopCapture()
            } else {
                startCapture()
            }
        }
    }

    private fun hasPermissions(): Boolean {
        return REQUIRED_PERMISSIONS.all {
            ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
        }
    }

    private fun requestPermissions() {
        ActivityCompat.requestPermissions(this, REQUIRED_PERMISSIONS, PERMISSION_REQUEST_CODE)
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == PERMISSION_REQUEST_CODE) {
            if (hasPermissions()) {
                initializeCamera()
            } else {
                updateStatus("Permissions required!")
            }
        }
    }

    private fun initializeCamera() {
        updateStatus("Initializing RealSense...")

        // TODO: Initialize RealSense SDK
        // RsContext.init(applicationContext)
        // rsContext = RsContext()
        // pipeline = Pipeline()

        updateStatus("Ready - Connect RealSense via USB")
    }

    private fun startCapture() {
        updateStatus("Starting capture...")

        // TODO: Configure and start RealSense pipeline
        // val config = Config()
        // config.enableStream(StreamType.DEPTH, 640, 480)
        // config.enableStream(StreamType.COLOR, 640, 480)
        // pipeline?.start(config)

        isStreaming = true
        startButton.text = getString(R.string.stop_capture)
        updateStatus("Streaming")

        // Start capture loop
        // handler.post(captureRunnable)
    }

    private fun stopCapture() {
        updateStatus("Stopping capture...")

        // TODO: Stop pipeline
        // handler.removeCallbacks(captureRunnable)
        // pipeline?.stop()

        isStreaming = false
        startButton.text = getString(R.string.start_capture)
        updateStatus("Stopped")
    }

    private fun updateStatus(status: String) {
        runOnUiThread {
            statusText.text = "Status: $status"
        }
    }

    override fun onPause() {
        super.onPause()
        if (isStreaming) {
            stopCapture()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        // TODO: Clean up RealSense resources
        // pipeline?.close()
        // rsContext?.close()
    }
}
