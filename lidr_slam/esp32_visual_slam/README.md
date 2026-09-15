# ESP32-CAM + VL53L1X Visual SLAM & 3D Mapping

A real-time Python Visual SLAM (Simultaneous Localization and Mapping) and sparse 3D reconstruction system designed for the **AI Thinker ESP32-CAM** combined with a single-ray **VL53L1X Time-of-Flight (ToF)** distance sensor streaming over UDP Wi-Fi.

---

## System Architecture

```
ESP32-CAM (JPEG Slices + VL53L1X Distance)
        │  UDP Wi-Fi (Port 5000, Header: <IHHH)
        ▼
   udp_receiver.py (Reassembly, Low-Latency Single-Slot Queue, Frame Dropping)
        │
        ▼
   camera.py (Undistortion cv2.undistort, Grayscale Conversion)
        │
        ▼
   visual_odometry.py (ORB Features, Hamming KNN, Lowe's Ratio Test, Essential Matrix RANSAC, recoverPose)
        │
        ├── sensor_fusion.py (VL53L1X ToF filtering, Metric Scale Estimation, Forward Obstacle Tracking)
        ▼
   mapping.py (Sparse 3D Triangulation cv2.triangulatePoints, Cheirality Check, Voxel Filtering)
        │
        ├── Open3D Window (Sparse 3D Point Cloud, Trajectory, Coordinate Frame, Wireframe Frustum)
        ├── OpenCV Window 1 (Live Feed, HUD Overlays: UDP FPS, VO FPS, ToF, Features, Poses)
        └── OpenCV Window 2 (Feature Matches Correspondences)
```

---

## UDP Packet Protocol Specification

Each incoming UDP packet contains a 10-byte binary packed header:
- `uint32_t frameID`: Monotonically increasing frame index (4 bytes)
- `uint16_t packetIndex`: Index of this slice within the frame [0 ... totalPackets-1] (2 bytes)
- `uint16_t totalPackets`: Total packets comprising this frame (2 bytes)
- `uint16_t distanceMM`: Real-time VL53L1X range reading in millimeters (2 bytes)

Python struct format: `"<IHHH"` (10 bytes).
The raw JPEG chunk immediately follows the 10-byte header.

---

## Installation & Setup (Linux)

### 1. Requirements
- Linux (Ubuntu / Debian / Zorin OS / Mint / Arch)
- Python 3.10+ (tested on Python 3.12)
- NVIDIA GPU (tested on RTX 4050 Laptop GPU)
- Open3D, OpenCV, NumPy

### 2. Activate Virtual Environment
A dedicated virtual environment has been configured in `../venv`:
```bash
source ../venv/bin/activate
pip install -r requirements.txt
```

---

## Running the Application

### Option A: Live ESP32-CAM Mode
Power on your ESP32-CAM running the UDP streaming firmware pointing to your laptop's IP address on port 5000.

Run:
```bash
python3 main.py
```
Or with virtual environment explicitly:
```bash
../venv/bin/python3 main.py
```

### Option B: Offline / Synthetic Testing Mode (No ESP32 Hardware Needed)
To test the pipeline locally, run the included mock streamer in one terminal:
```bash
# Terminal 1: Start Mock ESP32-CAM UDP Streamer
../venv/bin/python3 mock_esp32_streamer.py --fps 15
```

Then launch the SLAM application in another terminal:
```bash
# Terminal 2: Run Visual SLAM
../venv/bin/python3 main.py
```

---

## User Interface Windows

1. **Window 1: ESP32-CAM Live Feed**:
   - Displays undistorted video feed.
   - Real-time HUD showing:
     - `UDP FPS`: Streaming network delivery rate.
     - `VO FPS`: Visual Odometry tracking rate.
     - `VL53L1X`: Distance in mm and meters, validity badge.
     - `ORB Features` and `Inlier Matches` counts.
     - `Camera Position [m]`: Estimated global $X, Y, Z$ coordinates.
     - `Metric Scale`: Filtered translation scale applied.
     - Optical center crosshair showing ToF ray alignment.

2. **Window 2: Feature Matches**:
   - Side-by-side keypoint correspondences between previous and current frames.
   - Green lines signify verified RANSAC inlier feature matches.

3. **Window 3: 3D SLAM Map (Open3D)**:
   - **Sparse Point Cloud**: Reconstructed 3D landmarks with RGB color.
   - **Coordinate Axes**: $X$ = Red (Right), $Y$ = Green (Down), $Z$ = Blue (Forward).
   - **Camera Trajectory**: Red 3D path line tracking robot motion.
   - **Camera Frustum**: Wireframe pyramid showing current 3D position and orientation.

Press `q` or `ESC` in any window to trigger a clean shutdown.

---

## Configuration & Camera Calibration

All parameters are organized in `config/camera_config.py`:
- **Camera Calibration**:
  - `fx, fy`: Focal length in pixels.
  - `cx, cy`: Principal point in pixels.
  - `k1, k2, p1, p2, k3`: Lens radial and tangential distortion coefficients.
- **ORB Parameters**: Feature count (default 2000), scale factor, levels, Lowe's ratio test (0.75).
- **VL53L1X Fusion**: Distance limits (40mm - 4000mm), EMA filter smoothing factor, scale estimation flags.
- **Mapping Bounds**: Min depth (0.1m), max depth (25.0m), max reprojection error (2.5px), voxel grid size (0.02m).

---

## Running Unit Tests

Run the automated test suite:
```bash
../venv/bin/python3 -m unittest discover -s tests -p "test_*.py"
```
All 6 automated tests verify:
- Binary UDP header packing/unpacking
- Frame slice reassembly and stale frame dropping
- Camera pose composition and inversion
- 3D point triangulation precision against ground truth
- Monocular VO tracking on synthetic motion sequences
- End-to-end streaming and mapping pipeline
