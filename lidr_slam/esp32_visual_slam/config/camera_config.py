"""Camera calibration and SLAM configuration parameters.

This file provides centralized configuration for:
- Camera intrinsic matrix K and distortion coefficients D
- UDP receiver parameters and protocol definitions
- ORB feature extraction and matching thresholds
- Visual odometry and RANSAC settings
- VL53L1X ToF distance sensor filtering and fusion
- 3D point cloud mapping limits and voxel filtering
"""

from dataclasses import dataclass, field
import numpy as np
import json
import os


@dataclass
class CameraCalibration:
    """Camera intrinsic and distortion parameters.
    
    Default parameters are estimated for the standard AI-Thinker ESP32-CAM OV2640 module
    running at VGA resolution (640x480) with a typical ~66° diagonal FOV lens.
    
    Replace with your calibrated parameters (e.g. from OpenCV chessboard calibration).
    """
    # Image resolution (defaults to QQVGA 160x120, the native resolution of the
    # ESP32-CAM grayscale firmware; auto-scaled to live frame size on first frame)
    image_width: int = 160
    image_height: int = 120

    # Camera Intrinsic Matrix K:
    # [[fx,  0, cx],
    #  [ 0, fy, cy],
    #  [ 0,  0,  1]]
    fx: float = 130.0   # ≈ 260 * (160/320) — scaled from VGA estimate
    fy: float = 130.0
    cx: float = 80.0    # half of 160
    cy: float = 60.0    # half of 120


    # Distortion coefficients: [k1, k2, p1, p2, k3]
    # Set to zero for ideal pinhole or approximate initial testing
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    k3: float = 0.0

    @property
    def K(self) -> np.ndarray:
        """Returns the 3x3 camera intrinsic matrix."""
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

    @property
    def D(self) -> np.ndarray:
        """Returns the distortion coefficients vector (5,)."""
        return np.array([self.k1, self.k2, self.p1, self.p2, self.k3], dtype=np.float64)

    def save_to_json(self, filepath: str) -> None:
        """Export calibration parameters to a JSON file."""
        data = {
            "image_width": self.image_width,
            "image_height": self.image_height,
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "k1": self.k1,
            "k2": self.k2,
            "p1": self.p1,
            "p2": self.p2,
            "k3": self.k3
        }
        with open(filepath, "w") as f:
            json.dump(data, f, indent=4)

    @classmethod
    def load_from_json(cls, filepath: str) -> "CameraCalibration":
        """Load calibration parameters from a JSON file."""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Calibration file {filepath} not found.")
        with open(filepath, "r") as f:
            data = json.load(f)
        return cls(**data)


@dataclass
class SLAMConfig:
    """Global configuration settings for the Visual SLAM system."""

    # -------------------------------------------------------------
    # 1. Camera Calibration
    # -------------------------------------------------------------
    camera: CameraCalibration = field(default_factory=CameraCalibration)

    # -------------------------------------------------------------
    # 2. UDP Receiver Configuration
    # -------------------------------------------------------------
    udp_ip: str = "0.0.0.0"
    udp_port: int = 5000
    udp_buffer_size: int = 65535  # Socket buffer size
    # Packet header format (little-endian, packed):
    #   uint32 frameID       4 bytes
    #   uint16 packetIndex   2 bytes
    #   uint16 totalPackets  2 bytes
    #   uint16 distanceMM    2 bytes
    #   uint16 width         2 bytes
    #   uint16 height        2 bytes
    #   ─────────────────────────────
    #   TOTAL               14 bytes
    header_format: str = "<IHHHHH"
    header_size: int = 14

    frame_assembly_timeout_sec: float = 0.35  # Drop incomplete frame if not completed within this time
    max_active_frames: int = 5  # Max in-flight frames to track simultaneously to prevent memory buildup

    # -------------------------------------------------------------
    # 3. ORB Feature Detection and Matching
    # -------------------------------------------------------------
    orb_nfeatures: int = 2000
    orb_scale_factor: float = 1.15
    orb_nlevels: int = 8
    orb_edge_threshold: int = 15
    orb_first_level: int = 0
    orb_wta_k: int = 2
    orb_patch_size: int = 31
    orb_fast_threshold: int = 7
    lowe_ratio: float = 0.82  # KNN Lowe's ratio test threshold

    # -------------------------------------------------------------
    # 4. Visual Odometry & RANSAC
    # -------------------------------------------------------------
    min_inlier_matches: int = 8  # Minimum valid matched points required for pose estimation
    ransac_prob: float = 0.999
    ransac_threshold_pixels: float = 2.0  # Reprojection threshold in pixels for Essential Matrix RANSAC
    default_step_scale: float = 0.05  # Default uncalibrated step scale (meters) if ToF scale is unavailable

    # -------------------------------------------------------------
    # 5. VL53L1X ToF Sensor & Sensor Fusion
    # -------------------------------------------------------------
    tof_min_distance_mm: int = 40      # Below 40mm is typically deadband / sensor noise
    tof_max_distance_mm: int = 4000    # VL53L1X max distance in long-distance mode is ~4000mm
    tof_ema_alpha: float = 0.35        # Exponential moving average filter coefficient
    enable_tof_scale_constraint: bool = True  # Use forward ToF delta to scale forward visual motion
    tof_roi_radius_pixels: int = 40    # Central optical cone radius where ToF ray intersects camera image

    # -------------------------------------------------------------
    # 6. Sparse 3D Point Triangulation & Mapping
    # -------------------------------------------------------------
    min_triangulation_depth: float = 0.05  # Minimum depth in meters (reject points closer than 5cm)
    max_triangulation_depth: float = 25.0  # Maximum depth in meters (reject distant outliers)
    max_reprojection_error: float = 3.5    # Maximum reprojection error in pixels for triangulated points
    min_parallax_degrees: float = 0.2      # Minimum viewing angle between two camera rays to avoid ill-conditioned depths
    voxel_size_m: float = 0.02             # Open3D voxel downsampling grid size (2cm)
    max_map_points: int = 50000            # Maximum global points to maintain in memory

    # -------------------------------------------------------------
    # 7. Visualization
    # -------------------------------------------------------------
    show_opencv_windows: bool = True
    show_open3d_window: bool = True
    camera_frustum_scale: float = 0.2      # Size of wireframe frustum in 3D viewer
    trajectory_line_color: list = field(default_factory=lambda: [1.0, 0.0, 0.0])  # Red trajectory line
