"""Camera preprocessing, calibration, and undistortion module."""

from typing import Tuple, Optional
import cv2
import numpy as np

from config.camera_config import CameraCalibration, SLAMConfig


class Camera:
    """Handles camera calibration, image undistortion, and color conversions."""

    def __init__(self, calibration: Optional[CameraCalibration] = None):
        self.calib = calibration or CameraCalibration()
        self.K = self.calib.K
        self.D = self.calib.D
        self.width = self.calib.image_width
        self.height = self.calib.image_height

        # Precompute undistortion map for maximum runtime performance if resolution matches
        self._has_distortion = np.any(self.D != 0.0)
        self._map1: Optional[np.ndarray] = None
        self._map2: Optional[np.ndarray] = None
        if self._has_distortion:
            self._init_undistort_maps(self.width, self.height)

        # Resolution lock: once adapted to the first live frame resolution, never adapt again.
        # This prevents K from cycling every frame when the mock streamer alternates resolutions,
        # which would destroy Visual Odometry pose continuity.
        self._resolution_locked: bool = False

    def _init_undistort_maps(self, width: int, height: int) -> None:
        """Precompute rectification map for real-time low-latency undistortion."""
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, np.eye(3), self.K, (width, height), cv2.CV_32FC1
        )

    def undistort(self, image: np.ndarray, use_fast_map: bool = True) -> np.ndarray:
        """Undistorts the incoming frame using camera calibration parameters.
        
        Uses cv2.undistort() directly or fast precomputed maps if available.
        """
        if not self._has_distortion:
            return image

        h, w = image.shape[:2]
        if use_fast_map and self._map1 is not None and (w, h) == (self.width, self.height):
            return cv2.remap(image, self._map1, self._map2, interpolation=cv2.INTER_LINEAR)
        else:
            # Direct cv2.undistort() implementation
            return cv2.undistort(image, self.K, self.D)

    def adapt_resolution(self, width: int, height: int) -> bool:
        """Adapts intrinsic matrix K to match incoming image resolution if different.
        
        After the first successful adaptation, the resolution is locked to prevent
        the K matrix from cycling between sizes (which destroys VO pose continuity).
        """
        if self._resolution_locked:
            return False
        if (width, height) == (self.width, self.height):
            return False

        scale_x = width / float(self.width)
        scale_y = height / float(self.height)
        self.calib.fx *= scale_x
        self.calib.fy *= scale_y
        self.calib.cx = width / 2.0
        self.calib.cy = height / 2.0
        self.width = width
        self.height = height
        self.K = self.calib.K
        self._resolution_locked = True  # Lock: never adapt again after first live frame

        if self._has_distortion:
            self._init_undistort_maps(self.width, self.height)

        print(f"[Camera] Adapted & locked intrinsics to frame resolution {width}x{height}: fx={self.K[0,0]:.1f}, fy={self.K[1,1]:.1f}, cx={self.K[0,2]:.1f}, cy={self.K[1,2]:.1f}")
        return True

    @staticmethod
    def to_grayscale(image: np.ndarray, enhance: bool = True) -> np.ndarray:
        """Convert BGR image to single-channel grayscale with CLAHE contrast enhancement."""
        if len(image.shape) == 2:
            gray = image
        else:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if enhance:
            clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
            return clahe.apply(gray)
        return gray

    def pixel_to_normalized(self, points: np.ndarray) -> np.ndarray:
        """Unproject 2D pixel coordinates to normalized camera ray coordinates (x/z, y/z).
        
        Args:
            points: Array of shape (N, 2) in pixel space.
        Returns:
            Array of shape (N, 2) in normalized camera plane coordinates.
        """
        pts = np.asarray(points, dtype=np.float64)
        if pts.ndim == 1:
            pts = pts.reshape(1, 2)
        
        # [u, v] -> [(u - cx) / fx, (v - cy) / fy]
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        norm_pts = np.empty_like(pts)
        norm_pts[:, 0] = (pts[:, 0] - cx) / fx
        norm_pts[:, 1] = (pts[:, 1] - cy) / fy
        return norm_pts

    def normalized_to_pixel(self, points: np.ndarray) -> np.ndarray:
        """Project normalized camera coordinates back to pixel space."""
        pts = np.asarray(points, dtype=np.float64)
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        px_pts = np.empty_like(pts)
        px_pts[:, 0] = pts[:, 0] * fx + cx
        px_pts[:, 1] = pts[:, 1] * fy + cy
        return px_pts
