"""Sparse 3D Point Triangulation and Global Map Management module."""

from typing import Tuple, Optional, List, Dict
import cv2
import numpy as np

from config.camera_config import SLAMConfig


class MapPoint:
    """Represents a triangulated 3D landmark in world coordinates."""
    def __init__(self, position_w: np.ndarray, color_rgb: np.ndarray):
        self.position = np.asarray(position_w, dtype=np.float64)  # (3,) in world coords
        self.color = np.asarray(color_rgb, dtype=np.float64)        # (3,) normalized [0, 1]
        self.observation_count = 1


class SparseMap:
    """Maintains the global sparse 3D point cloud reconstructed from camera motion."""

    def __init__(self, config: Optional[SLAMConfig] = None):
        self.config = config or SLAMConfig()
        self.K = self.config.camera.K
        self.min_depth = self.config.min_triangulation_depth
        self.max_depth = self.config.max_triangulation_depth
        self.max_reproj_err = self.config.max_reprojection_error
        self.min_parallax_rad = np.radians(self.config.min_parallax_degrees)
        self.voxel_size = self.config.voxel_size_m
        self.max_points = self.config.max_map_points

        # Spatial hash map for fast voxel-based deduplication:
        # (ix, iy, iz) -> MapPoint
        self._voxel_grid: Dict[Tuple[int, int, int], MapPoint] = {}

    def update_camera_matrix(self, K: np.ndarray) -> None:
        """Updates the camera intrinsic matrix K dynamically."""
        self.K = np.asarray(K, dtype=np.float64)

    def triangulate(
        self,
        T_c_w_prev: np.ndarray,
        T_c_w_curr: np.ndarray,
        pts_prev: np.ndarray,
        pts_curr: np.ndarray,
        curr_bgr: Optional[np.ndarray] = None
    ) -> int:
        """Triangulate 2D feature correspondences into 3D world coordinates.
        
        Args:
            T_c_w_prev: World-to-Camera 4x4 matrix for previous frame [R_prev | t_prev].
            T_c_w_curr: World-to-Camera 4x4 matrix for current frame [R_curr | t_curr].
            pts_prev: 2D feature coordinates in previous frame (N, 2).
            pts_curr: 2D feature coordinates in current frame (N, 2).
            curr_bgr: Current color frame to extract RGB colors for 3D points.
            
        Returns:
            Number of newly added 3D points.
        """
        if pts_prev is None or pts_curr is None or len(pts_prev) < 4:
            return 0

        # Construct 3x4 Projection Matrices: P = K [R | t]
        R_prev = T_c_w_prev[:3, :3]
        t_prev = T_c_w_prev[:3, 3:4]
        P_prev = self.K @ np.hstack([R_prev, t_prev])

        R_curr = T_c_w_curr[:3, :3]
        t_curr = T_c_w_curr[:3, 3:4]
        P_curr = self.K @ np.hstack([R_curr, t_curr])

        # Camera centers in world coordinates
        C_prev = -R_prev.T @ t_prev  # (3, 1)
        C_curr = -R_curr.T @ t_curr  # (3, 1)

        # Baseline distance between camera views
        baseline = float(np.linalg.norm(C_curr - C_prev))
        if baseline < 0.005:
            # Baseline is too small for stable triangulation (motion parallax is negligible)
            return 0

        # Triangulate points using cv2.triangulatePoints
        # Inputs must be shape (2, N)
        pts_prev_t = pts_prev.T.astype(np.float64)
        pts_curr_t = pts_curr.T.astype(np.float64)

        homog_pts = cv2.triangulatePoints(P_prev, P_curr, pts_prev_t, pts_curr_t)

        # Homogeneous dehomogenization: (4, N) -> (N, 3)
        w = homog_pts[3, :]
        valid_w = np.abs(w) > 1e-7
        pts_3d_w = np.zeros((pts_prev.shape[0], 3), dtype=np.float64)
        pts_3d_w[valid_w] = (homog_pts[:3, valid_w] / w[valid_w]).T

        added_count = 0
        h_img, w_img = (curr_bgr.shape[:2]) if curr_bgr is not None else (480, 640)

        for i in range(len(pts_prev)):
            if not valid_w[i]:
                continue

            pw = pts_3d_w[i]  # 3D coordinate in world frame

            # 1. Cheirality check: Depth must be positive in BOTH camera frames
            p_cam_prev = R_prev @ pw + t_prev.ravel()
            p_cam_curr = R_curr @ pw + t_curr.ravel()

            z_prev = p_cam_prev[2]
            z_curr = p_cam_curr[2]

            if z_prev <= self.min_depth or z_prev >= self.max_depth:
                continue
            if z_curr <= self.min_depth or z_curr >= self.max_depth:
                continue

            # 2. Parallax angle test: ensure sufficient geometric convergence angle
            ray_prev = pw - C_prev.ravel()
            ray_curr = pw - C_curr.ravel()
            norm_prev = np.linalg.norm(ray_prev)
            norm_curr = np.linalg.norm(ray_curr)

            if norm_prev < 1e-5 or norm_curr < 1e-5:
                continue

            cos_parallax = np.dot(ray_prev, ray_curr) / (norm_prev * norm_curr)
            cos_parallax = np.clip(cos_parallax, -1.0, 1.0)
            parallax_angle = np.arccos(cos_parallax)

            if parallax_angle < self.min_parallax_rad:
                continue

            # 3. Reprojection error check for both views
            # View 1
            u_proj_prev = (self.K[0, 0] * p_cam_prev[0] / z_prev) + self.K[0, 2]
            v_proj_prev = (self.K[1, 1] * p_cam_prev[1] / z_prev) + self.K[1, 2]
            err_prev = np.hypot(u_proj_prev - pts_prev[i, 0], v_proj_prev - pts_prev[i, 1])
            if err_prev > self.max_reproj_err:
                continue

            # View 2
            u_proj_curr = (self.K[0, 0] * p_cam_curr[0] / z_curr) + self.K[0, 2]
            v_proj_curr = (self.K[1, 1] * p_cam_curr[1] / z_curr) + self.K[1, 2]
            err_curr = np.hypot(u_proj_curr - pts_curr[i, 0], v_proj_curr - pts_curr[i, 1])
            if err_curr > self.max_reproj_err:
                continue

            # Sample RGB color from current frame
            u_int = int(round(pts_curr[i, 0]))
            v_int = int(round(pts_curr[i, 1]))
            if curr_bgr is not None and 0 <= u_int < w_img and 0 <= v_int < h_img:
                bgr = curr_bgr[v_int, u_int]
                rgb = np.array([bgr[2], bgr[1], bgr[0]], dtype=np.float64) / 255.0
            else:
                rgb = np.array([0.2, 0.7, 1.0], dtype=np.float64)

            # Spatial voxel hashing to avoid dense duplicates
            voxel_key = (
                int(np.floor(pw[0] / self.voxel_size)),
                int(np.floor(pw[1] / self.voxel_size)),
                int(np.floor(pw[2] / self.voxel_size))
            )

            if voxel_key in self._voxel_grid:
                # Average position for noise reduction
                existing = self._voxel_grid[voxel_key]
                existing.position = 0.5 * (existing.position + pw)
                existing.observation_count += 1
            else:
                if len(self._voxel_grid) >= self.max_points:
                    # Drop oldest entry if boundary is hit
                    self._voxel_grid.pop(next(iter(self._voxel_grid)))
                self._voxel_grid[voxel_key] = MapPoint(pw, rgb)
                added_count += 1

        return added_count

    def get_points_and_colors(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns all global 3D map points and RGB colors as numpy arrays.
        
        Returns:
            points: (N, 3) float64 array of 3D positions in world frame.
            colors: (N, 3) float64 array of RGB colors normalized [0, 1].
        """
        if not self._voxel_grid:
            return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.float64)

        points = np.array([mp.position for mp in self._voxel_grid.values()], dtype=np.float64)
        colors = np.array([mp.color for mp in self._voxel_grid.values()], dtype=np.float64)
        return points, colors

    def get_central_cone_depths(self, T_c_w: np.ndarray, radius_px: float = 80.0) -> List[float]:
        """Returns camera-frame depths of landmarks located near the central optical axis (ToF ray)."""
        if not self._voxel_grid:
            return []

        R = T_c_w[:3, :3]
        t = T_c_w[:3, 3]
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        fx = self.K[0, 0]
        fy = self.K[1, 1]

        depths = []
        for mp in self._voxel_grid.values():
            pc = R @ mp.position + t
            z = pc[2]
            if z <= 0.05:
                continue
            u = (fx * pc[0] / z) + cx
            v = (fy * pc[1] / z) + cy
            dist_to_center = np.hypot(u - cx, v - cy)
            if dist_to_center <= radius_px:
                depths.append(float(z))
        return depths

    @property
    def point_count(self) -> int:
        return len(self._voxel_grid)
