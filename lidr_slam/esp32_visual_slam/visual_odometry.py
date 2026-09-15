"""Visual Odometry module using ORB features, KNN matching, and Essential Matrix decomposition."""

from typing import Tuple, Optional, List
import cv2
import numpy as np

from config.camera_config import SLAMConfig


class VisualOdometry:
    """Monocular Visual Odometry computing 6-DOF camera motion across frames."""

    def __init__(self, config: Optional[SLAMConfig] = None):
        self.config = config or SLAMConfig()
        self.K = self.config.camera.K

        # 1. Initialize ORB Feature Detector
        # Using 1500–2000 features with tuned Harris score and scale levels
        self.orb = cv2.ORB_create(
            nfeatures=self.config.orb_nfeatures,
            scaleFactor=self.config.orb_scale_factor,
            nlevels=self.config.orb_nlevels,
            edgeThreshold=self.config.orb_edge_threshold,
            firstLevel=self.config.orb_first_level,
            WTA_K=self.config.orb_wta_k,
            scoreType=cv2.ORB_HARRIS_SCORE,
            patchSize=self.config.orb_patch_size,
            fastThreshold=self.config.orb_fast_threshold
        )

        # 2. Hamming Distance Matcher for binary ORB descriptors
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # State tracking
        self.prev_gray: Optional[np.ndarray] = None
        self.prev_kps: Optional[List[cv2.KeyPoint]] = None
        self.prev_desc: Optional[np.ndarray] = None

        # Camera poses:
        # T_w_c: Camera-to-World transformation [R_cw | t_cw] (camera position in world coordinates)
        # T_c_w: World-to-Camera transformation [R_wc | t_wc] (projection matrix basis)
        self.T_w_c = np.eye(4, dtype=np.float64)
        self.T_c_w = np.eye(4, dtype=np.float64)

        # Global trajectory history (world positions)
        self.trajectory: List[np.ndarray] = [self.T_w_c[:3, 3].copy()]

        # Rendering cache for feature matching visualization
        self.render_prev_bgr: Optional[np.ndarray] = None
        self.render_prev_kps: Optional[List[cv2.KeyPoint]] = None
        self.render_curr_kps: Optional[List[cv2.KeyPoint]] = None

        # Last tracking metadata for visualization & mapping
        self.last_good_matches: List[cv2.DMatch] = []
        self.last_inlier_matches: List[cv2.DMatch] = []
        self.last_matched_pts_prev: Optional[np.ndarray] = None
        self.last_matched_pts_curr: Optional[np.ndarray] = None
        self.last_relative_R = np.eye(3, dtype=np.float64)
        self.last_relative_t = np.zeros((3, 1), dtype=np.float64)
        self.is_tracking = False

    def update_camera_matrix(self, K: np.ndarray) -> None:
        """Updates the camera intrinsic matrix K dynamically."""
        self.K = np.asarray(K, dtype=np.float64)

    def detect_and_compute(self, gray_image: np.ndarray) -> Tuple[List[cv2.KeyPoint], Optional[np.ndarray]]:
        """Extracts ORB keypoints and descriptors from a grayscale frame."""
        kps, desc = self.orb.detectAndCompute(gray_image, None)
        return kps, desc

    def match_features(
        self, desc_prev: np.ndarray, desc_curr: np.ndarray
    ) -> List[cv2.DMatch]:
        """Matches ORB descriptors between consecutive frames using KNN and Lowe's ratio test."""
        if desc_prev is None or desc_curr is None or len(desc_prev) < 2 or len(desc_curr) < 2:
            return []

        # Find 2 nearest neighbors for each descriptor
        knn_matches = self.matcher.knnMatch(desc_prev, desc_curr, k=2)

        good_matches = []
        for match_pair in knn_matches:
            if len(match_pair) == 2:
                m, n = match_pair
                # Lowe's ratio test
                if m.distance < self.config.lowe_ratio * n.distance:
                    good_matches.append(m)

        return good_matches

    def track(
        self,
        gray_image: np.ndarray,
        metric_scale: Optional[float] = None
    ) -> Tuple[bool, np.ndarray, np.ndarray]:
        """Process incoming camera frame, track motion, and accumulate global camera pose.
        
        Args:
            gray_image: Current grayscale frame (undistorted).
            metric_scale: Translation metric scale (meters) supplied by sensor fusion.
                          Defaults to config.default_step_scale if None or invalid.
        Returns:
            Tuple: (success_flag, current_T_w_c, current_T_c_w)
        """
        curr_kps, curr_desc = self.detect_and_compute(gray_image)

        # First frame initialization
        if self.prev_gray is None or self.prev_desc is None or self.prev_kps is None:
            self.prev_gray = gray_image
            self.prev_kps = curr_kps
            self.prev_desc = curr_desc
            self.is_tracking = True
            return True, self.T_w_c, self.T_c_w

        # Match features with previous frame
        good_matches = self.match_features(self.prev_desc, curr_desc)
        self.last_good_matches = good_matches

        if len(good_matches) < self.config.min_inlier_matches:
            # Insufficient correspondences; keep previous frame reference
            self.is_tracking = False
            return False, self.T_w_c, self.T_c_w

        # Extract 2D coordinate pairs
        pts_prev = np.float32([self.prev_kps[m.queryIdx].pt for m in good_matches])
        pts_curr = np.float32([curr_kps[m.trainIdx].pt for m in good_matches])

        # Estimate Essential Matrix using RANSAC
        E, inlier_mask = cv2.findEssentialMat(
            pts_prev,
            pts_curr,
            self.K,
            method=cv2.RANSAC,
            prob=self.config.ransac_prob,
            threshold=self.config.ransac_threshold_pixels
        )

        if E is None or inlier_mask is None:
            self.is_tracking = False
            return False, self.T_w_c, self.T_c_w

        # Recover relative camera motion: R, t such that P_curr = R * P_prev + t
        num_inliers, R_rel, t_rel, pose_mask = cv2.recoverPose(
            E, pts_prev, pts_curr, self.K, mask=inlier_mask
        )

        if num_inliers < self.config.min_inlier_matches:
            self.is_tracking = False
            return False, self.T_w_c, self.T_c_w

        # Filter strictly for recovered inliers
        inlier_indices = np.where(pose_mask.ravel() > 0)[0]
        self.last_inlier_matches = [good_matches[idx] for idx in inlier_indices]
        self.last_matched_pts_prev = pts_prev[inlier_indices]
        self.last_matched_pts_curr = pts_curr[inlier_indices]

        # Apply metric scale
        # t_rel is returned as a unit vector (norm = 1.0)
        scale = metric_scale if (metric_scale is not None and metric_scale > 0.0) else self.config.default_step_scale
        t_rel_scaled = t_rel * scale

        self.last_relative_R = R_rel
        self.last_relative_t = t_rel_scaled

        # Compose global camera pose:
        # Relative transformation: T_prev_to_curr
        T_prev_to_curr = np.eye(4, dtype=np.float64)
        T_prev_to_curr[:3, :3] = R_rel
        T_prev_to_curr[:3, 3] = t_rel_scaled.ravel()

        # Update World-to-Camera: T_c_w = T_prev_to_curr * T_c_w_prev
        T_c_w_new = T_prev_to_curr @ self.T_c_w

        # Update Camera-to-World (pose in world): T_w_c = (T_c_w)^-1
        T_w_c_new = np.linalg.inv(T_c_w_new)

        # === Trajectory Explosion Guard ===
        # Reject any single-frame displacement > 0.5m as a bad pose estimate.
        # Monocular EssentialMat can produce wildly wrong results on degenerate frames.
        curr_pos = T_w_c_new[:3, 3]
        prev_pos = self.T_w_c[:3, 3]
        displacement = float(np.linalg.norm(curr_pos - prev_pos))
        if displacement > 0.5:
            # This frame produced an unrealistic jump; reject it and keep previous pose
            self.is_tracking = False
            # Still update prev frame so we don't re-match the same bad frame pair
            self.prev_gray = gray_image
            self.prev_kps = list(curr_kps)
            self.prev_desc = curr_desc
            return False, self.T_w_c, self.T_c_w

        self.T_c_w = T_c_w_new
        self.T_w_c = T_w_c_new

        # Record world position in camera trajectory
        self.trajectory.append(curr_pos.copy())

        # Cache rendering state BEFORE advancing self.prev_kps.
        # Deep-copy both keypoint lists and match objects so the render cache
        # is not aliased to the live state (which caused the green blob artifact).
        self.render_prev_bgr = cv2.cvtColor(self.prev_gray, cv2.COLOR_GRAY2BGR)
        self.render_prev_kps = list(self.prev_kps)           # deep copy prev list
        self.render_curr_kps = list(curr_kps)                # deep copy curr list
        self.last_inlier_matches = [                          # deep copy DMatch list
            cv2.DMatch(m.queryIdx, m.trainIdx, m.distance)
            for m in self.last_inlier_matches
        ]

        # Advance state to current frame
        self.prev_gray = gray_image
        self.prev_kps = list(curr_kps)
        self.prev_desc = curr_desc
        self.is_tracking = True

        return True, self.T_w_c, self.T_c_w


    def draw_matches_image(self, curr_bgr: np.ndarray) -> np.ndarray:
        """Visualizes feature correspondences between previous and current frame."""
        if self.render_prev_bgr is None or self.render_prev_kps is None or len(self.last_inlier_matches) == 0:
            return curr_bgr.copy()

        # Draw matches side-by-side using previous and current keypoints
        match_img = cv2.drawMatches(
            self.render_prev_bgr,
            self.render_prev_kps,
            curr_bgr,
            self.render_curr_kps,
            self.last_inlier_matches,
            None,
            matchColor=(0, 255, 0),      # Green for RANSAC inliers
            singlePointColor=(0, 0, 255), # Red for unmatched
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
        return match_img
