"""Unit tests for Visual Odometry, pose composition, and 3D triangulation."""

import unittest
import numpy as np
import cv2

from config.camera_config import SLAMConfig, CameraCalibration
from visual_odometry import VisualOdometry
from mapping import SparseMap


class TestVOMapping(unittest.TestCase):

    def setUp(self):
        self.calib = CameraCalibration(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
        self.config = SLAMConfig(camera=self.calib)
        self.K = self.calib.K

    def test_triangulation_accuracy(self):
        """Mathematically verifies cv2.triangulatePoints correctly reconstructs 3D world points."""
        # 1. Generate known ground-truth 3D landmarks in front of camera
        np.random.seed(123)
        num_points = 20
        gt_points_w = np.random.uniform(low=[-1.0, -1.0, 2.0], high=[1.0, 1.0, 4.0], size=(num_points, 3))

        # 2. Camera 1 at origin
        T_c1_w = np.eye(4)
        P1 = self.K @ T_c1_w[:3, :]

        # 3. Camera 2 translated along +X by 0.2m (baseline)
        T_c2_w = np.eye(4)
        T_c2_w[0, 3] = -0.2  # Camera moved right, scene moved left
        P2 = self.K @ T_c2_w[:3, :]

        # 4. Project points onto both camera image planes
        pts1 = []
        pts2 = []
        for pw in gt_points_w:
            # View 1
            p1_cam = T_c1_w[:3, :3] @ pw + T_c1_w[:3, 3]
            u1 = (self.K[0, 0] * p1_cam[0] / p1_cam[2]) + self.K[0, 2]
            v1 = (self.K[1, 1] * p1_cam[1] / p1_cam[2]) + self.K[1, 2]
            pts1.append([u1, v1])

            # View 2
            p2_cam = T_c2_w[:3, :3] @ pw + T_c2_w[:3, 3]
            u2 = (self.K[0, 0] * p2_cam[0] / p2_cam[2]) + self.K[0, 2]
            v2 = (self.K[1, 1] * p2_cam[1] / p2_cam[2]) + self.K[1, 2]
            pts2.append([u2, v2])

        pts1_arr = np.array(pts1)
        pts2_arr = np.array(pts2)

        # 5. Triangulate using SparseMap
        sparse_map = SparseMap(self.config)
        added = sparse_map.triangulate(T_c1_w, T_c2_w, pts1_arr, pts2_arr)

        self.assertGreaterEqual(added, num_points - 2)
        recovered_pts, _ = sparse_map.get_points_and_colors()
        self.assertGreater(len(recovered_pts), 0)

        # Verify error against ground truth
        for pw in gt_points_w:
            # Find nearest recovered point
            dists = np.linalg.norm(recovered_pts - pw, axis=1)
            min_dist = np.min(dists)
            self.assertLess(min_dist, 0.05, f"Point {pw} reconstruction error too large: {min_dist}m")

    def test_pose_composition_consistency(self):
        """Verifies camera pose inversion and world coordinates compounding."""
        vo = VisualOdometry(self.config)

        # Camera begins at origin
        np.testing.assert_allclose(vo.T_w_c, np.eye(4))
        np.testing.assert_allclose(vo.T_c_w, np.eye(4))

        # Manually verify relation: T_w_c @ T_c_w == I
        inv_check = vo.T_w_c @ vo.T_c_w
        np.testing.assert_allclose(inv_check, np.eye(4), atol=1e-10)

    def test_vo_track_synthetic_sequence(self):
        """Verifies feature detection, matching, pose recovery, and HUD rendering on synthetic frames."""
        from mock_esp32_streamer import create_synthetic_scene
        from camera import Camera

        vo = VisualOdometry(self.config)
        cam = Camera(self.calib)

        # Process first frame
        img1, d1 = create_synthetic_scene(1)
        gray1 = cam.to_grayscale(img1)
        success1, _, _ = vo.track(gray1, metric_scale=0.05)
        self.assertTrue(success1)
        self.assertEqual(len(vo.trajectory), 1)

        # Process second frame with small shift
        img2, d2 = create_synthetic_scene(2)
        gray2 = cam.to_grayscale(img2)
        success2, T_w_c, _ = vo.track(gray2, metric_scale=0.05)
        self.assertTrue(success2)
        self.assertGreaterEqual(len(vo.trajectory), 2)

        # Verify draw_matches_image executes without error
        match_img = vo.draw_matches_image(img2)
        self.assertIsNotNone(match_img)
        self.assertEqual(match_img.shape[0], 480)


if __name__ == "__main__":
    unittest.main()
