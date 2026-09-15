import os
from typing import List, Optional, Tuple
import cv2
import numpy as np

# Prevent OpenCV from overriding Qt's platform plugin search path
if "QT_QPA_PLATFORM_PLUGIN_PATH" in os.environ and "cv2" in os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]:
    del os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

from config.camera_config import SLAMConfig


class Visualizer3D:
    """Real-time 3D SLAM visualizer powered by Open3D.
    
    Displays:
    - Sparse 3D Point Cloud with RGB colors
    - Accumulated Camera Trajectory Path
    - Current Camera Coordinate Axes (X=Red, Y=Green, Z=Blue)
    - 3D Wireframe Camera Frustum illustrating real-time orientation
    """

    def __init__(self, config: Optional[SLAMConfig] = None, window_name: str = "3D SLAM Map (Open3D)"):
        self.config = config or SLAMConfig()
        self.window_name = window_name
        self.enabled = self.config.show_open3d_window and HAS_OPEN3D
        self.frustum_scale = self.config.camera_frustum_scale
        self.vis = None

        if not HAS_OPEN3D:
            print("[Visualizer3D] Warning: open3d is not installed. 3D window disabled.")
            return

        try:
            self.vis = o3d.visualization.Visualizer()
            success = self.vis.create_window(window_name=self.window_name, width=800, height=600)
            if not success:
                print("[Visualizer3D] Warning: create_window failed. 3D window disabled.")
                self.enabled = False
                return

            # Coordinate axes at world origin (size = 0.5 meters)
            # Red = +X (Right), Green = +Y (Down in camera standard), Blue = +Z (Forward)
            self.world_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4, origin=[0, 0, 0])
            self.vis.add_geometry(self.world_axes)

            # Geometries
            self.point_cloud = o3d.geometry.PointCloud()
            self._pc_added = False

            self.trajectory_lines = o3d.geometry.LineSet()
            self._traj_added = False

            # Dynamic camera frustum wireframe
            self.frustum_lines = o3d.geometry.LineSet()
            f_pts, f_lines, f_colors = self._create_frustum_geometry(np.eye(4))
            self.frustum_lines.points = o3d.utility.Vector3dVector(f_pts)
            self.frustum_lines.lines = o3d.utility.Vector2iVector(f_lines)
            self.frustum_lines.colors = o3d.utility.Vector3dVector(f_colors)
            self.vis.add_geometry(self.frustum_lines)
            self._frustum_added = True

            # Camera view render options
            render_opt = self.vis.get_render_option()
            if render_opt is not None:
                render_opt.background_color = np.asarray([0.08, 0.08, 0.1])  # Dark slate background
                render_opt.point_size = 3.5

            self._first_frame = True
        except Exception as e:
            print(f"[Visualizer3D] Warning: Failed to initialize Open3D window: {e}")
            self.enabled = False

    def _create_frustum_geometry(self, T_w_c: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Creates vertices, line indices, and colors for a camera frustum in world frame."""
        s = self.frustum_scale
        # Local frustum coordinates (apex at camera center, base projected forward along +Z)
        # 0: Apex (camera center)
        # 1: Top-Left, 2: Top-Right, 3: Bottom-Right, 4: Bottom-Left
        local_corners = np.array([
            [0.0, 0.0, 0.0],          # 0: Apex
            [-s * 0.8, -s * 0.6, s],   # 1: Top-Left
            [s * 0.8, -s * 0.6, s],    # 2: Top-Right
            [s * 0.8, s * 0.6, s],     # 3: Bottom-Right
            [-s * 0.8, s * 0.6, s]     # 4: Bottom-Left
        ], dtype=np.float64)

        # Transform to world coordinates: P_w = R_w_c * P_c + t_w_c
        R_w_c = T_w_c[:3, :3]
        t_w_c = T_w_c[:3, 3]
        world_corners = (R_w_c @ local_corners.T).T + t_w_c

        # Lines connecting apex to base, and base perimeter
        lines = np.array([
            [0, 1], [0, 2], [0, 3], [0, 4],  # Pyramid edges
            [1, 2], [2, 3], [3, 4], [4, 1]   # Image sensor plane perimeter
        ], dtype=np.int32)

        # Yellow wireframe for camera frustum
        colors = np.tile([1.0, 0.85, 0.1], (len(lines), 1))
        return world_corners, lines, colors

    def update(
        self,
        points: np.ndarray,
        colors: np.ndarray,
        trajectory: List[np.ndarray],
        current_T_w_c: np.ndarray
    ) -> bool:
        """Updates 3D geometries and executes an Open3D render cycle.
        
        Returns False if the user closed the window.
        """
        if not self.enabled:
            return True

        # 1. Update sparse 3D point cloud
        if len(points) > 0:
            self.point_cloud.points = o3d.utility.Vector3dVector(points)
            if len(colors) == len(points):
                self.point_cloud.colors = o3d.utility.Vector3dVector(colors)
            if not self._pc_added:
                self.vis.add_geometry(self.point_cloud)
                self._pc_added = True
            else:
                self.vis.update_geometry(self.point_cloud)

        # 2. Update camera trajectory line set
        if len(trajectory) >= 2:
            traj_pts = np.asarray(trajectory, dtype=np.float64)
            traj_lines = np.array([[i, i + 1] for i in range(len(trajectory) - 1)], dtype=np.int32)
            traj_colors = np.tile(self.config.trajectory_line_color, (len(traj_lines), 1))

            self.trajectory_lines.points = o3d.utility.Vector3dVector(traj_pts)
            self.trajectory_lines.lines = o3d.utility.Vector2iVector(traj_lines)
            self.trajectory_lines.colors = o3d.utility.Vector3dVector(traj_colors)
            if not self._traj_added:
                self.vis.add_geometry(self.trajectory_lines)
                self._traj_added = True
            else:
                self.vis.update_geometry(self.trajectory_lines)

        # 3. Update current camera frustum
        f_pts, f_lines, f_colors = self._create_frustum_geometry(current_T_w_c)
        self.frustum_lines.points = o3d.utility.Vector3dVector(f_pts)
        self.frustum_lines.lines = o3d.utility.Vector2iVector(f_lines)
        self.frustum_lines.colors = o3d.utility.Vector3dVector(f_colors)
        self.vis.update_geometry(self.frustum_lines)

        # Reset camera view when points are first triangulated to frame the scene
        if not hasattr(self, "_points_focused"):
            self._points_focused = False
        if not self._points_focused and len(points) >= 2:
            self.vis.reset_view_point(True)
            self._points_focused = True

        # Non-blocking render step
        keep_running = self.vis.poll_events()
        self.vis.update_renderer()
        return keep_running

    def close(self) -> None:
        """Closes visualizer window cleanly."""
        if self.enabled:
            try:
                self.vis.destroy_window()
            except Exception:
                pass


class OpenCVHUD:
    """Manages OpenCV UI windows and real-time Heads-Up Display (HUD) overlays."""

    def __init__(self, config: Optional[SLAMConfig] = None, enable_windows: bool = False):
        self.config = config or SLAMConfig()
        self.enable_windows = enable_windows
        self.window_feed = "Window 1: ESP32-CAM Live Feed"
        self.window_matches = "Window 2: Feature Matches"
        self.window_map2d = "Window 3: Top-Down SLAM Map Plot"

        if self.enable_windows:
            try:
                cv2.namedWindow(self.window_feed, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_feed, 480, 360)

                cv2.namedWindow(self.window_matches, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_matches, 480, 360)

                cv2.namedWindow(self.window_map2d, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_map2d, 520, 520)
            except Exception as e:
                print(f"[OpenCVHUD] Warning: Failed to create GUI windows: {e}")
                self.enable_windows = False

    def draw_hud(
        self,
        frame_bgr: np.ndarray,
        udp_fps: float,
        vo_fps: float,
        distance_mm: int,
        is_tof_valid: bool,
        num_features: int,
        num_matches: int,
        curr_pos: np.ndarray,
        metric_scale: float,
        display_fps: float = 0.0
    ) -> np.ndarray:
        """Renders an informative HUD overlay on the live camera frame."""
        vis_frame = frame_bgr.copy()
        h, w = vis_frame.shape[:2]

        # Draw dark semi-transparent telemetry overlay banner at top
        overlay = vis_frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 110), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.65, vis_frame, 0.35, 0, vis_frame)

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.48
        thick = 1

        # Line 1: Frame Rates & Tracking Status (UDP, Processing, Display)
        fps_text = f"UDP: {udp_fps:4.1f} FPS | Proc: {vo_fps:4.1f} FPS | GUI: {display_fps:4.1f} FPS"
        cv2.putText(vis_frame, fps_text, (12, 24), font, scale, (0, 255, 255), thick, cv2.LINE_AA)

        # Line 2: ToF Distance & Validity
        tof_status = "VALID" if is_tof_valid else "OUT OF RANGE"
        tof_color = (0, 255, 0) if is_tof_valid else (0, 0, 255)
        tof_text = f"VL53L1X: {distance_mm} mm ({distance_mm/1000.0:4.2f} m) [{tof_status}]"
        cv2.putText(vis_frame, tof_text, (12, 50), font, scale, tof_color, thick, cv2.LINE_AA)

        # Line 3: ORB Features & Inlier Matches
        match_text = f"ORB Features: {num_features} | Inliers: {num_matches}"
        cv2.putText(vis_frame, match_text, (12, 76), font, scale, (255, 200, 100), thick, cv2.LINE_AA)

        # Line 4: Camera Position & Scale
        pos_text = f"Pos [m]: X={curr_pos[0]:+.2f} Y={curr_pos[1]:+.2f} Z={curr_pos[2]:+.2f} | Scale={metric_scale:.3f}m"
        cv2.putText(vis_frame, pos_text, (12, 100), font, 0.44, (220, 220, 220), thick, cv2.LINE_AA)

        # Draw optical center ToF target reticle
        cx, cy = int(w / 2.0), int(h / 2.0)
        cv2.drawMarker(vis_frame, (cx, cy), tof_color, cv2.MARKER_CROSS, 16, 1, cv2.LINE_AA)
        cv2.circle(vis_frame, (cx, cy), min(40, int(self.config.tof_roi_radius_pixels * w / 640.0)), tof_color, 1, cv2.LINE_AA)

        return vis_frame

    def draw_topdown_map(
        self,
        points: np.ndarray,
        trajectory: List[np.ndarray],
        current_T_w_c: np.ndarray,
        distance_mm: int,
        is_tof_valid: bool,
        canvas_size: int = 520
    ) -> np.ndarray:
        """Renders an interactive 2D top-down (Bird's Eye) SLAM map plot.

        The camera is always drawn at the center of the canvas. All world
        coordinates (landmarks, trajectory) are shifted relative to the camera
        so the map never goes off-canvas regardless of how far the camera drifts.
        
        Plots:
        - Metric scale grid lines with meter markings
        - 3D landmarks projected onto ground plane (X, Z)
        - Continuous camera trajectory path
        - Current camera position, heading direction vector, and ToF distance ray
        """
        canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)
        canvas[:] = (22, 22, 28)  # Dark slate plot background

        # Plot parameters: pixels per meter
        ppm = 60.0

        # Camera is always at canvas center
        cam_cx = canvas_size // 2
        cam_cz = canvas_size // 2

        # Current camera world position — used as the canvas origin
        cam_pos_w = current_T_w_c[:3, 3]

        def world_to_canvas(pt_w: np.ndarray):
            """Map a 3D world point to 2D canvas pixel, relative to camera position."""
            dx = pt_w[0] - cam_pos_w[0]
            dz = pt_w[2] - cam_pos_w[2]
            px = int(cam_cx + dx * ppm)
            pz = int(cam_cz - dz * ppm)
            return px, pz

        # 1. Draw metric grid (relative to camera position; snapped to meter grid)
        grid_step_px = int(ppm * 0.5)  # 0.5 meter intervals
        # Offset grid so lines fall on metric boundaries
        offset_x = int((cam_pos_w[0] % 0.5) * ppm)
        offset_z = int((cam_pos_w[2] % 0.5) * ppm)
        for x in range((cam_cx - offset_x) % grid_step_px, canvas_size, grid_step_px):
            color = (50, 50, 60) if abs(x - cam_cx) < 2 else (35, 35, 42)
            cv2.line(canvas, (x, 0), (x, canvas_size), color, 1)
        for y in range((cam_cz - offset_z) % grid_step_px, canvas_size, grid_step_px):
            color = (50, 50, 60) if abs(y - cam_cz) < 2 else (35, 35, 42)
            cv2.line(canvas, (0, y), (canvas_size, y), color, 1)

        # World origin marker (may drift off canvas as camera moves)
        ox, oz = world_to_canvas(np.zeros(3))
        if 4 <= ox < canvas_size - 4 and 4 <= oz < canvas_size - 4:
            cv2.drawMarker(canvas, (ox, oz), (100, 100, 120), cv2.MARKER_CROSS, 14, 1)
            cv2.putText(canvas, "Origin", (ox + 6, oz + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 140), 1)

        # 2. Draw 3D map points projected on ground plane (X, Z)
        if len(points) > 0:
            for pt in points:
                px, pz = world_to_canvas(pt)
                if 0 <= px < canvas_size and 0 <= pz < canvas_size:
                    cv2.circle(canvas, (px, pz), 2, (0, 220, 255), -1, cv2.LINE_AA)

        # 3. Draw Camera Trajectory path (clipped to canvas bounds)
        if len(trajectory) >= 2:
            traj_pts_2d = []
            for t_pt in trajectory:
                tx, tz = world_to_canvas(t_pt)
                traj_pts_2d.append([tx, tz])
            traj_pts_arr = np.array(traj_pts_2d, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(canvas, [traj_pts_arr], isClosed=False,
                          color=(0, 255, 128), thickness=2, lineType=cv2.LINE_AA)

        # 4. Draw Current Camera Position and Heading (always at canvas center)
        # Heading vector: camera optical axis (+Z in camera frame) projected into world
        R_w_c = current_T_w_c[:3, :3]
        forward_w = R_w_c @ np.array([0.0, 0.0, 1.0])
        heading_len_px = 28
        hx = int(cam_cx + forward_w[0] * heading_len_px)
        hz = int(cam_cz - forward_w[2] * heading_len_px)

        # Draw ToF distance ray forward from camera center
        dist_m = distance_mm / 1000.0
        if is_tof_valid and dist_m > 0.04:
            tof_px = int(cam_cx + forward_w[0] * (dist_m * ppm))
            tof_pz = int(cam_cz - forward_w[2] * (dist_m * ppm))
            cv2.line(canvas, (cam_cx, cam_cz), (tof_px, tof_pz), (0, 255, 255), 1, cv2.LINE_AA)
            cv2.circle(canvas, (tof_px, tof_pz), 5, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(canvas, f"{dist_m:.2f}m", (tof_px + 6, tof_pz - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1, cv2.LINE_AA)

        # Draw camera body (cyan circle + heading arrow)
        cv2.circle(canvas, (cam_cx, cam_cz), 7, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(canvas, (cam_cx, cam_cz), 7, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.arrowedLine(canvas, (cam_cx, cam_cz), (hx, hz), (0, 120, 255), 2, tipLength=0.35)

        # 5. Telemetry Banner on Map
        cv2.rectangle(canvas, (0, 0), (canvas_size, 34), (15, 15, 20), -1)
        pos_text = f"X={cam_pos_w[0]:+.2f} Z={cam_pos_w[2]:+.2f} m  |  {len(points)} pts  |  {len(trajectory)} steps"
        cv2.putText(canvas, pos_text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 200), 1, cv2.LINE_AA)
        scale_info = "1 box=0.5m"
        cv2.putText(canvas, scale_info, (canvas_size - 90, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 170), 1, cv2.LINE_AA)

        return canvas



    def show(self, hud_image: np.ndarray, match_image: Optional[np.ndarray] = None, map_image: Optional[np.ndarray] = None) -> int:
        """Displays OpenCV windows and polls for key events. Returns waitKey result."""
        if not self.enable_windows:
            return 255
        try:
            cv2.imshow(self.window_feed, hud_image)
            if match_image is not None:
                cv2.imshow(self.window_matches, match_image)
            if map_image is not None:
                cv2.imshow(self.window_map2d, map_image)
            return cv2.waitKey(1) & 0xFF
        except Exception:
            return 255

    def close(self) -> None:
        """Destroys OpenCV GUI windows."""
        if self.enable_windows:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
