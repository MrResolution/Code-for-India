"""Main application entrypoint with decoupled multithreaded architecture for rock-solid GUI stability.

Architecture:
1. UDP Receiver Thread:
   - Continuously receives packets, reassembles JPEG frames.
   - Automatically drops incomplete/stale frames without affecting the display.
   - Measures UDP FPS.

2. Shared State Buffers:
   - DisplayBuffer: Thread-safe container holding the latest processed frame and telemetry.
   - MapBuffer: Thread-safe snapshot of 3D landmarks, trajectory, and camera pose for Open3D.

3. SLAM Processing Thread:
   - Runs independently in background; never blocks GUI rendering.
   - Fetches the freshest raw frame, dropping intermediate frames if camera FPS > processing FPS.
   - Executes camera undistortion, ORB detection, KNN matching, VO pose estimation, ToF fusion, and triangulation.
   - Measures Processing FPS.

4. Main GUI Thread:
   - Exclusively handles all OpenCV GUI calls (cv2.imshow, cv2.waitKey) and Open3D updates.
   - Runs at a steady ~30 FPS.
   - Always re-renders the last valid processed frame, completely eliminating flickering or black flashing.
   - Measures Display FPS independently.
"""

import os
import sys
import time
import signal
import threading
from dataclasses import dataclass
from typing import Optional, List, Tuple
import cv2
import numpy as np

# Ensure module imports resolve correctly regardless of working directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config.camera_config import SLAMConfig
from udp_receiver import UDPReceiver
from camera import Camera
from visual_odometry import VisualOdometry
from sensor_fusion import SingleRayToFFusion
from mapping import SparseMap
from visualization import Visualizer3D, OpenCVHUD


@dataclass
class DisplayState:
    """Thread-safe snapshot of the latest processed frame and telemetry."""
    raw_undistorted: np.ndarray
    matches_frame: Optional[np.ndarray]
    distance_mm: int
    is_tof_valid: bool
    num_features: int
    num_matches: int
    curr_pos: np.ndarray
    metric_scale: float
    udp_fps: float
    processing_fps: float
    timestamp: float


class DisplayBuffer:
    """Thread-safe buffer holding the latest visual frame and telemetry for GUI display."""
    def __init__(self):
        self._lock = threading.Lock()
        self._state: Optional[DisplayState] = None

    def update(self, state: DisplayState) -> None:
        with self._lock:
            self._state = state

    def get_state(self) -> Optional[DisplayState]:
        with self._lock:
            return self._state


class MapBuffer:
    """Thread-safe buffer holding snapshots of 3D points and trajectory for Open3D rendering."""
    def __init__(self):
        self._lock = threading.Lock()
        self.points = np.empty((0, 3), dtype=np.float64)
        self.colors = np.empty((0, 3), dtype=np.float64)
        self.trajectory: List[np.ndarray] = [np.zeros(3, dtype=np.float64)]
        self.current_T_w_c = np.eye(4, dtype=np.float64)

    def update(self, points: np.ndarray, colors: np.ndarray, trajectory: List[np.ndarray], T_w_c: np.ndarray) -> None:
        with self._lock:
            self.points = points
            self.colors = colors
            self.trajectory = list(trajectory)
            self.current_T_w_c = T_w_c.copy()

    def get_snapshot(self) -> Tuple[np.ndarray, np.ndarray, List[np.ndarray], np.ndarray]:
        with self._lock:
            return self.points, self.colors, self.trajectory, self.current_T_w_c


def main():
    print("=" * 70)
    print("   ESP32-CAM + VL53L1X Visual SLAM & 3D Mapping System (Stable GUI)")
    print("=" * 70)

    # 1. Initialize configuration and core subsystems
    config = SLAMConfig()
    print(f"[Init] UDP Server: {config.udp_ip}:{config.udp_port}")
    print(f"[Init] Camera Intrinsic: fx={config.camera.fx}, fy={config.camera.fy}, cx={config.camera.cx}, cy={config.camera.cy}")
    print(f"[Init] ORB Features: {config.orb_nfeatures}, Lowe's Ratio: {config.lowe_ratio}")
    print(f"[Init] ToF Range: {config.tof_min_distance_mm}mm - {config.tof_max_distance_mm}mm")

    camera = Camera(config.camera)
    udp_receiver = UDPReceiver(config)
    vo = VisualOdometry(config)
    sensor_fusion = SingleRayToFFusion(config)
    sparse_map = SparseMap(config)
    hud = OpenCVHUD(config, enable_windows=True)
    visualizer_3d = Visualizer3D(config)

    # Thread-safe decoupled buffers
    display_buffer = DisplayBuffer()
    map_buffer = MapBuffer()

    # Graceful shutdown handler
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        print("\n[Main] Shutdown signal received. Exiting...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 2. Start UDP receiver thread
    udp_receiver.start()

    # 3. Independent SLAM Processing Thread
    def slam_worker():
        nonlocal running
        proc_frame_counter = 0
        proc_fps_timer = time.time()
        processing_fps = 0.0

        print("[SLAMWorker] Processing thread started.")

        while running:
            # Ingest newest complete frame from UDP receiver (non-blocking / short timeout)
            # Intermediate frames are automatically dropped if camera FPS exceeds processing capacity
            frame_data = udp_receiver.get_latest_frame(timeout=0.04)
            if frame_data is None:
                continue

            raw_bgr = frame_data.image
            distance_mm = frame_data.distance_mm
            timestamp = frame_data.timestamp

            # Dynamically adapt camera matrix K if frame resolution differs from calibration
            h_raw, w_raw = raw_bgr.shape[:2]
            if camera.adapt_resolution(w_raw, h_raw):
                vo.update_camera_matrix(camera.K)
                sparse_map.update_camera_matrix(camera.K)

            # Camera Undistortion & Preprocessing
            undistorted_bgr = camera.undistort(raw_bgr)
            gray_frame = camera.to_grayscale(undistorted_bgr, enhance=True)

            # VL53L1X Sensor Fusion
            central_depths = sparse_map.get_central_cone_depths(
                vo.T_c_w, radius_px=config.tof_roi_radius_pixels
            )
            fusion_state = sensor_fusion.update(
                t_rel_unit=vo.last_relative_t,
                raw_tof_mm=distance_mm,
                timestamp=timestamp,
                central_feature_depths=central_depths
            )

            # Monocular Visual Odometry Tracking
            prev_T_c_w = vo.T_c_w.copy()
            tracking_success, curr_T_w_c, curr_T_c_w = vo.track(
                gray_frame,
                metric_scale=fusion_state.estimated_metric_scale
            )

            # Sparse 3D Triangulation
            if tracking_success and vo.last_matched_pts_prev is not None and vo.last_matched_pts_curr is not None:
                sparse_map.triangulate(
                    T_c_w_prev=prev_T_c_w,
                    T_c_w_curr=curr_T_c_w,
                    pts_prev=vo.last_matched_pts_prev,
                    pts_curr=vo.last_matched_pts_curr,
                    curr_bgr=undistorted_bgr
                )

            # Render feature matches
            matches_frame = vo.draw_matches_image(undistorted_bgr)

            # Calculate Processing FPS
            proc_frame_counter += 1
            now = time.time()
            if now - proc_fps_timer >= 1.0:
                processing_fps = proc_frame_counter / (now - proc_fps_timer)
                proc_frame_counter = 0
                proc_fps_timer = now

            # Store newest result into shared display buffer
            curr_pos = curr_T_w_c[:3, 3].copy()
            num_kps = len(vo.prev_kps) if vo.prev_kps is not None else 0
            num_inliers = len(vo.last_inlier_matches)

            state = DisplayState(
                raw_undistorted=undistorted_bgr,
                matches_frame=matches_frame,
                distance_mm=distance_mm,
                is_tof_valid=fusion_state.is_valid_reading,
                num_features=num_kps,
                num_matches=num_inliers,
                curr_pos=curr_pos,
                metric_scale=fusion_state.estimated_metric_scale,
                udp_fps=udp_receiver.udp_fps,
                processing_fps=processing_fps,
                timestamp=now
            )
            display_buffer.update(state)

            # Update map buffer snapshot for 3D visualizer
            pts, cols = sparse_map.get_points_and_colors()
            map_buffer.update(pts, cols, vo.trajectory, curr_T_w_c)

        print("[SLAMWorker] Processing thread exited.")

    slam_thread = threading.Thread(target=slam_worker, name="SLAMWorkerThread", daemon=True)
    slam_thread.start()

    # 4. Main GUI Thread Loop (Running at steady ~30 FPS)
    # Placeholder initial waiting banners
    waiting_feed = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(waiting_feed, "Waiting for ESP32-CAM UDP Stream...", (40, 220),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
    cv2.putText(waiting_feed, f"Listening on {config.udp_ip}:{config.udp_port}", (160, 260),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)

    waiting_matches = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(waiting_matches, "Feature Matches Initializing...", (120, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 2)

    disp_frame_counter = 0
    disp_fps_timer = time.time()
    display_fps = 0.0

    last_known_state: Optional[DisplayState] = None

    print("\n[Main] GUI active. Press 'q' or 'ESC' in any OpenCV window to exit.\n")

    try:
        while running:
            t_loop_start = time.time()

            # 1. Fetch newest state from DisplayBuffer (or retain last known state)
            new_state = display_buffer.get_state()
            if new_state is not None:
                last_known_state = new_state

            # 2. Fetch current 3D map points, trajectory, and pose snapshot
            map_pts, map_cols, traj, T_w_c = map_buffer.get_snapshot()

            if last_known_state is not None:
                # Continuous smooth re-rendering of last valid frame (ZERO flickering!)
                hud_frame = hud.draw_hud(
                    frame_bgr=last_known_state.raw_undistorted,
                    udp_fps=last_known_state.udp_fps,
                    vo_fps=last_known_state.processing_fps,
                    distance_mm=last_known_state.distance_mm,
                    is_tof_valid=last_known_state.is_tof_valid,
                    num_features=last_known_state.num_features,
                    num_matches=last_known_state.num_matches,
                    curr_pos=last_known_state.curr_pos,
                    metric_scale=last_known_state.metric_scale,
                    display_fps=display_fps
                )
                matches_to_show = (
                    last_known_state.matches_frame
                    if last_known_state.matches_frame is not None
                    else waiting_matches
                )
                # Render 2D Top-Down SLAM Map Plot (Trajectory, Heading, Landmarks, ToF Ray)
                map_2d_frame = hud.draw_topdown_map(
                    points=map_pts,
                    trajectory=traj,
                    current_T_w_c=T_w_c,
                    distance_mm=last_known_state.distance_mm,
                    is_tof_valid=last_known_state.is_tof_valid
                )
                key = hud.show(hud_frame, matches_to_show, map_2d_frame)
            else:
                # Prior to first frame reception, display static waiting screen
                waiting_map = hud.draw_topdown_map(
                    points=map_pts,
                    trajectory=traj,
                    current_T_w_c=T_w_c,
                    distance_mm=0,
                    is_tof_valid=False
                )
                key = hud.show(waiting_feed, waiting_matches, waiting_map)

            if key in (ord('q'), 27):
                print("[Main] User requested termination via keyboard.")
                running = False
                break

            # 3. Update Open3D 3D SLAM Visualizer on the main thread (throttled: every 3rd frame)
            # Open3D poll_events() is expensive; running it every frame drops GUI FPS from 30 to ~4.
            if disp_frame_counter % 3 == 0:
                keep_vis = visualizer_3d.update(
                    points=map_pts,
                    colors=map_cols,
                    trajectory=traj,
                    current_T_w_c=T_w_c
                )
                if not keep_vis:
                    print("[Main] User closed 3D visualizer window.")
                    running = False
                    break

            # 4. Track Display FPS
            disp_frame_counter += 1
            now = time.time()
            if now - disp_fps_timer >= 1.0:
                display_fps = disp_frame_counter / (now - disp_fps_timer)
                disp_frame_counter = 0
                disp_fps_timer = now

            # 5. Target ~30 FPS GUI refresh rate
            elapsed = time.time() - t_loop_start
            sleep_remaining = (1.0 / 30.0) - elapsed
            if sleep_remaining > 0:
                time.sleep(sleep_remaining)


    except Exception as e:
        print(f"[Main] Unexpected runtime exception: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("[Main] Shutting down subsystems...")
        running = False
        udp_receiver.stop()
        if slam_thread.is_alive():
            slam_thread.join(timeout=1.0)
        hud.close()
        visualizer_3d.close()
        print("[Main] Clean shutdown complete.")


if __name__ == "__main__":
    main()
