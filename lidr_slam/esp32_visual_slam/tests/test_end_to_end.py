"""End-to-End integration test for the full ESP32 Visual SLAM pipeline."""

import unittest
import socket
import struct
import time
import math
import cv2
import numpy as np

from config.camera_config import SLAMConfig
from udp_receiver import UDPReceiver
from camera import Camera
from visual_odometry import VisualOdometry
from sensor_fusion import SingleRayToFFusion
from mapping import SparseMap
from mock_esp32_streamer import create_synthetic_scene, PACKET_PAYLOAD_SIZE, HEADER_FORMAT


class TestEndToEndSLAM(unittest.TestCase):

    def test_full_pipeline_with_mock_stream(self):
        """Simulates full UDP streaming, reception, VO tracking, and 3D triangulation."""
        config = SLAMConfig(udp_port=5005)  # Use dedicated port for test

        camera = Camera(config.camera)
        receiver = UDPReceiver(config)
        vo = VisualOdometry(config)
        fusion = SingleRayToFFusion(config)
        sparse_map = SparseMap(config)

        receiver.start()

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        target = ("127.0.0.1", config.udp_port)

        total_frames_to_send = 15

        import threading
        stop_event = threading.Event()

        def sender_thread():
            frame_id = 0
            while not stop_event.is_set():
                frame_id += 1
                img, dist_mm = create_synthetic_scene(frame_id)
                success, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not success:
                    continue

                jpeg_bytes = buf.tobytes()
                total_bytes = len(jpeg_bytes)
                total_packets = math.ceil(total_bytes / PACKET_PAYLOAD_SIZE)

                for packet_idx in range(total_packets):
                    start_b = packet_idx * PACKET_PAYLOAD_SIZE
                    end_b = min(start_b + PACKET_PAYLOAD_SIZE, total_bytes)
                    payload = jpeg_bytes[start_b:end_b]
                    header = struct.pack(HEADER_FORMAT, frame_id, packet_idx, total_packets, dist_mm)
                    try:
                        sock.sendto(header + payload, target)
                    except OSError:
                        return

                time.sleep(0.06)  # ~16 Hz realistic stream

        th = threading.Thread(target=sender_thread, daemon=True)
        th.start()

        try:
            # Ingest and process frames in SLAM pipeline concurrently
            processed_frames = 0
            start_wait = time.time()

            while processed_frames < 6 and (time.time() - start_wait) < 5.0:
                frame_data = receiver.get_latest_frame(timeout=0.15)
                if frame_data is None:
                    continue

                processed_frames += 1
                undist = camera.undistort(frame_data.image)
                gray = camera.to_grayscale(undist)

                # Sensor fusion
                central_depths = sparse_map.get_central_cone_depths(vo.T_c_w)
                fusion_state = fusion.update(
                    vo.last_relative_t, frame_data.distance_mm, frame_data.timestamp, central_depths
                )

                prev_T_c_w = vo.T_c_w.copy()
                success, curr_T_w_c, curr_T_c_w = vo.track(
                    gray, metric_scale=fusion_state.estimated_metric_scale
                )

                if success and vo.last_matched_pts_prev is not None and vo.last_matched_pts_curr is not None:
                    sparse_map.triangulate(
                        prev_T_c_w, curr_T_c_w,
                        vo.last_matched_pts_prev, vo.last_matched_pts_curr,
                        curr_bgr=undist
                    )

            self.assertGreaterEqual(processed_frames, 5, "Should process multiple frames from UDP")
            self.assertGreater(len(vo.trajectory), 1, "Camera trajectory should record motion")
            print(f"\n[TestEndToEnd] Processed {processed_frames} frames successfully.")
            print(f"[TestEndToEnd] Final camera position: {vo.T_w_c[:3, 3]}")
            print(f"[TestEndToEnd] Sparse 3D map points: {sparse_map.point_count}")

        finally:
            stop_event.set()
            th.join(timeout=1.0)
            receiver.stop()
            sock.close()


if __name__ == "__main__":
    unittest.main()
