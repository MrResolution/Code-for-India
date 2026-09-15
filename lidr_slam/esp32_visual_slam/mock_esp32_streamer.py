"""Mock ESP32-CAM and VL53L1X UDP streamer for local testing and validation.

Generates synthetic moving camera frames with textured geometric features,
simulates realistic VL53L1X range measurements, splits JPEG frames across
UDP packets matching the protocol (<IHHH header), and broadcasts to port 5000.
"""

import socket
import struct
import time
import math
from typing import Tuple
import cv2
import numpy as np


HEADER_FORMAT = "<IHHH"
HEADER_SIZE = 10
PACKET_PAYLOAD_SIZE = 1400  # Safe UDP payload within standard MTU (1500)


def create_synthetic_scene(t_step: int, width: int = 640, height: int = 480) -> Tuple[np.ndarray, int]:
    """Generates a textured synthetic indoor environment with camera panning and forward motion."""
    image = np.zeros((height, width, 3), dtype=np.uint8)

    # Simulated motion parameters
    # Forward motion (distance to wall decreasing then cycling)
    base_dist = 1800  # 1.8 meters
    distance_mm = int(base_dist - 400 * math.sin(t_step * 0.05))
    distance_mm = max(200, min(3500, distance_mm))

    # Grid / Checkerboard perspective wall
    shift_x = int(50 * math.sin(t_step * 0.04))
    shift_y = int(25 * math.cos(t_step * 0.04))
    zoom = 1.0 + 0.3 * math.sin(t_step * 0.05)

    # Draw textured grid
    grid_size = int(40 * zoom)
    for y in range(0, height, grid_size):
        cv2.line(image, (0, y + shift_y % grid_size), (width, y + shift_y % grid_size), (60, 60, 60), 1)
    for x in range(0, width, grid_size):
        cv2.line(image, (x + shift_x % grid_size, 0), (x + shift_x % grid_size, height), (60, 60, 60), 1)

    # Draw several textured landmarks (circles, rectangles, stars) with high ORB feature responses
    np.random.seed(42)
    for i in range(15):
        orig_x = int(100 + (i % 5) * 110)
        orig_y = int(100 + (i // 5) * 120)

        # Apply motion projection
        px = int(width / 2 + (orig_x - width / 2 + shift_x) * zoom)
        py = int(height / 2 + (orig_y - height / 2 + shift_y) * zoom)

        if 30 < px < width - 30 and 30 < py < height - 30:
            radius = int(18 * zoom)
            color = (int(50 + (i * 40) % 200), int(100 + (i * 30) % 150), int(150 + (i * 50) % 100))
            cv2.circle(image, (px, py), radius, color, -1)
            # Add high-contrast internal features for rich ORB descriptors
            cv2.circle(image, (px, py), radius // 2, (255, 255, 255), 2)
            cv2.rectangle(image, (px - 5, py - 5), (px + 5, py + 5), (0, 0, 0), -1)

    # Add text banner with timestamp
    cv2.putText(image, f"MOCK ESP32-CAM | Frame {t_step}", (20, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)

    return image, distance_mm


def stream_mock_data(target_ip: str = "127.0.0.1", port: int = 5000, fps: float = 15.0):
    """Encodes and transmits mock camera packets over UDP."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[MockStreamer] Streaming mock ESP32 packets to {target_ip}:{port} at {fps} FPS...")

    frame_id = 0
    interval = 1.0 / fps

    try:
        while True:
            t0 = time.time()
            frame_id += 1

            # 1. Generate frame & sensor reading
            bgr_image, distance_mm = create_synthetic_scene(frame_id)

            # 2. Encode to JPEG
            success, encoded_buf = cv2.imencode(".jpg", bgr_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not success:
                continue

            jpeg_bytes = encoded_buf.tobytes()
            total_bytes = len(jpeg_bytes)

            # 3. Calculate packet fragmentation
            total_packets = math.ceil(total_bytes / PACKET_PAYLOAD_SIZE)

            # 4. Transmit packets
            for packet_idx in range(total_packets):
                start_byte = packet_idx * PACKET_PAYLOAD_SIZE
                end_byte = min(start_byte + PACKET_PAYLOAD_SIZE, total_bytes)
                payload = jpeg_bytes[start_byte:end_byte]

                # Pack header: frameID (uint32), packetIndex (uint16), totalPackets (uint16), distanceMM (uint16)
                header = struct.pack(HEADER_FORMAT, frame_id, packet_idx, total_packets, distance_mm)
                packet_data = header + payload

                sock.sendto(packet_data, (target_ip, port))

            # Maintain frame rate
            elapsed = time.time() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[MockStreamer] Stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Mock ESP32 UDP Streamer")
    parser.add_argument("--ip", default="127.0.0.1", help="Target receiver IP")
    parser.add_argument("--port", type=int, default=5000, help="Target UDP port")
    parser.add_argument("--fps", type=float, default=15.0, help="Stream FPS")
    args = parser.parse_args()

    stream_mock_data(target_ip=args.ip, port=args.port, fps=args.fps)
