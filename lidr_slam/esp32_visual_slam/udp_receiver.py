"""Robust UDP receiver and raw-grayscale frame reassembly module for ESP32-CAM + VL53L1X.

Protocol Specification (ESP32-CAM Grayscale UDP Firmware):
- 14-byte binary header per packet (little-endian, packed):
    uint32_t frameID       (4 bytes)  — monotonically increasing frame counter
    uint16_t packetIndex   (2 bytes)  — 0-based index of this packet within the frame
    uint16_t totalPackets  (2 bytes)  — total packets required to reconstruct this frame
    uint16_t distanceMM    (2 bytes)  — VL53L1X ToF distance in millimetres (0 = invalid)
    uint16_t width         (2 bytes)  — frame width in pixels (160 for QQVGA)
    uint16_t height        (2 bytes)  — frame height in pixels (120 for QQVGA)
- Python struct format: "<IHHHHH"
- Payload: raw 8-bit grayscale pixel bytes (no JPEG encoding).
- Full frame: width × height bytes reassembled in packet-index order.
"""

import socket
import struct
import time
import threading
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple
import cv2
import numpy as np

from config.camera_config import SLAMConfig


@dataclass
class ReassembledFrame:
    """Represents a fully reassembled and decoded camera frame with sensor data."""
    frame_id: int
    image: np.ndarray          # Decoded BGR image
    distance_mm: int           # VL53L1X distance in millimeters
    timestamp: float           # Local receive timestamp
    packet_count: int          # Total packets reassembled


class InFlightFrame:
    """Accumulates incoming UDP packets for a specific frame."""
    def __init__(self, frame_id: int, total_packets: int, distance_mm: int,
                 width: int, height: int):
        self.frame_id = frame_id
        self.total_packets = total_packets
        self.distance_mm = distance_mm
        self.width = width
        self.height = height
        self.packets: Dict[int, bytes] = {}
        self.first_received_time = time.time()
        self.last_received_time = self.first_received_time

    def add_packet(self, packet_index: int, payload: bytes, distance_mm: int) -> bool:
        """Add a packet payload. Updates distance to the freshest received packet."""
        if packet_index not in self.packets:
            self.packets[packet_index] = payload
            self.distance_mm = distance_mm  # Keep latest ToF reading
            self.last_received_time = time.time()
        return len(self.packets) == self.total_packets

    def assemble_bytes(self) -> Optional[bytes]:
        """Assemble byte slices in index order if all packets are present."""
        if len(self.packets) != self.total_packets:
            return None
        return b"".join(self.packets[i] for i in range(self.total_packets))



class UDPReceiver:
    """High-performance non-blocking UDP receiver for ESP32-CAM streams."""

    def __init__(self, config: Optional[SLAMConfig] = None):
        self.config = config or SLAMConfig()
        self.header_format = self.config.header_format
        self.header_size = self.config.header_size
        self.timeout_sec = self.config.frame_assembly_timeout_sec
        self.max_active_frames = self.config.max_active_frames

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._socket: Optional[socket.socket] = None

        # Active in-flight frames: frame_id -> InFlightFrame
        self._active_frames: Dict[int, InFlightFrame] = {}
        self._lock = threading.Lock()

        # Freshest frame container (guarantees zero queue lag)
        self._latest_frame: Optional[ReassembledFrame] = None
        self._frame_condition = threading.Condition()

        # Performance & telemetry statistics
        self._total_packets_received = 0
        self._total_frames_completed = 0
        self._total_frames_dropped = 0
        self._current_udp_fps = 0.0
        self._last_fps_calc_time = time.time()
        self._fps_frame_counter = 0

    def start(self) -> None:
        """Initialize the UDP socket and start the background listener thread."""
        if self._running:
            return

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.config.udp_buffer_size)
        self._socket.settimeout(0.5)  # Periodic timeout to allow clean thread shutdown
        self._socket.bind((self.config.udp_ip, self.config.udp_port))

        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, name="UDPReceiverThread", daemon=True)
        self._thread.start()
        print(f"[UDPReceiver] Listening on {self.config.udp_ip}:{self.config.udp_port}...")

    def stop(self) -> None:
        """Stop receiver thread and close socket."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
        print("[UDPReceiver] Receiver stopped.")

    def _receive_loop(self) -> None:
        """Continuous packet receiving loop executed in a background thread."""
        while self._running:
            try:
                data, _ = self._socket.recvfrom(self.config.udp_buffer_size)
            except socket.timeout:
                self._cleanup_stale_frames()
                continue
            except Exception as e:
                if self._running:
                    print(f"[UDPReceiver] Socket read error: {e}")
                break

            if len(data) < self.header_size:
                continue

            self._total_packets_received += 1

            # Unpack 14-byte header: frameID, packetIndex, totalPackets, distanceMM, width, height
            try:
                frame_id, packet_idx, total_packets, distance_mm, width, height = struct.unpack(
                    self.header_format, data[:self.header_size]
                )
            except struct.error:
                continue

            if total_packets == 0 or packet_idx >= total_packets:
                continue

            payload = data[self.header_size:]

            completed_raw_bytes: Optional[bytes] = None
            completed_distance: int = distance_mm
            completed_frame_id: int = frame_id
            completed_packet_count: int = total_packets
            completed_width: int = width
            completed_height: int = height

            with self._lock:
                # Initialize in-flight frame if new
                if frame_id not in self._active_frames:
                    # Enforce buffer limit: remove oldest if exceeding max active
                    if len(self._active_frames) >= self.max_active_frames:
                        oldest_id = min(self._active_frames.keys(), key=lambda fid: self._active_frames[fid].first_received_time)
                        del self._active_frames[oldest_id]
                        self._total_frames_dropped += 1

                    self._active_frames[frame_id] = InFlightFrame(
                        frame_id, total_packets, distance_mm, width, height
                    )

                in_flight = self._active_frames[frame_id]
                is_complete = in_flight.add_packet(packet_idx, payload, distance_mm)

                if is_complete:
                    completed_raw_bytes    = in_flight.assemble_bytes()
                    completed_distance     = in_flight.distance_mm
                    completed_frame_id     = in_flight.frame_id
                    completed_packet_count = in_flight.total_packets
                    completed_width        = in_flight.width
                    completed_height       = in_flight.height
                    del self._active_frames[frame_id]

            # Periodic cleanup of expired frames
            self._cleanup_stale_frames()

            # If a full frame was assembled, decode raw grayscale and store freshest frame
            if completed_raw_bytes is not None:
                self._process_completed_frame(
                    completed_frame_id, completed_raw_bytes,
                    completed_distance, completed_packet_count,
                    completed_width, completed_height
                )



    def _process_completed_frame(
        self,
        frame_id: int,
        raw_bytes: bytes,
        distance_mm: int,
        packet_count: int,
        width: int,
        height: int,
    ) -> None:
        """Decode raw 8-bit grayscale bytes into an OpenCV BGR image and update freshest frame.

        The ESP32-CAM firmware sends pixels in row-major order (top-left → bottom-right),
        one byte per pixel, no header inside the payload.  We:
          1. Validate the payload is exactly width × height bytes.
          2. Reshape into a (height, width) uint8 array.
          3. Convert GRAY → BGR so the rest of the pipeline (undistort, ORB, HUD) is unchanged.
        """
        expected_size = width * height

        if len(raw_bytes) < expected_size:
            print(
                f"[UDPReceiver] Frame {frame_id}: payload too short "
                f"({len(raw_bytes)} < {expected_size} bytes). Dropping."
            )
            return

        try:
            gray = np.frombuffer(raw_bytes[:expected_size], dtype=np.uint8).reshape((height, width))
            image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            print(f"[UDPReceiver] Error decoding grayscale frame {frame_id}: {e}")
            return

        now = time.time()
        self._fps_frame_counter += 1
        elapsed = now - self._last_fps_calc_time
        if elapsed >= 1.0:
            self._current_udp_fps = self._fps_frame_counter / elapsed
            self._fps_frame_counter = 0
            self._last_fps_calc_time = now

        frame = ReassembledFrame(
            frame_id=frame_id,
            image=image,
            distance_mm=distance_mm,
            timestamp=now,
            packet_count=packet_count,
        )

        with self._frame_condition:
            self._latest_frame = frame
            self._total_frames_completed += 1
            self._frame_condition.notify()



    def _cleanup_stale_frames(self) -> None:
        """Remove incomplete frames that have exceeded the timeout window."""
        now = time.time()
        with self._lock:
            stale_ids = [
                fid for fid, in_flight in self._active_frames.items()
                if (now - in_flight.first_received_time) > self.timeout_sec
            ]
            for fid in stale_ids:
                del self._active_frames[fid]
                self._total_frames_dropped += 1

    def get_latest_frame(self, timeout: float = 0.05) -> Optional[ReassembledFrame]:
        """Fetch the most recent complete frame.
        
        Always prioritizes the newest frame and leaves no backlog.
        Returns None if no frame is available within timeout.
        """
        with self._frame_condition:
            if self._latest_frame is None:
                self._frame_condition.wait(timeout=timeout)
            frame = self._latest_frame
            self._latest_frame = None  # Consume frame
            return frame

    @property
    def udp_fps(self) -> float:
        """Current UDP frames-per-second rate."""
        return self._current_udp_fps

    @property
    def stats(self) -> dict:
        """Telemetry diagnostics dictionary."""
        return {
            "packets_received": self._total_packets_received,
            "frames_completed": self._total_frames_completed,
            "frames_dropped": self._total_frames_dropped,
            "udp_fps": round(self._current_udp_fps, 1),
            "in_flight_frames": len(self._active_frames)
        }
