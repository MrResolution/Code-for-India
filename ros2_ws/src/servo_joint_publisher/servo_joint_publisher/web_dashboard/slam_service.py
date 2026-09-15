"""SLAM Service Bridge for WebGL 3D Robotics Dashboard.
--------------------------------------------------------
Wraps the ESP32-CAM + VL53L1X Visual SLAM and 3D Mapping pipeline
from lidr_slam/esp32_visual_slam into a headless, thread-safe service.

Features:
- Decoupled background processing thread for UDP frame ingestion, VO, ToF fusion, and 3D triangulation.
- Live JPEG encoders for camera HUD stream and 2D top-down map plot.
- Downsampled 3D point cloud, trajectory, and 6-DOF camera pose export for Three.js WebGL.
- Built-in Mock UDP Streamer thread for immediate browser testing without hardware.
"""

import os
import sys
import time
import math
import socket
import struct
import subprocess
import threading
import urllib.request
from typing import Optional, Dict, Any, Tuple, List
import cv2
import numpy as np


def get_local_ip() -> str:
    """Discovers the primary local LAN IP address of this machine."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.2)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "10.35.234.59"


# Prevent OpenCV from hijacking Qt's platform plugin search path
if "QT_QPA_PLATFORM_PLUGIN_PATH" in os.environ and "cv2" in os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]:
    del os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]

# Locate and import lidr_slam modules
current_dir = os.path.dirname(os.path.abspath(__file__))
candidate_paths = [
    os.path.abspath(os.path.join(current_dir, "../../../../../lidr_slam/esp32_visual_slam")),
    "/home/sabo/Documents/learn_/Hardware/lidr_slam/esp32_visual_slam"
]

lidr_slam_path = None
for p in candidate_paths:
    if os.path.exists(p):
        lidr_slam_path = p
        if p not in sys.path:
            sys.path.insert(0, p)
        break

if lidr_slam_path is None:
    raise RuntimeError("Could not find lidr_slam/esp32_visual_slam directory.")

from config.camera_config import SLAMConfig
from udp_receiver import UDPReceiver, ReassembledFrame
from camera import Camera
from visual_odometry import VisualOdometry
from sensor_fusion import SingleRayToFFusion
from mapping import SparseMap
from visualization import OpenCVHUD


class HTTPMJPEGReceiver:
    """Streams MJPEG frames from ESP32-CAM via HTTP URL or IP."""

    def __init__(self, target: str):
        self.target = target.strip()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._latest_frame: Optional[ReassembledFrame] = None
        self._frame_condition = threading.Condition()
        self._fps = 0.0
        self._fps_counter = 0
        self._last_fps_time = time.time()
        self.connected = False
        self.active_url: Optional[str] = None
        self.error_msg: Optional[str] = None

    def _get_candidate_urls(self) -> List[str]:
        if self.target.startswith("http://") or self.target.startswith("https://"):
            return [self.target]
        ip = self.target
        if ":" in ip:
            return [f"http://{ip}/stream", f"http://{ip}/", f"http://{ip}"]
        return [
            f"http://{ip}:81/stream",
            f"http://{ip}/stream",
            f"http://{ip}:8080/stream",
            f"http://{ip}:80/stream",
            f"http://{ip}:81/",
            f"http://{ip}/"
        ]

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._stream_loop, name="HTTPMJPEGReceiverThread", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.connected = False

    def _stream_loop(self) -> None:
        frame_id = 0
        candidate_urls = self._get_candidate_urls()

        while self._running:
            response = None
            for url in candidate_urls:
                if not self._running:
                    return
                try:
                    req = urllib.request.Request(
                        url,
                        headers={"User-Agent": "RoboticsDashboard/1.0", "Accept": "*/*"}
                    )
                    resp = urllib.request.urlopen(req, timeout=0.8)
                    if resp.status == 200:
                        response = resp
                        self.active_url = url
                        self.connected = True
                        self.error_msg = None
                        print(f"[HTTPMJPEGReceiver] Connected to camera stream: {url}")
                        break
                except Exception as e:
                    self.error_msg = str(e)
                    continue

            if response is None:
                self.connected = False
                time.sleep(1.0)
                continue

            stream_bytes = bytearray()
            try:
                while self._running:
                    chunk = response.read(4096)
                    if not chunk:
                        break
                    stream_bytes.extend(chunk)

                    if len(stream_bytes) > 1024 * 1024:
                        stream_bytes = stream_bytes[-65536:]

                    a = stream_bytes.find(b'\xff\xd8')
                    b = stream_bytes.find(b'\xff\xd9')
                    if a != -1 and b != -1 and b > a:
                        jpg = bytes(stream_bytes[a:b+2])
                        stream_bytes = stream_bytes[b+2:]

                        img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            frame_id += 1
                            now = time.time()
                            self._fps_counter += 1
                            if now - self._last_fps_time >= 1.0:
                                self._fps = self._fps_counter / (now - self._last_fps_time)
                                self._fps_counter = 0
                                self._last_fps_time = now

                            rf = ReassembledFrame(
                                frame_id=frame_id,
                                image=img,
                                distance_mm=0,
                                timestamp=now,
                                packet_count=1
                            )
                            with self._frame_condition:
                                self._latest_frame = rf
                                self._frame_condition.notify_all()
            except Exception as e:
                self.connected = False
                self.error_msg = str(e)
                try:
                    response.close()
                except Exception:
                    pass
                time.sleep(0.5)

    def get_latest_frame(self, timeout: float = 0.05) -> Optional[ReassembledFrame]:
        with self._frame_condition:
            if self._latest_frame is None and self._running:
                self._frame_condition.wait(timeout=timeout)
            frame = self._latest_frame
            self._latest_frame = None
            return frame

    @property
    def fps(self) -> float:
        return self._fps


class SerialReceiver:
    """Reads JPEG frames directly from an ESP32-CAM connected via Serial."""

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._latest_frame: Optional[ReassembledFrame] = None
        self._frame_condition = threading.Condition()
        self._fps = 0.0
        self._fps_counter = 0
        self._last_fps_time = time.time()
        self.connected = False
        self.error_msg: Optional[str] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, name="SerialReceiverThread", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.connected = False

    def _read_loop(self) -> None:
        try:
            import serial
        except ImportError:
            print("[SerialReceiver] pyserial not installed.")
            return

        frame_id = 0
        while self._running:
            s = None
            try:
                s = serial.Serial()
                s.port = self.port
                s.baudrate = self.baudrate
                s.dtr = False
                s.rts = False
                s.timeout = 1.0
                s.open()
                self.connected = True
                self.error_msg = None
                print(f"[SerialReceiver] Connected to serial port {self.port}")

                buf = bytearray()
                while self._running:
                    chunk = s.read(1024)
                    if chunk:
                        buf.extend(chunk)
                        if len(buf) > 1024 * 1024:
                            buf = buf[-65536:]
                        a = buf.find(b'\xff\xd8')
                        b = buf.find(b'\xff\xd9')
                        if a != -1 and b != -1 and b > a:
                            jpg = bytes(buf[a:b+2])
                            buf = buf[b+2:]
                            img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if img is not None:
                                frame_id += 1
                                now = time.time()
                                self._fps_counter += 1
                                if now - self._last_fps_time >= 1.0:
                                    self._fps = self._fps_counter / (now - self._last_fps_time)
                                    self._fps_counter = 0
                                    self._last_fps_time = now
                                rf = ReassembledFrame(
                                    frame_id=frame_id,
                                    image=img,
                                    distance_mm=0,
                                    timestamp=now,
                                    packet_count=1
                                )
                                with self._frame_condition:
                                    self._latest_frame = rf
                                    self._frame_condition.notify_all()
                    else:
                        time.sleep(0.01)
                s.close()
            except Exception as e:
                self.connected = False
                self.error_msg = str(e)
                if s and s.is_open:
                    try:
                        s.close()
                    except Exception:
                        pass
                time.sleep(1.0)

    def get_latest_frame(self, timeout: float = 0.05) -> Optional[ReassembledFrame]:
        with self._frame_condition:
            if self._latest_frame is None and self._running:
                self._frame_condition.wait(timeout=timeout)
            frame = self._latest_frame
            self._latest_frame = None
            return frame

    @property
    def fps(self) -> float:
        return self._fps


class SLAMManager:
    """Headless manager for the ESP32-CAM Visual SLAM and ToF mapping pipeline."""

    def __init__(self, udp_port: int = 5000):
        self.udp_port = udp_port
        self.config = SLAMConfig(udp_port=udp_port)
        self.camera = Camera(self.config.camera)
        self.receiver = UDPReceiver(self.config)
        self.vo = VisualOdometry(self.config)
        self.fusion = SingleRayToFFusion(self.config)
        self.sparse_map = SparseMap(self.config)
        self.hud = OpenCVHUD(self.config, enable_windows=False)

        self._lock = threading.Lock()
        self.running = False
        self.worker_thread: Optional[threading.Thread] = None

        # Network and host discovery
        self.local_ip: str = get_local_ip()

        # Multi-source streaming targets
        self.current_target: str = "10.88.106.30"
        self.source_mode: str = "auto"
        self.http_receiver: Optional[HTTPMJPEGReceiver] = None
        self.serial_receiver: Optional[SerialReceiver] = None

        # Performance & FPS trackers
        self.proc_fps = 0.0
        self.web_fps = 0.0
        self.last_frame_time = 0.0

        # Cached JPEG frames
        self._latest_hud_jpeg: Optional[bytes] = None
        self._latest_map2d_jpeg: Optional[bytes] = None
        self._placeholder_hud_jpeg: bytes = self._generate_placeholder_hud()
        self._placeholder_map2d_jpeg: bytes = self._generate_placeholder_map2d()

        # Telemetry snapshot
        self._state: Dict[str, Any] = {
            "active": False,
            "target": self.current_target,
            "connected": False,
            "source_mode": "Auto",
            "local_ip": self.local_ip,
            "udp_port": self.udp_port,
            "udp_fps": 0.0,
            "vo_fps": 0.0,
            "distance_mm": 0,
            "distance_m": 0.0,
            "is_tof_valid": False,
            "num_features": 0,
            "num_matches": 0,
            "curr_pos": [0.0, 0.0, 0.0],
            "metric_scale": 0.05,
            "point_count": 0,
            "heading_deg": 0.0,
            "status": "Standby",
            "diag_reason": "Awaiting stream connection...",
            "is_error": False
        }

        # 3D WebGL Map snapshot
        self._map_data: Dict[str, Any] = {
            "points": [],
            "colors": [],
            "trajectory": [[0.0, 0.0, 0.0]],
            "pose": np.eye(4).tolist(),
            "tof_ray": {"valid": False, "length": 0.0}
        }

    def _generate_placeholder_hud(self, status_override: Optional[str] = None, diag_override: Optional[str] = None, is_error: bool = False) -> bytes:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = (18, 22, 28)

        # 1. Header Banner
        cv2.rectangle(img, (0, 0), (640, 48), (28, 36, 48), -1)
        cv2.line(img, (0, 48), (640, 48), (45, 60, 80), 1)
        cv2.putText(img, "ESP32-CAM Visual SLAM & ToF", (16, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 215, 255), 2, cv2.LINE_AA)

        local_ip = getattr(self, "local_ip", "10.35.234.59")
        cv2.putText(img, f"Laptop Ingest: {local_ip}:{self.udp_port}", (340, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (160, 200, 240), 1, cv2.LINE_AA)

        # 2. Connection Status Card
        status_text = status_override or (self._state.get("status", "Listening") if hasattr(self, "_state") else "Listening")
        diag_text = diag_override or (self._state.get("diag_reason", "Awaiting video stream packets...") if hasattr(self, "_state") else "Awaiting video stream packets...")

        box_bg = (35, 20, 45) if is_error else (20, 42, 32)
        box_border = (70, 50, 220) if is_error else (40, 180, 100)
        status_color = (130, 130, 255) if is_error else (90, 240, 150)

        cv2.rectangle(img, (24, 62), (616, 142), box_bg, -1)
        cv2.rectangle(img, (24, 62), (616, 142), box_border, 1)

        cv2.putText(img, "DIAGNOSTIC STATUS:", (36, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 195, 210), 1, cv2.LINE_AA)
        cv2.putText(img, status_text[:48], (36, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, status_color, 2, cv2.LINE_AA)
        cv2.putText(img, diag_text[:70], (36, 132),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 210, 225), 1, cv2.LINE_AA)

        # 3. Stream Ingest Info
        packets = getattr(self.receiver, "_total_packets_received", 0) if hasattr(self, "receiver") else 0
        cv2.putText(img, f"Target: {self.current_target}   |   Mode: {self.source_mode.upper()}   |   Packets: {packets}",
                    (24, 168), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 220, 240), 1, cv2.LINE_AA)

        # 4. Diagnostic & Quick Setup Checklist Card
        cv2.rectangle(img, (24, 185), (616, 455), (22, 28, 38), -1)
        cv2.rectangle(img, (24, 185), (616, 455), (45, 55, 75), 1)

        cv2.putText(img, "SETUP & TROUBLESHOOTING CHECKLIST:", (36, 212),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 185, 60), 1, cv2.LINE_AA)

        cv2.putText(img, "1. Power Supply:", (36, 244),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (56, 189, 248), 1, cv2.LINE_AA)
        cv2.putText(img, "   Ensure ESP32-CAM is powered ON via USB or 5V power supply.", (36, 264),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.39, (190, 200, 215), 1, cv2.LINE_AA)

        cv2.putText(img, "2. Wi-Fi Network:", (36, 294),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (56, 189, 248), 1, cv2.LINE_AA)
        cv2.putText(img, "   ESP32 must be connected to the SAME Wi-Fi network ('SEC_LAB.').", (36, 314),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.39, (190, 200, 215), 1, cv2.LINE_AA)

        cv2.putText(img, "3. UDP Stream Destination IP:", (36, 344),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (56, 189, 248), 1, cv2.LINE_AA)
        cv2.putText(img, f"   ESP32 UDP client must stream to laptop: {local_ip}:{self.udp_port}", (36, 364),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (100, 240, 180), 1, cv2.LINE_AA)

        cv2.putText(img, "4. Serial Fallback / Direct USB:", (36, 394),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (56, 189, 248), 1, cv2.LINE_AA)
        cv2.putText(img, "   Connect ESP32 via USB and click '🔌 USB Serial' to stream via /dev/ttyUSB0.", (36, 414),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.39, (190, 200, 215), 1, cv2.LINE_AA)
        cv2.putText(img, "   Click '🔍 Auto-Detect' to scan and connect automatically.", (36, 434),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.39, (190, 200, 215), 1, cv2.LINE_AA)

        _, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        return buf.tobytes()

    def _generate_placeholder_map2d(self) -> bytes:
        img = np.zeros((520, 520, 3), dtype=np.uint8)
        img[:] = (22, 22, 28)
        for i in range(0, 520, 30):
            cv2.line(img, (i, 0), (i, 520), (35, 35, 45), 1)
            cv2.line(img, (0, i), (520, i), (35, 35, 45), 1)
        cv2.putText(img, "2D Top-Down SLAM Map", (130, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(img, "Awaiting Visual Motion...", (160, 275),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 170), 1, cv2.LINE_AA)
        _, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        return buf.tobytes()

    def start(self) -> None:
        """Starts the SLAM processing loop and receivers."""
        if self.running:
            return
        self.running = True
        self.receiver.start()
        if self.current_target:
            self._init_source(self.current_target)
        self.worker_thread = threading.Thread(target=self._worker_loop, name="SLAMWorker", daemon=True)
        self.worker_thread.start()
        with self._lock:
            self._state["active"] = True
            self._state["status"] = "Listening"
            self._state["target"] = self.current_target
            self._state["local_ip"] = self.local_ip
        print(f"[SLAMManager] Started SLAM service on UDP port {self.udp_port} with target {self.current_target}")

    def stop(self) -> None:
        """Stops the SLAM processing loop and all receivers."""
        self.running = False
        self.receiver.stop()
        if self.http_receiver:
            self.http_receiver.stop()
            self.http_receiver = None
        if self.serial_receiver:
            self.serial_receiver.stop()
            self.serial_receiver = None
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        with self._lock:
            self._state["active"] = False
            self._state["status"] = "Stopped"
        print("[SLAMManager] Stopped SLAM service")

    def connect_to(self, target: str) -> Dict[str, Any]:
        """Dynamically connects to a new camera IP, HTTP stream URL, or Serial port."""
        target = target.strip()
        if not target:
            target = "10.88.106.30"
        self.current_target = target
        mode = self._init_source(target)

        if not self.running:
            self.start()

        with self._lock:
            self._placeholder_hud_jpeg = self._generate_placeholder_hud()
            self._state["target"] = self.current_target
            self._state["source_mode"] = mode
            self._state["local_ip"] = self.local_ip
            self._state["status"] = f"Connecting ({self.current_target})"
            self._state["diag_reason"] = f"Attempting connection via {mode.upper()}..."

        print(f"[SLAMManager] Set streaming target to {self.current_target} (mode: {mode})")
        return {"status": "ok", "target": self.current_target, "mode": mode, "local_ip": self.local_ip}

    def _send_udp_handshake(self, target_ip: str) -> None:
        """Sends UDP initiation packets to target to trigger stream if needed."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(0.2)
            for port in [5000, 8888, 8080]:
                for payload in [b"START\n", b"HELLO\n", b"STREAM\n", b"\x01"]:
                    try:
                        sock.sendto(payload, (target_ip, port))
                    except Exception:
                        pass
            sock.close()
        except Exception:
            pass

    def _init_source(self, target: str) -> str:
        """Initializes or switches the secondary receiver for HTTP or Serial streams."""
        if self.http_receiver:
            self.http_receiver.stop()
            self.http_receiver = None
        if self.serial_receiver:
            self.serial_receiver.stop()
            self.serial_receiver = None

        mode = "auto"
        # 1. Handle auto / detect keyword
        if target.lower() in ("auto", "detect", "scan"):
            for p in ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]:
                if os.path.exists(p):
                    self.current_target = p
                    self.serial_receiver = SerialReceiver(port=p)
                    self.serial_receiver.start()
                    self.source_mode = "serial"
                    return "serial"
            self.current_target = "10.88.106.30"
            target = self.current_target

        # 2. Serial port
        if target.startswith("/dev/") or target.startswith("COM"):
            self.serial_receiver = SerialReceiver(port=target)
            self.serial_receiver.start()
            mode = "serial"
        # 3. Explicit HTTP stream
        elif target.startswith("http://") or target.startswith("https://"):
            self.http_receiver = HTTPMJPEGReceiver(target=target)
            self.http_receiver.start()
            mode = "http"
        # 4. IP Target (ESP32 UDP client + HTTP fallback)
        else:
            mode = "udp"
            self._send_udp_handshake(target)
            self.http_receiver = HTTPMJPEGReceiver(target=target)
            self.http_receiver.start()

        self.source_mode = mode
        return mode

    def disconnect(self) -> Dict[str, Any]:
        """Disconnects secondary stream receivers."""
        if self.http_receiver:
            self.http_receiver.stop()
            self.http_receiver = None
        if self.serial_receiver:
            self.serial_receiver.stop()
            self.serial_receiver = None
        with self._lock:
            self._state["status"] = "Disconnected"
            self._state["connected"] = False
        return {"status": "ok"}

    def reset_map(self) -> None:
        """Clears 3D point cloud, visual trajectory, and resets pose to origin."""
        with self._lock:
            self.sparse_map = SparseMap(self.config)
            self.vo = VisualOdometry(self.config)
            self.fusion = SingleRayToFFusion(self.config)
            self._map_data = {
                "points": [],
                "colors": [],
                "trajectory": [[0.0, 0.0, 0.0]],
                "pose": np.eye(4).tolist(),
                "tof_ray": {"valid": False, "length": 0.0}
            }
            self._latest_map2d_jpeg = None
            self._state["num_features"] = 0
            self._state["num_matches"] = 0
            self._state["point_count"] = 0
            self._state["curr_pos"] = [0.0, 0.0, 0.0]
        print("[SLAMManager] Reset SLAM map and camera pose to origin")

    def _run_idle_diagnostics(self) -> None:
        """Runs periodic health and network diagnostics when no video frames are received."""
        target = self.current_target
        status = "Awaiting camera stream..."
        is_error = False
        diag_reason = ""

        # Case A: Serial Target
        if target.startswith("/dev/") or target.startswith("COM"):
            if not os.path.exists(target):
                status = f"⚠️ Serial Port {target} Disconnected"
                diag_reason = f"Device {target} not found. Plug in USB cable."
                is_error = True
            elif self.serial_receiver and not self.serial_receiver.connected:
                status = f"Connecting to {target}..."
                diag_reason = f"Opening {target} at 115200 baud..."
            else:
                status = f"Listening on {target}"
                diag_reason = "Serial port open, waiting for JPEG header (0xFF, 0xD8)..."

        # Case B: IP Target / UDP / HTTP
        else:
            is_reachable = False
            try:
                res = subprocess.run(
                    ["ping", "-c", "1", "-W", "1", target],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                is_reachable = (res.returncode == 0)
            except Exception:
                is_reachable = False

            if not is_reachable:
                status = f"⚠️ Host Unreachable ({target})"
                diag_reason = f"No ping response from {target}. Check ESP32 power & Wi-Fi."
                is_error = True
            else:
                packets = self.receiver._total_packets_received
                status = f"Listening on UDP :{self.udp_port} ({target} Online)"
                diag_reason = f"Host {target} is online. Awaiting UDP packets to {self.local_ip}:{self.udp_port}."
                self._send_udp_handshake(target)

        with self._lock:
            self._state["status"] = status
            self._state["diag_reason"] = diag_reason
            self._state["is_error"] = is_error
            self._state["local_ip"] = self.local_ip
            self._placeholder_hud_jpeg = self._generate_placeholder_hud(
                status_override=status, diag_override=diag_reason, is_error=is_error
            )

    def _worker_loop(self) -> None:
        """Independent SLAM ingestion and computation thread."""
        proc_counter = 0
        proc_timer = time.time()
        last_diag_time = 0.0

        while self.running:
            # 1. Fetch newest frame from HTTP, Serial, or UDP receiver queue
            frame_data = None
            if self.http_receiver and self.http_receiver.connected:
                frame_data = self.http_receiver.get_latest_frame(timeout=0.02)
            if frame_data is None and self.serial_receiver and self.serial_receiver.connected:
                frame_data = self.serial_receiver.get_latest_frame(timeout=0.02)
            if frame_data is None:
                frame_data = self.receiver.get_latest_frame(timeout=0.02)

            now = time.time()
            if frame_data is None:
                if now - last_diag_time >= 1.0:
                    last_diag_time = now
                    self._run_idle_diagnostics()
                time.sleep(0.01)
                continue

            raw_bgr = frame_data.image
            distance_mm = frame_data.distance_mm
            timestamp = frame_data.timestamp

            # 2. Dynamic resolution adaptation
            h_raw, w_raw = raw_bgr.shape[:2]
            if self.camera.adapt_resolution(w_raw, h_raw):
                self.vo.update_camera_matrix(self.camera.K)
                self.sparse_map.update_camera_matrix(self.camera.K)

            # 3. Camera Undistortion & Preprocessing
            undistorted_bgr = self.camera.undistort(raw_bgr)
            gray_frame = self.camera.to_grayscale(undistorted_bgr, enhance=True)

            # 4. Sensor Fusion: Central ToF ray with visual landmark depths
            central_depths = self.sparse_map.get_central_cone_depths(
                self.vo.T_c_w, radius_px=self.config.tof_roi_radius_pixels
            )
            fusion_state = self.fusion.update(
                t_rel_unit=self.vo.last_relative_t,
                raw_tof_mm=distance_mm,
                timestamp=timestamp,
                central_feature_depths=central_depths
            )

            # 5. Monocular Visual Odometry Tracking
            prev_T_c_w = self.vo.T_c_w.copy()
            tracking_success, curr_T_w_c, curr_T_c_w = self.vo.track(
                gray_frame,
                metric_scale=fusion_state.estimated_metric_scale
            )

            # 6. Sparse 3D Triangulation
            if tracking_success and self.vo.last_matched_pts_prev is not None and self.vo.last_matched_pts_curr is not None:
                self.sparse_map.triangulate(
                    T_c_w_prev=prev_T_c_w,
                    T_c_w_curr=curr_T_c_w,
                    pts_prev=self.vo.last_matched_pts_prev,
                    pts_curr=self.vo.last_matched_pts_curr,
                    curr_bgr=undistorted_bgr
                )

            # 7. Processing FPS Calculation
            proc_counter += 1
            now = time.time()
            if now - proc_timer >= 1.0:
                self.proc_fps = proc_counter / (now - proc_timer)
                proc_counter = 0
                proc_timer = now

            # 8. Render Telemetry HUD frame and encode to JPEG
            curr_pos = curr_T_w_c[:3, 3].copy()
            num_kps = len(self.vo.prev_kps) if self.vo.prev_kps is not None else 0
            num_inliers = len(self.vo.last_inlier_matches)

            # Calculate active ingest rate
            if self.http_receiver and self.http_receiver.connected and self.http_receiver.fps > 0:
                current_ingest_fps = self.http_receiver.fps
                current_source_name = "HTTP"
            elif self.serial_receiver and self.serial_receiver.connected and self.serial_receiver.fps > 0:
                current_ingest_fps = self.serial_receiver.fps
                current_source_name = "Serial"
            elif self.receiver.udp_fps > 0:
                current_ingest_fps = self.receiver.udp_fps
                current_source_name = "UDP"
            else:
                current_ingest_fps = max(self.receiver.udp_fps, getattr(self.http_receiver, 'fps', 0.0))
                current_source_name = self.source_mode.upper() if self.source_mode != "auto" else "Listening"

            hud_frame = self.hud.draw_hud(
                frame_bgr=undistorted_bgr,
                udp_fps=current_ingest_fps,
                vo_fps=self.proc_fps,
                distance_mm=distance_mm,
                is_tof_valid=fusion_state.is_valid_reading,
                num_features=num_kps,
                num_matches=num_inliers,
                curr_pos=curr_pos,
                metric_scale=fusion_state.estimated_metric_scale,
                display_fps=self.proc_fps
            )
            _, hud_jpg = cv2.imencode(".jpg", hud_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])

            # 9. Render 2D Top-Down SLAM Map and encode to JPEG
            map_pts, map_cols = self.sparse_map.get_points_and_colors()
            map2d_frame = self.hud.draw_topdown_map(
                points=map_pts,
                trajectory=self.vo.trajectory,
                current_T_w_c=curr_T_w_c,
                distance_mm=distance_mm,
                is_tof_valid=fusion_state.is_valid_reading
            )
            _, map2d_jpg = cv2.imencode(".jpg", map2d_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])

            # 10. Extract yaw/heading angle from current rotation matrix R_w_c
            R = curr_T_w_c[:3, :3]
            heading_rad = math.atan2(R[0, 2], R[2, 2])
            heading_deg = round(math.degrees(heading_rad), 1)

            # 11. Downsample 3D point cloud for snappy WebGL network transfer (max 2000 points)
            total_pts = len(map_pts)
            if total_pts > 2000:
                indices = np.random.choice(total_pts, size=2000, replace=False)
                send_pts = map_pts[indices].tolist()
                send_cols = map_cols[indices].tolist()
            else:
                send_pts = map_pts.tolist()
                send_cols = map_cols.tolist()

            send_traj = [p.tolist() for p in self.vo.trajectory[-300:]]

            # 12. Update thread-safe snapshots
            with self._lock:
                self._latest_hud_jpeg = hud_jpg.tobytes()
                self._latest_map2d_jpeg = map2d_jpg.tobytes()

                is_connected = (current_ingest_fps > 0.1) or (self.http_receiver and self.http_receiver.connected) or (self.serial_receiver and self.serial_receiver.connected)
                self._state = {
                    "active": self.running,
                    "target": self.current_target,
                    "connected": is_connected,
                    "source_mode": current_source_name,
                    "local_ip": self.local_ip,
                    "udp_port": self.udp_port,
                    "udp_fps": round(current_ingest_fps, 1),
                    "vo_fps": round(self.proc_fps, 1),
                    "distance_mm": distance_mm,
                    "distance_m": round(distance_mm / 1000.0, 2),
                    "is_tof_valid": fusion_state.is_valid_reading,
                    "num_features": num_kps,
                    "num_matches": num_inliers,
                    "curr_pos": [round(float(curr_pos[0]), 3), round(float(curr_pos[1]), 3), round(float(curr_pos[2]), 3)],
                    "metric_scale": round(float(fusion_state.estimated_metric_scale), 3),
                    "point_count": self.sparse_map.point_count,
                    "heading_deg": heading_deg,
                    "status": "Tracking" if tracking_success else ("Connected" if is_connected else "Listening"),
                    "diag_reason": "Live stream active",
                    "is_error": False
                }

                self._map_data = {
                    "points": send_pts,
                    "colors": send_cols,
                    "trajectory": send_traj,
                    "pose": curr_T_w_c.tolist(),
                    "tof_ray": {
                        "valid": fusion_state.is_valid_reading,
                        "length": float(distance_mm / 1000.0) if distance_mm > 0 else 0.0
                    }
                }

    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._state)

    def get_map_data(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._map_data)

    def get_latest_hud_jpeg(self) -> bytes:
        with self._lock:
            if self._latest_hud_jpeg is not None:
                return self._latest_hud_jpeg
            return self._placeholder_hud_jpeg

    def get_latest_map2d_jpeg(self) -> bytes:
        with self._lock:
            if self._latest_map2d_jpeg is not None:
                return self._latest_map2d_jpeg
            return self._placeholder_map2d_jpeg
