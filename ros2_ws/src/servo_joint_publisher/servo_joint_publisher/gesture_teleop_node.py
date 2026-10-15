#!/usr/bin/env python3
"""
ROS 2 Node: gesture_teleop_node (Low-Sensitivity Defined Posture Tracking)
---------------------------------------------------------------------------
Captures live webcam feed (/dev/video0), tracks human upper-body posture (strictly
till waist), applies heavy low-pass exponential smoothing (alpha = 0.10) and deadband filtering
to eliminate jitter, and maps defined postures to the arm's joint angles.

Sensitivity & Defined Gesture Features:
  • Low-Pass EMA Filter (alpha = 0.10) for steady, jitter-free servo motion
  • 1.5° Deadband threshold to suppress small body/hand twitches
  • Defined posture zones: Neutral Center, Left/Right Base Sweep, Shoulder Elevation, Gripper Open/Close
  • Live Cyberpunk HUD with active posture indicators and low-sensitivity status.

Output: a JointState (twin radians) on `output_topic`. The default,
/joint_states, is what servo_serial_commander turns into servo commands. The
desktop app's Gesture Mode sets /gesture_teleop/joint_targets instead and moves
its own sliders, so its limits and E-STOP stay in charge.

Pose tracking works with both MediaPipe APIs: the legacy `mp.solutions.pose`
(MediaPipe 0.10.21 and older) and the Tasks PoseLandmarker (newer releases
dropped `mp.solutions`). The Tasks API needs a model file: `model_path`, or
pose_landmarker_full.task, downloaded once into ~/.cache/code_for_india/.

The node exits with status 1 when it can't track (no OpenCV, no MediaPipe, no
model, no camera) instead of running blind.
"""

import math
import os
import shutil
import sys
import time
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import cv2
    CV2_ERROR = None
except Exception as e:  # ImportError, or a numpy ABI clash inside cv2
    cv2 = None
    CV2_ERROR = str(e)

try:
    import mediapipe as mp
    MEDIAPIPE_ERROR = None
except Exception as e:
    mp = None
    MEDIAPIPE_ERROR = str(e)

INSTALL_HINT = (f'install it for the Python that runs this node: {sys.executable} -m pip install '
                '--user --break-system-packages attrs mediapipe "numpy<2" (docs/QUICK_INSTALL_GUIDE.md)')

POSE_MODEL_URL = ('https://storage.googleapis.com/mediapipe-models/pose_landmarker/'
                  'pose_landmarker_full/float16/latest/pose_landmarker_full.task')

# Joints this node drives, in twin radians (as the dashboards publish them).
ARM_JOINTS = [
    'turntable_link_joint_dup',    # Base Turntable Yaw
    'turntable_link_joint',        # Lower Shoulder Pitch
    'turntable_link_joint_dup_1',  # Upper Forearm Pitch
    'turntable_link_joint_dup_2',  # Wrist Pitch
    'turntable_link_joint_dup_3',  # Wrist Twist / Spin (GPIO 27)
    'grip',                        # Gripper (GPIO 26)
]


class StartupError(Exception):
    """The node can't track; the message says why."""


def default_model_path():
    cache = os.environ.get('XDG_CACHE_HOME') or os.path.join(os.path.expanduser('~'), '.cache')
    return os.path.join(cache, 'code_for_india', 'pose_landmarker_full.task')


def ensure_model(model_path, logger):
    """Path to the PoseLandmarker model, downloading it once if there is none."""
    if model_path:
        if not os.path.isfile(model_path):
            raise StartupError(f"model_path {model_path} does not exist")
        return model_path
    path = default_model_path()
    if os.path.isfile(path):
        return path
    logger.info(f"⬇️ Downloading the pose model (about 9 MB, once) to {path} ...")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    part = path + '.part'
    try:
        with urllib.request.urlopen(POSE_MODEL_URL, timeout=30) as r, open(part, 'wb') as f:
            shutil.copyfileobj(r, f)
        os.replace(part, path)
    except Exception as e:
        try:
            os.remove(part)
        except OSError:
            pass
        raise StartupError(f"could not download the pose model ({e}). Download {POSE_MODEL_URL} "
                           f"to {path}, or pass -p model_path:=<file>") from e
    return path


class LegacyPose:
    """mp.solutions.pose (MediaPipe 0.10.21 and older)."""
    name = 'MediaPipe Pose (solutions API)'

    def __init__(self):
        self.tracker = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )

    def detect(self, rgb):
        results = self.tracker.process(rgb)
        return results.pose_landmarks.landmark if results.pose_landmarks else None

    def close(self):
        self.tracker.close()


class TasksPose:
    """Tasks PoseLandmarker (MediaPipe releases without mp.solutions)."""
    name = 'MediaPipe PoseLandmarker (Tasks API)'

    def __init__(self, model_path):
        from mediapipe.tasks import python as mp_tasks
        from mediapipe.tasks.python import vision
        options = vision.PoseLandmarkerOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=model_path),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.6,
            min_pose_presence_confidence=0.6,
            min_tracking_confidence=0.6
        )
        self.tracker = vision.PoseLandmarker.create_from_options(options)
        self.last_ts_ms = -1

    def detect(self, rgb):
        # VIDEO mode needs strictly increasing timestamps.
        ts_ms = max(int(time.monotonic() * 1000), self.last_ts_ms + 1)
        self.last_ts_ms = ts_ms
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self.tracker.detect_for_video(image, ts_ms)
        return result.pose_landmarks[0] if result.pose_landmarks else None

    def close(self):
        self.tracker.close()


def make_pose_tracker(model_path, logger):
    """The legacy tracker where MediaPipe still has it, else the Tasks one."""
    errors = []
    if getattr(mp, 'solutions', None) is not None:
        try:
            return LegacyPose()
        except Exception as e:
            errors.append(f"mp.solutions.pose: {e}")
    try:
        return TasksPose(ensure_model(model_path, logger))
    except StartupError:
        raise
    except Exception as e:
        errors.append(f"PoseLandmarker: {e}")
    raise StartupError(f"MediaPipe {getattr(mp, '__version__', '?')} could not start pose tracking: "
                       + '; '.join(errors))


class GestureTeleopNode(Node):

    def __init__(self):
        super().__init__('gesture_teleop_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('video_device', 0)
        self.declare_parameter('smooth_alpha', 0.10)   # Reduced from 0.25 -> 0.10 for heavy jitter suppression
        self.declare_parameter('deadband_deg', 1.5)     # Deadband threshold in degrees
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('model_path', '')        # Tasks API model; '' = cached download
        self.declare_parameter('show_window', True)
        self.declare_parameter('grip_closed_rad', 0.0)
        self.declare_parameter('grip_open_rad', 0.7)
        # Hand length (wrist → index) over forearm length: open above, closed below
        self.declare_parameter('grip_open_ratio', 0.55)
        self.declare_parameter('grip_close_ratio', 0.45)

        self.device_id = self.get_parameter('video_device').get_parameter_value().integer_value
        self.alpha = self.get_parameter('smooth_alpha').get_parameter_value().double_value
        self.deadband_rad = math.radians(self.get_parameter('deadband_deg').get_parameter_value().double_value)
        self.pub_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value or '/joint_states'
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.show_window = self.get_parameter('show_window').get_parameter_value().bool_value
        self.grip_closed_rad = self.get_parameter('grip_closed_rad').get_parameter_value().double_value
        self.grip_open_rad = self.get_parameter('grip_open_rad').get_parameter_value().double_value
        self.grip_open_ratio = self.get_parameter('grip_open_ratio').get_parameter_value().double_value
        self.grip_close_ratio = self.get_parameter('grip_close_ratio').get_parameter_value().double_value

        # ── ROS 2 Publishers ─────────────────────────────────────────────
        self.joint_pub = self.create_publisher(JointState, self.output_topic, 10)

        # ── Joint State Definitions ──────────────────────────────────────
        self.joint_names = list(ARM_JOINTS)
        self.target_angles = {j: 0.0 for j in self.joint_names}

        # Set default home pose positions
        self.target_angles['turntable_link_joint'] = 0.3    # ~17° shoulder elevation
        self.target_angles['turntable_link_joint_dup_1'] = 1.0  # ~57° elbow pitch
        self.target_angles['turntable_link_joint_dup_2'] = 0.5  # ~28° wrist pitch
        self.target_angles['grip'] = self.grip_closed_rad
        self.current_angles = dict(self.target_angles)
        self.grip_open = False

        # Start from the arm's current pose, not a jump to the defaults above:
        # take it from the first /joint_states message from someone else.
        self.started_at = time.monotonic()
        self.pose_from_arm = False
        self.init_sub = None
        if self.output_topic != '/joint_states':
            self.init_sub = self.create_subscription(JointState, '/joint_states', self.on_arm_pose, 10)

        # Upper-body joint connections strictly till waist (Landmarks 0 to 24)
        self.UPPER_BODY_CONNECTIONS = [
            (0, 1), (1, 2), (2, 3), (3, 7), (0, 4), (4, 5), (5, 6), (6, 8), (9, 10),
            (11, 12),
            (11, 13), (13, 15), (15, 17), (15, 19), (15, 21), (17, 19),
            (12, 14), (14, 16), (16, 18), (16, 20), (16, 22), (18, 20),
            (11, 23), (12, 24), (23, 24)
        ]

        # Active posture labels for HUD
        self.posture_state = "NEUTRAL"
        self.grip_ratio = 0.0

        self.cap = None
        self.pose_tracker = None
        self.timer = None
        self.last_frame_t = 0.0
        self.quit_requested = False
        self.exit_code = 0
        self.window_name = "Code for India — Upper Body Cyberpunk Gesture Teleop"
        self.window_initialized = False

    def start(self):
        """Open the camera and the pose tracker; StartupError says what's missing."""
        if cv2 is None:
            raise StartupError(f"OpenCV (cv2) is not available: {CV2_ERROR}. "
                               "Install it with: sudo apt install python3-opencv")
        if mp is None:
            raise StartupError(f"MediaPipe is not available ({MEDIAPIPE_ERROR}): {INSTALL_HINT}")

        self.pose_tracker = make_pose_tracker(self.model_path, self.get_logger())
        self.get_logger().info(f"🦴 Pose tracker: {self.pose_tracker.name}, "
                               f"MediaPipe {getattr(mp, '__version__', '?')}")

        # ── Video Capture ────────────────────────────────────────────────
        self.get_logger().info(f"🎥 Opening camera /dev/video{self.device_id} ...")
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            raise StartupError(f"could not open camera /dev/video{self.device_id}. Is it plugged in, "
                               "or held by another program (an old gesture_teleop_node: "
                               "pkill -f gesture_teleop_node)? Pick another with -p video_device:=<n>")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.last_frame_t = time.monotonic()
        self.get_logger().info("✅ Camera initialized successfully.")

        # Timer loop @ publish_rate Hz
        self.timer = self.create_timer(1.0 / self.pub_rate, self.processing_loop)
        self.get_logger().info(f"🚀 Gesture teleop ready: publishing {', '.join(self.joint_names)} "
                               f"on {self.output_topic}")

    def on_arm_pose(self, msg: JointState):
        if self.pose_from_arm:
            return
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_angles and math.isfinite(pos):
                self.current_angles[name] = pos
                self.pose_from_arm = True

    def fail(self, text):
        """Log a fatal error and stop the node with status 1."""
        self.get_logger().error(f"❌ {text}")
        self.exit_code = 1
        self.quit_requested = True

    def calculate_angle_2d(self, a, b, c):
        """Calculate angle in degrees at vertex b formed by points a-b-c."""
        ba = [a[0] - b[0], a[1] - b[1]]
        bc = [c[0] - b[0], c[1] - b[1]]

        dot = ba[0] * bc[0] + ba[1] * bc[1]
        mag_ba = math.sqrt(ba[0]**2 + ba[1]**2)
        mag_bc = math.sqrt(bc[0]**2 + bc[1]**2)

        if mag_ba * mag_bc == 0:
            return 0.0

        cos_angle = max(-1.0, min(1.0, dot / (mag_ba * mag_bc)))
        return math.degrees(math.acos(cos_angle))

    def draw_corner_brackets(self, img, bbox, color=(255, 255, 255), thickness=2, length=25):
        """Draw corner bracket box around bounding box (xmin, ymin, xmax, ymax)."""
        x1, y1, x2, y2 = bbox
        cv2.line(img, (x1, y1), (x1 + length, y1), color, thickness)
        cv2.line(img, (x1, y1), (x1, y1 + length), color, thickness)
        cv2.line(img, (x2, y1), (x2 - length, y1), color, thickness)
        cv2.line(img, (x2, y1), (x2, y1 + length), color, thickness)
        cv2.line(img, (x1, y2), (x1 + length, y2), color, thickness)
        cv2.line(img, (x1, y2), (x1, y2 - length), color, thickness)
        cv2.line(img, (x2, y2), (x2 - length, y2), color, thickness)
        cv2.line(img, (x2, y2), (x2, y2 - length), color, thickness)

    def processing_loop(self):
        """Main loop: process frame, extract upper-body pose, apply deadband & low-pass EMA filter."""
        if self.quit_requested:
            return
        ret, frame = self.cap.read()
        now = time.monotonic()
        if not ret or frame is None:
            if now - self.last_frame_t > 3.0:
                self.fail(f"camera /dev/video{self.device_id} sent no frame for 3 s")
            return
        self.last_frame_t = now

        # Wait (up to 1 s) for the arm's current pose so the first command is not a jump.
        if self.init_sub is not None and not self.pose_from_arm and now - self.started_at < 1.0:
            return
        if self.init_sub is not None:
            self.destroy_subscription(self.init_sub)
            self.init_sub = None

        # Track on the camera image, so MediaPipe's left/right are the person's;
        # draw and measure in the mirrored view the operator sees.
        landmarks = self.pose_tracker.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape

        # Desaturated high-contrast monochrome style
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hud_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        hud_frame = cv2.convertScaleAbs(hud_frame, alpha=0.85, beta=10)

        pose_detected = False
        arm_label = "STAND IN FRONT OF WEBCAM"
        rel_x, rel_y, rel_z = 0.0, 0.0, 0.0

        if landmarks:
            pose_detected = True

            # Extract upper-body landmark coordinates (strictly 0..24, stopping at waist/hips)
            lm_points = {}
            ub_x_coords, ub_y_coords = [], []

            for idx in range(25):  # 0 to 24 (waist level)
                lm = landmarks[idx]
                vis = lm.visibility if lm.visibility is not None else 1.0
                px, py = int((1.0 - lm.x) * w), int(lm.y * h)   # mirrored x
                lm_points[idx] = (px, py, lm.z, vis)
                if vis > 0.5:
                    ub_x_coords.append(px)
                    ub_y_coords.append(py)

            # Draw Corner Bracket Bounding Box
            if len(ub_x_coords) > 0 and len(ub_y_coords) > 0:
                margin = 35
                bx1 = max(10, min(ub_x_coords) - margin)
                by1 = max(10, min(ub_y_coords) - margin)
                bx2 = min(w - 10, max(ub_x_coords) + margin)
                by2 = min(h - 10, max(ub_y_coords) + margin)
                self.draw_corner_brackets(hud_frame, (bx1, by1, bx2, by2), color=(255, 255, 255), thickness=2, length=30)

            # Draw White Skeleton Lines
            for p1_idx, p2_idx in self.UPPER_BODY_CONNECTIONS:
                if p1_idx in lm_points and p2_idx in lm_points:
                    pt1 = lm_points[p1_idx]
                    pt2 = lm_points[p2_idx]
                    if pt1[3] > 0.5 and pt2[3] > 0.5:
                        cv2.line(hud_frame, (pt1[0], pt1[1]), (pt2[0], pt2[1]), (255, 255, 255), 2)

            # Draw Orange-Red Keypoint Circles
            for idx, (px, py, z_val, vis) in lm_points.items():
                if vis > 0.5:
                    cv2.circle(hud_frame, (px, py), 7, (30, 80, 255), -1)
                    cv2.circle(hud_frame, (px, py), 8, (255, 255, 255), 1)

            # Select active arm: the one with the higher wrist
            r_wrist = landmarks[16]
            l_wrist = landmarks[15]

            if r_wrist.y < l_wrist.y:
                s_pt, e_pt, w_pt, hip_pt, index_pt, pinky_pt, thumb_pt = (12, 14, 16, 24, 20, 18, 22)
                arm_label = "RIGHT ARM ACTIVE"
                is_right_arm = True
            else:
                s_pt, e_pt, w_pt, hip_pt, index_pt, pinky_pt, thumb_pt = (11, 13, 15, 23, 19, 17, 21)
                arm_label = "LEFT ARM ACTIVE"
                is_right_arm = False

            sp = (lm_points[s_pt][0], lm_points[s_pt][1])
            ep = (lm_points[e_pt][0], lm_points[e_pt][1])
            wp = (lm_points[w_pt][0], lm_points[w_pt][1])
            hp = (lm_points[hip_pt][0], lm_points[hip_pt][1])
            ip = (lm_points[index_pt][0], lm_points[index_pt][1])
            pp = (lm_points[pinky_pt][0], lm_points[pinky_pt][1])
            tp = (lm_points[thumb_pt][0], lm_points[thumb_pt][1])

            rel_x = (wp[0] - sp[0]) / 10.0
            rel_y = (wp[1] - sp[1]) / 10.0
            rel_z = lm_points[w_pt][2] * 100.0

            if min(lm_points[i][3] for i in (s_pt, e_pt, w_pt)) < 0.5:
                # Guessed landmarks: hold the last targets rather than follow them
                arm_label += " (NOT VISIBLE, HOLDING)"
            else:
                # ── Defined Gesture Posture Logic with Deadband Zones ─────────

                # 1. Base Turntable Yaw (turntable_link_joint_dup)
                # Deadband Zone: -0.12 .. +0.12 normalized sweep = Center Neutral
                dx_sweep = (wp[0] - sp[0]) / (w / 2.0)
                if abs(dx_sweep) < 0.12:
                    raw_yaw = 0.0
                    yaw_desc = "BASE CENTER"
                else:
                    raw_yaw = math.copysign((abs(dx_sweep) - 0.12) * 1.6, dx_sweep)
                    yaw_desc = "BASE SWEEP LEFT" if raw_yaw < 0 else "BASE SWEEP RIGHT"

                self.target_angles['turntable_link_joint_dup'] = max(-1.4, min(1.4, raw_yaw))

                # 2. Shoulder Pitch (turntable_link_joint)
                sh_deg = self.calculate_angle_2d(hp, sp, ep)
                if sh_deg < 25.0:
                    raw_shoulder = -0.3   # Resting neutral position
                    sh_desc = "ARM RESTING"
                else:
                    raw_shoulder = (sh_deg - 25.0) * (math.pi / 180.0) - 0.2
                    sh_desc = "ARM ELEVATED"

                self.target_angles['turntable_link_joint'] = max(-1.2, min(1.2, raw_shoulder))

                # 3. Elbow Pitch (turntable_link_joint_dup_1)
                el_deg = self.calculate_angle_2d(sp, ep, wp)
                if el_deg > 155.0:
                    raw_elbow = 0.2     # Arm straight / extended
                else:
                    raw_elbow = ((155.0 - el_deg) / 130.0) * 2.0 + 0.2

                self.target_angles['turntable_link_joint_dup_1'] = max(-0.8, min(1.8, raw_elbow))

                # 4. Wrist Pitch (turntable_link_joint_dup_2)
                wr_deg = self.calculate_angle_2d(ep, wp, ip)
                raw_wrist = (wr_deg - 90.0) * (math.pi / 180.0)
                self.target_angles['turntable_link_joint_dup_2'] = max(-1.0, min(1.0, raw_wrist))

                # 5. Wrist Twist / Spin (turntable_link_joint_dup_3)
                # Measures the roll/tilt of the knuckle vector (Pinky to Thumb)
                # relative to the hand pointing vector (Wrist to Index).
                ux = ip[0] - wp[0]
                uy = ip[1] - wp[1]
                u_len = math.hypot(ux, uy)

                wx = (tp[0] - pp[0]) if is_right_arm else (pp[0] - tp[0])
                wy = (tp[1] - pp[1]) if is_right_arm else (pp[1] - tp[1])
                w_len = math.hypot(wx, wy)

                if u_len > 5.0 and w_len > 5.0:
                    roll_dot = (wx * ux + wy * uy) / (w_len * u_len)
                    # Deadband (-0.18 .. +0.18) for neutral flat hand
                    if abs(roll_dot) < 0.18:
                        raw_spin = 0.0
                        spin_desc = "WRIST FLAT"
                    else:
                        raw_spin = math.copysign((abs(roll_dot) - 0.18) * 2.2, roll_dot)
                        spin_desc = "WRIST SPIN CW" if raw_spin > 0 else "WRIST SPIN CCW"
                    self.target_angles['turntable_link_joint_dup_3'] = max(-1.8, min(1.8, raw_spin))
                else:
                    spin_desc = "WRIST LEVEL"
                    self.target_angles['turntable_link_joint_dup_3'] = 0.0

                # 6. Gripper Open / Close: hand length relative to the forearm, so it
                # doesn't depend on the distance to the camera. Hysteresis stops chatter.
                forearm = math.hypot(wp[0] - ep[0], wp[1] - ep[1])
                self.grip_ratio = math.hypot(ip[0] - wp[0], ip[1] - wp[1]) / forearm if forearm > 1.0 else 0.0
                if self.grip_open and self.grip_ratio < self.grip_close_ratio:
                    self.grip_open = False
                elif not self.grip_open and self.grip_ratio > self.grip_open_ratio:
                    self.grip_open = True
                grip_desc = "GRIPPER OPEN" if self.grip_open else "GRIPPER CLOSED"
                self.target_angles['grip'] = self.grip_open_rad if self.grip_open else self.grip_closed_rad

                self.posture_state = f"{sh_desc} | {yaw_desc} | {spin_desc} | {grip_desc}"

        # ── Apply Heavy Low-Pass EMA Filter & Deadband Thresholding ──────────
        for j in self.joint_names:
            diff = self.target_angles[j] - self.current_angles[j]
            # Apply deadband: ignore tiny fluctuations below deadband threshold
            if abs(diff) > self.deadband_rad:
                self.current_angles[j] += self.alpha * diff

        # ── Publish ROS 2 JointState Message ────────────────────────────────
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.current_angles[j] for j in self.joint_names]
        if rclpy.ok():
            try:
                self.joint_pub.publish(msg)
            except Exception:
                return

        if not self.show_window:
            return

        # ── Render Futuristic Monochromatic HUD Overlay ──────────────────────
        # Top-Left Telemetry
        cv2.putText(hud_frame, "UPPER-BODY GESTURE TELEOP (LOW-SENSITIVITY)", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
        cv2.putText(hud_frame, f"STATUS: {arm_label if pose_detected else 'SEARCHING POSTURE...'}", (30, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 200, 255) if pose_detected else (150, 150, 150), 1)

        if pose_detected:
            cv2.putText(hud_frame, f"POSTURE: {self.posture_state}", (30, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 200), 1)

        # Right Vertical HUD Telemetry
        rx = w - 140
        cv2.putText(hud_frame, "FIG 01", (rx, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(hud_frame, f"X {rel_x:+05.1f}", (rx, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        cv2.putText(hud_frame, f"Y {rel_y:+05.1f}", (rx, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        cv2.putText(hud_frame, f"Z {rel_z:+05.1f}", (rx, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        cv2.putText(hud_frame, f"GRIP {self.grip_ratio:.2f}", (rx, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

        # Bottom-Left Joint Angles
        j1_deg = math.degrees(self.current_angles['turntable_link_joint_dup'])
        j2_deg = math.degrees(self.current_angles['turntable_link_joint'])
        j3_deg = math.degrees(self.current_angles['turntable_link_joint_dup_1'])
        j4_deg = math.degrees(self.current_angles['turntable_link_joint_dup_2'])
        j5_deg = math.degrees(self.current_angles['turntable_link_joint_dup_3'])

        cv2.putText(hud_frame, f"BASE YAW:   {j1_deg:+.1f} deg", (30, h - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
        cv2.putText(hud_frame, f"SHOULDER:   {j2_deg:+.1f} deg", (30, h - 72), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
        cv2.putText(hud_frame, f"ELBOW:      {j3_deg:+.1f} deg", (30, h - 54), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
        cv2.putText(hud_frame, f"WRIST PITCH:{j4_deg:+.1f} deg", (30, h - 36), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
        cv2.putText(hud_frame, f"WRIST SPIN: {j5_deg:+.1f} deg", (30, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 200), 1)

        # Display Stylized HUD Output (q or Esc in the window quits)
        try:
            if not self.window_initialized:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 640, 480)
                try:
                    cv2.setWindowProperty(self.window_name, cv2.WND_PROP_TOPMOST, 1)
                except Exception:
                    pass
                self.window_initialized = True

            cv2.imshow(self.window_name, hud_frame)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as e:
            self.get_logger().warn(f"⚠️ No HUD window ({getattr(e, 'msg', e)}); tracking continues without it.")
            self.show_window = False
            return
        if key == 27 or key == ord('q'):
            self.get_logger().info("Quitting Low-Sensitivity Teleop...")
            self.quit_requested = True

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        if self.pose_tracker is not None:
            try:
                self.pose_tracker.close()
            except Exception:
                pass
        if cv2 is not None and self.show_window:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureTeleopNode()
    try:
        node.start()
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except StartupError as e:
        node.fail(str(e))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(node.exit_code)


if __name__ == '__main__':
    main()
