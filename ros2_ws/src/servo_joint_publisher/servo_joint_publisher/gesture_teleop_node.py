#!/usr/bin/env python3
"""
ROS 2 Node: gesture_teleop_node (Low-Sensitivity Defined Posture Tracking)
---------------------------------------------------------------------------
Captures live webcam feed (/dev/video0), tracks human upper-body posture (strictly
till waist), applies heavy low-pass exponential smoothing (alpha = 0.10) and deadband filtering
to eliminate jitter, and maps defined postures to the 4 URDF joint angles.

Sensitivity & Defined Gesture Features:
  • Low-Pass EMA Filter (alpha = 0.10) for steady, jitter-free servo motion
  • 1.5° Deadband threshold to suppress small body/hand twitches
  • Defined posture zones: Neutral Center, Left/Right Base Sweep, Shoulder Elevation, Gripper Open/Close
  • Live Cyberpunk HUD with active posture indicators and low-sensitivity status.
"""

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

HAS_MEDIAPIPE = False
try:
    import mediapipe as mp
    HAS_MEDIAPIPE = True
except ImportError:
    HAS_MEDIAPIPE = False


class GestureTeleopNode(Node):

    def __init__(self):
        super().__init__('gesture_teleop_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('video_device', 0)
        self.declare_parameter('smooth_alpha', 0.10)   # Reduced from 0.25 -> 0.10 for heavy jitter suppression
        self.declare_parameter('deadband_deg', 1.5)     # Deadband threshold in degrees
        self.declare_parameter('publish_rate', 30.0)

        device_id = self.get_parameter('video_device').get_parameter_value().integer_value
        self.alpha = self.get_parameter('smooth_alpha').get_parameter_value().double_value
        self.deadband_rad = math.radians(self.get_parameter('deadband_deg').get_parameter_value().double_value)
        pub_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # ── ROS 2 Publishers ─────────────────────────────────────────────
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── Joint State Definitions ──────────────────────────────────────
        self.joint_names = [
            'turntable_link_joint_dup',    # Base Turntable Yaw
            'turntable_link_joint',        # Lower Shoulder Pitch
            'turntable_link_joint_dup_1',  # Upper Forearm Pitch
            'turntable_link_joint_dup_2',  # Wrist Pitch
            'turntable_link_joint_dup_4',  # Gripper Actuated Joint
            'left_parallel_link_joint'     # Gripper Parallel Link
        ]

        self.target_angles = {j: 0.0 for j in self.joint_names}
        self.current_angles = {j: 0.0 for j in self.joint_names}

        # Set default home pose positions
        self.target_angles['turntable_link_joint'] = 0.3    # ~17° shoulder elevation
        self.target_angles['turntable_link_joint_dup_1'] = 1.0  # ~57° elbow pitch
        self.target_angles['turntable_link_joint_dup_2'] = 0.5  # ~28° wrist pitch
        self.current_angles = dict(self.target_angles)

        # ── Video Capture & MediaPipe Setup ──────────────────────────────
        self.get_logger().info(f"🎥 Initializing Low-Sensitivity Teleop Video Stream on /dev/video{device_id}...")
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Failed to open video device /dev/video{device_id}!")
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info("✅ Camera initialized successfully.")

        self.pose_tracker = None
        if HAS_MEDIAPIPE:
            try:
                self.mp_pose = mp.solutions.pose
                self.pose_tracker = self.mp_pose.Pose(
                    static_image_mode=False,
                    model_complexity=1,
                    smooth_landmarks=True,
                    min_detection_confidence=0.6,
                    min_tracking_confidence=0.6
                )
                self.get_logger().info("🦴 MediaPipe Upper-Body Tracker Initialized.")
            except Exception as e:
                self.get_logger().warn(f"⚠️ MediaPipe Init Warning: {e}")

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

        # Timer loop @ publish_rate Hz
        self.timer = self.create_timer(1.0 / pub_rate, self.processing_loop)
        self.get_logger().info("🚀 Low-Sensitivity Defined Gesture Teleop Node Ready!")

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
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        # Mirror frame horizontally
        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape

        # Desaturated high-contrast monochrome style
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hud_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        hud_frame = cv2.convertScaleAbs(hud_frame, alpha=0.85, beta=10)

        pose_detected = False
        arm_label = "STAND IN FRONT OF WEBCAM"
        rel_x, rel_y, rel_z = 0.0, 0.0, 0.0

        if self.pose_tracker is not None:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.pose_tracker.process(rgb_frame)

            if results.pose_landmarks:
                pose_detected = True
                landmarks = results.pose_landmarks.landmark

                # Extract upper-body landmark coordinates (strictly 0..24, stopping at waist/hips)
                lm_points = {}
                ub_x_coords, ub_y_coords = [], []

                for idx in range(25):  # 0 to 24 (waist level)
                    lm = landmarks[idx]
                    px, py = int(lm.x * w), int(lm.y * h)
                    lm_points[idx] = (px, py, lm.z, lm.visibility)
                    if lm.visibility > 0.5:
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

                # Select active arm
                r_wrist = landmarks[16]
                l_wrist = landmarks[15]

                if r_wrist.y < l_wrist.y:
                    s_pt, e_pt, w_pt, hip_pt, index_pt = (12, 14, 16, 24, 20)
                    arm_label = "RIGHT ARM ACTIVE"
                else:
                    s_pt, e_pt, w_pt, hip_pt, index_pt = (11, 13, 15, 23, 19)
                    arm_label = "LEFT ARM ACTIVE"

                sp = (lm_points[s_pt][0], lm_points[s_pt][1])
                ep = (lm_points[e_pt][0], lm_points[e_pt][1])
                wp = (lm_points[w_pt][0], lm_points[w_pt][1])
                hp = (lm_points[hip_pt][0], lm_points[hip_pt][1])
                ip = (lm_points[index_pt][0], lm_points[index_pt][1])

                rel_x = (wp[0] - sp[0]) / 10.0
                rel_y = (wp[1] - sp[1]) / 10.0
                rel_z = lm_points[w_pt][2] * 100.0

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
                    el_desc = "ELBOW EXTENDED"
                else:
                    raw_elbow = ((155.0 - el_deg) / 130.0) * 2.0 + 0.2
                    el_desc = "ELBOW FLEXED"

                self.target_angles['turntable_link_joint_dup_1'] = max(-0.8, min(1.8, raw_elbow))

                # 4. Wrist Pitch (turntable_link_joint_dup_2)
                wr_deg = self.calculate_angle_2d(ep, wp, ip)
                raw_wrist = (wr_deg - 90.0) * (math.pi / 180.0)
                self.target_angles['turntable_link_joint_dup_2'] = max(-1.0, min(1.0, raw_wrist))

                # 5. Gripper Open / Close Gesture Threshold
                pinch_dist = math.sqrt((ip[0] - wp[0])**2 + (ip[1] - wp[1])**2)
                if pinch_dist < 45.0:
                    raw_grip = 0.0      # Closed Fist / Gripper Closed
                    grip_desc = "GRIPPER CLOSED"
                else:
                    raw_grip = 0.7      # Open Hand / Gripper Open
                    grip_desc = "GRIPPER OPEN"

                self.target_angles['turntable_link_joint_dup_4'] = raw_grip
                self.target_angles['left_parallel_link_joint'] = -raw_grip

                self.posture_state = f"{sh_desc} | {yaw_desc} | {grip_desc}"

        # ── Apply Heavy Low-Pass EMA Filter & Deadband Thresholding ──────────
        for j in self.joint_names:
            diff = self.target_angles[j] - self.current_angles[j]
            # Apply deadband: ignore tiny fluctuations below deadband threshold
            if abs(diff) > self.deadband_rad:
                self.current_angles[j] += self.alpha * diff

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

        # Bottom-Left Joint Angles
        j1_deg = math.degrees(self.current_angles['turntable_link_joint_dup'])
        j2_deg = math.degrees(self.current_angles['turntable_link_joint'])
        j3_deg = math.degrees(self.current_angles['turntable_link_joint_dup_1'])
        j4_deg = math.degrees(self.current_angles['turntable_link_joint_dup_2'])

        cv2.putText(hud_frame, f"BASE YAW: {j1_deg:.1f} deg", (30, h - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        cv2.putText(hud_frame, f"SHOULDER: {j2_deg:.1f} deg", (30, h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        cv2.putText(hud_frame, f"ELBOW: {j3_deg:.1f} deg", (30, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # ── Publish ROS 2 JointState Message ────────────────────────────────
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.current_angles[j] for j in self.joint_names]
        self.joint_pub.publish(msg)

        # Display Stylized HUD Output
        cv2.imshow("Code for India — Upper Body Cyberpunk Gesture Teleop", hud_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            self.get_logger().info("Quitting Low-Sensitivity Teleop...")
            rclpy.shutdown()

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
