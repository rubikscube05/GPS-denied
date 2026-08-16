#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from geographic_msgs.msg import GeoPointStamped

from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome


class ArduPilotArUcoNavigator(Node):
    def __init__(self):
        super().__init__('ardupilot_aruco_navigator')

        self.bridge = CvBridge()

        # ==========================================
        # 1. TOPICS & NAMESPACES
        # ==========================================
        self.NAMESPACE = '/mavros'  # confirmed against live `ros2 topic list` (no /Joey prefix on mavros topics)

        IMAGE_TOPIC = '/world/iris_runway/model/Joey/model/gimbal/link/pitch_link/sensor/camera/image'
        CAMERA_INFO_TOPIC = '/world/iris_runway/model/Joey/model/gimbal/link/pitch_link/sensor/camera/camera_info'
        STATE_TOPIC = f'{self.NAMESPACE}/state'
        ODOM_TOPIC = f'{self.NAMESPACE}/local_position/odom'
        VISION_POSE_TOPIC = f'{self.NAMESPACE}/vision_pose/pose'
        SETPOINT_TOPIC = f'{self.NAMESPACE}/setpoint_position/local'
        ORIGIN_TOPIC = f'{self.NAMESPACE}/global_position/set_gp_origin'
        STATUSTEXT_TOPIC = f'{self.NAMESPACE}/statustext/recv'

        # ==========================================
        # 2. MISSION & VISION PARAMETERS
        # ==========================================
        self.target_marker_id = 0
        # aruco_marker_0/model.sdf: flat <box><size>0.5 0.5 0.01</size></box>,
        # texture is a decorative silhouette with a single functional tag
        # (DICT_5X5_50 id 0) nested dead-center, sized 0.15m (30% of the
        # 0.5m plate) - verified against
        # ardupilot_gazebo/models/aruco_marker_0/materials/textures/marker.png.
        # Centering keeps the tag in frame even if the drone/camera isn't
        # perfectly centered over the plate.
        self.marker_size = 0.15

        self.takeoff_altitude = 3.0
        self.target_xyz = np.array([0.0, 0.0, 3.0])

        # Raspberry Pi Camera V2 Specs
        self.camera_fov_h = 62.2
        self.camera_fov_v = 48.8

        self.fsm_state = "WAITING_FOR_ODOM"
        self.current_setpoint = np.array([0.0, 0.0, 0.0])

        self.last_marker_time = self.get_clock().now()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.current_pose = Pose()
        self.current_state = State()

        self.has_vision = False
        self.has_odom = False
        self.origin_set = False

        self.latest_vision_pose = None
        self.last_service_call_time = self.get_clock().now()

        # 3D object points for solvePnP
        half = self.marker_size / 2.0
        self.obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None

        # ==========================================
        # 3. PUBLISHERS, SUBSCRIBERS & CLIENTS
        # ==========================================
        sensor_qos = qos_profile_sensor_data
        setpoint_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        self.create_subscription(State, STATE_TOPIC, self.state_callback, setpoint_qos)
        self.create_subscription(Image, IMAGE_TOPIC, self.image_callback, sensor_qos)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, sensor_qos)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, sensor_qos)
        self.create_subscription(StatusText, STATUSTEXT_TOPIC, self.statustext_callback, sensor_qos)

        self.vision_pub = self.create_publisher(PoseStamped, VISION_POSE_TOPIC, setpoint_qos)
        self.setpoint_pub = self.create_publisher(PoseStamped, SETPOINT_TOPIC, setpoint_qos)
        self.origin_pub = self.create_publisher(GeoPointStamped, ORIGIN_TOPIC, 10)

        self.arm_client = self.create_client(CommandBool, f'{self.NAMESPACE}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'{self.NAMESPACE}/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, f'{self.NAMESPACE}/cmd/takeoff')
        self.set_home_client = self.create_client(CommandHome, f'{self.NAMESPACE}/cmd/set_home')

        self.control_timer = self.create_timer(0.1, self.fsm_control_loop)
        self.vision_timer = self.create_timer(0.05, self.vision_stream_callback)
        # marker is too close to see while grounded (0.5m marker, ~0.2m mount
        # height); set origin up front instead of waiting on first detection
        self.origin_timer = self.create_timer(1.0, self._origin_timer_cb)

        self.get_logger().info("ArduPilot Navigator Initialized. Waiting for ArUco marker...")

    # ==========================================
    # 4. CALLBACK FUNCTIONS
    # ==========================================
    def state_callback(self, msg: State):
        self.current_state = msg

    def statustext_callback(self, msg: StatusText):
        self.get_logger().warn(f"FCU: {msg.text}")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera Intrinsic Matrix received.")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        if not self.has_odom:
            self.get_logger().info("Local odometry is publishing.")
            self.has_odom = True

    def set_ekf_origin(self):
        origin_msg = GeoPointStamped()
        origin_msg.header.stamp = self.get_clock().now().to_msg()
        origin_msg.header.frame_id = "map"
        origin_msg.position.latitude = -35.363262
        origin_msg.position.longitude = 149.165237
        origin_msg.position.altitude = 584.0
        self.origin_pub.publish(origin_msg)
        self.get_logger().info("Published EKF Global Origin to ArduPilot!")

    def _origin_timer_cb(self):
        if not self.origin_set:
            self.set_ekf_origin()
            self.origin_set = True
        self.origin_timer.cancel()

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        best_corners = None
        target_idx = np.where(ids == self.target_marker_id)[0] if ids is not None else []

        if len(target_idx) > 0:
            best_corners = corners[target_idx[0]][0]
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        if best_corners is not None:
            self.has_vision = True
            cv2.putText(cv_image, "Tracking: MARKER", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, best_corners, self.camera_matrix, self.dist_coeffs
            )

            if success:
                self.last_marker_time = self.get_clock().now()
                self.process_vision_pose(tvec.flatten(), rvec.flatten(), msg.header.stamp)
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.2)

        try:
            cv2.imshow("ArUco Tracking Feed", cv_image)
            cv2.waitKey(1)
        except cv2.error:
            pass

    # ==========================================
    # 5. POSE CALCULATION & 20Hz STREAMING
    # ==========================================
    def process_vision_pose(self, tvec, rvec, image_stamp):
        R, _ = cv2.Rodrigues(rvec)
        R_inv = np.transpose(R)
        camera_tvec = -np.dot(R_inv, tvec)  # camera position in marker (~world) frame

        drone_x = float(camera_tvec[0])
        drone_y = float(-camera_tvec[1])
        drone_z = abs(float(camera_tvec[2]))

        R_drone = np.dot(np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]]), R_inv)
        q = self.rotation_matrix_to_quaternion(R_drone)

        vision_pose = PoseStamped()
        vision_pose.header.stamp = image_stamp
        vision_pose.header.frame_id = "map"

        vision_pose.pose.position.x = drone_x
        vision_pose.pose.position.y = drone_y
        vision_pose.pose.position.z = drone_z

        vision_pose.pose.orientation.x = q[0]
        vision_pose.pose.orientation.y = q[1]
        vision_pose.pose.orientation.z = q[2]
        vision_pose.pose.orientation.w = q[3]

        self.latest_vision_pose = vision_pose

    def vision_stream_callback(self):
        now = self.get_clock().now()
        time_since_last_seen = (now - self.last_marker_time).nanoseconds / 1e9

        if self.latest_vision_pose is not None:
            if time_since_last_seen < 0.2:
                self.latest_vision_pose.header.stamp = now.to_msg()
                self.vision_pub.publish(self.latest_vision_pose)
            else:
                self.get_logger().warn("Marker lost from FOV! Vision odometry paused to protect EKF.", throttle_duration_sec=2.0)

    def rotation_matrix_to_quaternion(self, R):
        q = np.empty(4)
        trace = np.trace(R)
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            q[3] = 0.25 * s
            q[0] = (R[2, 1] - R[1, 2]) / s
            q[1] = (R[0, 2] - R[2, 0]) / s
            q[2] = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return q

    # ==========================================
    # 6. FINITE STATE MACHINE (10 Hz)
    # ==========================================
    def fsm_control_loop(self):
        now = self.get_clock().now()
        time_since_last_call = (now - self.last_service_call_time).nanoseconds / 1e9

        if self.fsm_state == "WAITING_FOR_ODOM":
            if self.has_odom:
                self.get_logger().info("Odometry received! Setting Home and requesting GUIDED_NOGPS mode...")
                self.set_home_position()
                self.fsm_state = "SET_GUIDED_NOGPS"

        # GUIDED_NOGPS doesn't need a position estimate to arm (attitude/baro-alt
        # only), so it's used to get off the ground before the marker is visible.
        # The plate-mounted marker sits well inside the camera's ground-level FOV
        # margin, so it isn't reliably decodable until the drone climbs clear of it.
        elif self.fsm_state == "SET_GUIDED_NOGPS":
            if self.current_state.mode == "GUIDED_NOGPS":
                self.get_logger().info("Mode is GUIDED_NOGPS. Sending ARM command...")
                self.fsm_state = "ARMING"
            elif time_since_last_call > 2.0:
                self.set_mode("GUIDED_NOGPS")
                self.last_service_call_time = now

        elif self.fsm_state == "ARMING":
            if self.current_state.armed:
                self.get_logger().info("Vehicle is ARMED! Commanding Takeoff...")
                self.fsm_state = "SEND_TAKEOFF"
            elif time_since_last_call > 2.0:
                self.arm_vehicle(True)
                self.last_service_call_time = now

        elif self.fsm_state == "SEND_TAKEOFF":
            if time_since_last_call > 2.0:
                self.takeoff_vehicle(self.takeoff_altitude)
                self.last_service_call_time = now
                self.fsm_state = "TAKEOFF_NOGPS"

        elif self.fsm_state == "TAKEOFF_NOGPS":
            alt_error = abs(self.current_pose.position.z - self.takeoff_altitude)
            marker_seen_recently = (now - self.last_marker_time).nanoseconds / 1e9 < 0.5
            if alt_error >= 0.3:
                pass
            elif not (self.has_vision and marker_seen_recently):
                self.get_logger().warn("At altitude, marker not yet locked - holding in GUIDED_NOGPS.", throttle_duration_sec=2.0)
            else:
                self.get_logger().info("Marker locked at altitude. Switching to GUIDED for vision-based control...")
                self.set_mode("GUIDED")
                self.last_service_call_time = now
                self.fsm_state = "SWITCHING_TO_GUIDED"

        elif self.fsm_state == "SWITCHING_TO_GUIDED":
            if self.current_state.mode == "GUIDED":
                max_x = self.takeoff_altitude * np.tan(np.radians(self.camera_fov_h) / 2) * 0.8
                max_y = self.takeoff_altitude * np.tan(np.radians(self.camera_fov_v) / 2) * 0.8

                safe_x = float(np.clip(self.target_xyz[0], -max_x, max_x))
                safe_y = float(np.clip(self.target_xyz[1], -max_y, max_y))

                self.current_setpoint = np.array([safe_x, safe_y, self.target_xyz[2]])

                self.get_logger().info(f"Mode is GUIDED. Target clamped to FOV bounds: [{safe_x:.2f}, {safe_y:.2f}, {self.target_xyz[2]:.2f}]")
                self.fsm_state = "NAVIGATING"
            elif time_since_last_call > 2.0:
                self.set_mode("GUIDED")
                self.last_service_call_time = now

        elif self.fsm_state == "NAVIGATING":
            dx = self.current_pose.position.x - self.current_setpoint[0]
            dy = self.current_pose.position.y - self.current_setpoint[1]
            dz = self.current_pose.position.z - self.current_setpoint[2]
            dist = np.sqrt(dx**2 + dy**2 + dz**2)

            if dist < 0.25:
                self.get_logger().info("Target Waypoint Reached! Hovering.", throttle_duration_sec=10.0)
                self.fsm_state = "HOVER"

        elif self.fsm_state == "HOVER":
            pass

        if self.has_odom and self.fsm_state in ["NAVIGATING", "HOVER"]:
            goal = PoseStamped()
            goal.header.stamp = now.to_msg()
            goal.header.frame_id = "map"

            goal.pose.position.x = float(self.current_setpoint[0])
            goal.pose.position.y = float(self.current_setpoint[1])
            goal.pose.position.z = float(self.current_setpoint[2])
            goal.pose.orientation = self.current_pose.orientation

            self.setpoint_pub.publish(goal)

    # ==========================================
    # 7. SERVICE CALL HELPERS
    # ==========================================
    def set_mode(self, custom_mode: str):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("SetMode service unavailable")
            return
        req = SetMode.Request()
        req.custom_mode = custom_mode
        self.mode_client.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(f"set_mode({custom_mode}) -> mode_sent={f.result().mode_sent}"))

    def arm_vehicle(self, arm: bool):
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Arming service unavailable")
            return
        req = CommandBool.Request()
        req.value = arm
        self.arm_client.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(f"arm({arm}) -> success={f.result().success} result={f.result().result}"))

    def takeoff_vehicle(self, alt: float):
        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Takeoff service unavailable")
            return
        req = CommandTOL.Request()
        req.altitude = float(alt)
        req.latitude = 0.0
        req.longitude = 0.0
        self.takeoff_client.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(f"takeoff({alt}) -> success={f.result().success} result={f.result().result}"))

    def set_home_position(self):
        if not self.set_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Set Home service unavailable")
            return
        req = CommandHome.Request()
        req.current_gps = False
        req.latitude = -35.363262
        req.longitude = 149.165237
        req.altitude = 584.0
        self.set_home_client.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(f"set_home -> success={f.result().success}"))
        self.get_logger().info("Forced ArduPilot Home Position explicitly using simulated coordinates!")


def main(args=None):
    rclpy.init(args=args)
    navigator = ArduPilotArUcoNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.try_shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
