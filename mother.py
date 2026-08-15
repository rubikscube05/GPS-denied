#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
import cv2
import numpy as np

# ROS 2 Interfaces
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from geographic_msgs.msg import GeoPointStamped

# MAVROS Messages & Services
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome

class ArduPilotArUcoNavigator(Node):
    def __init__(self):
        super().__init__('ardupilot_aruco_navigator')
        
        # Enforce simulation time for Gazebo
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.bridge = CvBridge()

        # ==========================================
        # 1. TOPICS & NAMESPACES
        # ==========================================
        self.NAMESPACE = '/mavros'  # Matched to your MAVROS setup
        
        IMAGE_TOPIC = '/world/iris_runway/model/Joy/model/gimbal/link/pitch_link/sensor/camera/image'
        CAMERA_INFO_TOPIC = '/world/iris_runway/model/Joy/model/gimbal/link/pitch_link/sensor/camera/camera_info'
        STATE_TOPIC = f'{self.NAMESPACE}/state'
        ODOM_TOPIC = f'{self.NAMESPACE}/local_position/odom'
        VISION_POSE_TOPIC = f'{self.NAMESPACE}/vision_pose/pose'
        SETPOINT_TOPIC = f'{self.NAMESPACE}/setpoint_position/local'
        ORIGIN_TOPIC = f'{self.NAMESPACE}/global_position/set_gp_origin'
        
        # ==========================================
        # 2. MISSION & VISION PARAMETERS
        # ==========================================
        self.target_marker_id = 0
        self.marker_size_outer = 0.15      # 15 cm outer marker
        self.marker_size_inner = 0.064     # ~6.4 cm inner marker (MEASURE & ADJUST THIS!)
        
        self.takeoff_altitude = 0.5   # Target takeoff height (meters)
        self.target_xyz = np.array([0.0, 0.0, 1.5]) # User requested target (will be dynamically clamped)
        
        
        # Raspberry Pi Camera V2 Specs
        self.camera_fov_h = 62.2
        self.camera_fov_v = 48.8
        
        self.fsm_state = "WAITING_FOR_VISION"
        self.current_setpoint = np.array([0.0, 0.0, 0.0])

        self.last_marker_time = self.get_clock().now()
        self.last_z_estimate = self.takeoff_altitude # Used for nested marker area heuristic

        # Internal Variables
        self.camera_matrix = None
        self.dist_coeffs = None
        self.current_pose = Pose()
        self.current_state = State()
        
        self.has_vision = False
        self.has_odom = False
        self.origin_set = False
        
        self.latest_vision_pose = None
        self.last_service_call_time = self.get_clock().now()

        # 3D object points for OpenCV solvePnP (OUTER MARKER)
        half_outer = self.marker_size_outer / 2.0
        self.obj_points_outer = np.array([
            [-half_outer,  half_outer, 0],
            [ half_outer,  half_outer, 0],
            [ half_outer, -half_outer, 0],
            [-half_outer, -half_outer, 0]
        ], dtype=np.float32)

        # 3D object points for OpenCV solvePnP (INNER MARKER)
        half_inner = self.marker_size_inner / 2.0
        self.obj_points_inner = np.array([
            [-half_inner,  half_inner, 0],
            [ half_inner,  half_inner, 0],
            [ half_inner, -half_inner, 0],
            [-half_inner, -half_inner, 0]
        ], dtype=np.float32)

        # Version-agnostic OpenCV ArUco Setup
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
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

        self.vision_pub = self.create_publisher(PoseStamped, VISION_POSE_TOPIC, setpoint_qos)
        self.setpoint_pub = self.create_publisher(PoseStamped, SETPOINT_TOPIC, setpoint_qos)
        self.origin_pub = self.create_publisher(GeoPointStamped, ORIGIN_TOPIC, 10)

        self.arm_client = self.create_client(CommandBool, f'{self.NAMESPACE}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'{self.NAMESPACE}/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, f'{self.NAMESPACE}/cmd/takeoff')
        self.set_home_client = self.create_client(CommandHome, f'{self.NAMESPACE}/cmd/set_home')

        self.control_timer = self.create_timer(0.1, self.fsm_control_loop)
        self.vision_timer = self.create_timer(0.05, self.vision_stream_callback)

        self.get_logger().info("ArduPilot Navigator Initialized. Waiting for Nested ArUco Marker...")

    # ==========================================
    # 4. CALLBACK FUNCTIONS
    # ==========================================
    def state_callback(self, msg: State):
        self.current_state = msg

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera Intrinsic Matrix received.")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        if not self.has_odom:
            self.get_logger().info("SUCCESS: ArduPilot EKF3 is outputting Odometry! System is healthy.")
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

        if ids is not None:
            self.has_vision = True
            
            if not self.origin_set:
                self.set_ekf_origin()
                self.origin_set = True

            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            if self.target_marker_id in ids:
                
                # Find all indices where the ID matches our target ID (0)
                target_indices = np.where(ids == self.target_marker_id)[0]
                
                best_corners = None
                best_obj_points = None
                
                # --- NESTED MARKER LOGIC ---
                if len(target_indices) >= 2:
                    # Both markers are visible. Sort by contour area to find the largest (outer)
                    target_indices = sorted(target_indices, key=lambda i: cv2.contourArea(corners[i][0]), reverse=True)
                    
                    # Track the outer marker for more stable pose estimation
                    best_corners = corners[target_indices[0]][0]
                    best_obj_points = self.obj_points_outer
                    cv2.putText(cv_image, "Tracking: OUTER (Both Visible)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                elif len(target_indices) == 1:
                    idx = target_indices[0]
                    c = corners[idx][0]
                    area = cv2.contourArea(c)
                    
                    # Heuristic: Determine if the single visible marker is the Inner or Outer one
                    # by estimating expected pixel area based on our last known Z altitude.
                    current_z = max(self.last_z_estimate, 0.1) # Prevent div by zero
                    
                    fx = self.camera_matrix[0, 0]
                    fy = self.camera_matrix[1, 1]
                    
                    # Approximate expected pixel area = (focal_length * physical_size / distance)^2
                    expected_area_outer = (fx * self.marker_size_outer / current_z) * (fy * self.marker_size_outer / current_z)
                    expected_area_inner = (fx * self.marker_size_inner / current_z) * (fy * self.marker_size_inner / current_z)
                    
                    # Choose the model that best matches the observed area
                    if abs(area - expected_area_outer) < abs(area - expected_area_inner):
                        best_corners = c
                        best_obj_points = self.obj_points_outer
                        cv2.putText(cv_image, "Tracking: OUTER (Single)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    else:
                        best_corners = c
                        best_obj_points = self.obj_points_inner
                        cv2.putText(cv_image, "Tracking: INNER (Single)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

                # Execute solvePnP with the dynamically selected object points
                if best_corners is not None:
                    success, rvec, tvec = cv2.solvePnP(
                        best_obj_points, best_corners, self.camera_matrix, self.dist_coeffs
                    )

                    if success:
                        self.last_marker_time = self.get_clock().now()
                        self.last_z_estimate = abs(float(tvec[2])) # Cache altitude for next frame's heuristic
                        self.process_vision_pose(tvec.flatten(), rvec.flatten(), msg.header.stamp)
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

        cv2.imshow("ArUco Tracking Feed", cv_image)
        cv2.waitKey(1)

    # ==========================================
    # 5. POSE CALCULATION & 20Hz STREAMING
    # ==========================================
    def process_vision_pose(self, tvec, rvec, image_stamp):
        R, _ = cv2.Rodrigues(rvec)
        R_inv = np.transpose(R)
        camera_tvec = -np.dot(R_inv, tvec) # Camera position in OpenCV Marker Frame

        # Correct Mapping: OpenCV Marker Frame to ROS ENU World Frame
        drone_x = float(camera_tvec[0])
        drone_y = float(-camera_tvec[1])
        drone_z = abs(float(camera_tvec[2])) # Ensure altitude is positive above marker

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

        if self.fsm_state == "WAITING_FOR_VISION":
            if self.has_vision:
                self.get_logger().info("Vision acquired. Waiting for ArduPilot EKF to initialize odometry...")
                self.fsm_state = "WAITING_FOR_ODOM"

        elif self.fsm_state == "WAITING_FOR_ODOM":
            if self.has_odom:
                self.get_logger().info("Odometry received! Setting Home and requesting GUIDED mode...")
                self.set_home_position()
                self.fsm_state = "SET_GUIDED"

        elif self.fsm_state == "SET_GUIDED":
            if self.current_state.mode == "GUIDED":
                self.get_logger().info("Mode is GUIDED. Sending ARM command...")
                self.fsm_state = "ARMING"
            elif time_since_last_call > 2.0:
                self.set_mode("GUIDED")
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
                self.fsm_state = "TAKEOFF"

        elif self.fsm_state == "TAKEOFF":
            alt_error = abs(self.current_pose.position.z - self.takeoff_altitude)
            if alt_error < 0.2:
                max_x = self.takeoff_altitude * np.tan(np.radians(self.camera_fov_h) / 2) * 0.8
                max_y = self.takeoff_altitude * np.tan(np.radians(self.camera_fov_v) / 2) * 0.8
                
                safe_x = float(np.clip(self.target_xyz[0], -max_x, max_x))
                safe_y = float(np.clip(self.target_xyz[1], -max_y, max_y))
                
                self.current_setpoint = np.array([safe_x, safe_y, self.target_xyz[2]])
                
                self.get_logger().info(f"Reached altitude. Target clamped to RPi Cam FOV bounds: [{safe_x:.2f}, {safe_y:.2f}, {self.target_xyz[2]:.2f}]")
                self.fsm_state = "NAVIGATING"

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
        self.mode_client.call_async(req)

    def arm_vehicle(self, arm: bool):
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Arming service unavailable")
            return
        req = CommandBool.Request()
        req.value = arm
        self.arm_client.call_async(req)

    def takeoff_vehicle(self, alt: float):
        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Takeoff service unavailable")
            return
        req = CommandTOL.Request()
        req.altitude = float(alt)
        req.latitude = 0.0
        req.longitude = 0.0
        self.takeoff_client.call_async(req)
        
    def set_home_position(self):
        if not self.set_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Set Home service unavailable")
            return
        req = CommandHome.Request()
        req.current_gps = False
        req.latitude = -35.363262
        req.longitude = 149.165237
        req.altitude = 584.0
        self.set_home_client.call_async(req)
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
