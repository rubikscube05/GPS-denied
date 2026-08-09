#!/usr/bin/env python3

"""
ArduPilot Precision Landing Node — HYBRID SEARCH & MEMORY
===========================================================================
1. Initial Search: Expanding square pattern if the marker has never been seen.
2. Tracking: PD control to center over the marker + saves True World Coordinates.
3. Recovery: If lost, flies straight to saved memory and descends vertically.
4. Failsafe: Forces a LAND command if altitude drops below 0.50m.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image 
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from cv_bridge import CvBridge     

import cv2
import numpy as np
import time
import math


class EMAPoseFilter:
    def __init__(self, alpha=0.35):
        self.alpha = alpha
        self.filtered_pos = None

    def reset(self):
        self.filtered_pos = None

    def update(self, new_pos):
        if self.filtered_pos is None:
            self.filtered_pos = new_pos
        else:
            self.filtered_pos = self.alpha * new_pos + (1.0 - self.alpha) * self.filtered_pos
        return self.filtered_pos


class ArduPilotLandingNode(Node):

    # =========================================================================
    # ⚙ USER CONFIGURATION ZONE ⚙
    # =========================================================================
    
    ARUCO_DICT = cv2.aruco.DICT_5X5_50       
    MARKER_SIZE_M = 0.40                    

    KP_XY = 0.3         
    KD_XY = 0.2        
    
    MAX_VEL_XY = 0.4                          
    DESCENT_VEL = -0.80                      
    
    ACCEPT_ANGLE_DEG = 15.0                  
    
    NATIVE_LAND_ALT_M = 0.20                 
    TARGET_TIMEOUT_SEC = 1.0  

    # Initial Search Parameters               
    SEARCH_VEL = 0.4
    SEARCH_BASE_DUR_SEC = 2.0

    CAM_FX = 1238.69507
    CAM_FY = 1240.10018
    CAM_CX = 676.256475
    CAM_CY = 280.049937

    # =========================================================================
    
    def __init__(self):
        super().__init__('ardupilot_cv_landing')

        self.x_pos = self.y_pos = self.z_pos = self.yaw = 0.0
        self.pose_filter = EMAPoseFilter(alpha=0.35)

        self.current_mode = ""
        self.flight_state = "INITIAL SEARCH"
        self.target_v_x = self.target_v_y = self.target_v_z = 0.0
        
        self.smooth_v_x = 0.0
        self.smooth_v_y = 0.0
        self.vel_alpha = 0.15  

        self.landing_triggered = False
        self.last_mode_request_time = 0.0
        self.initial_guided_requested = False
        
        self.last_target_time = 0.0          
        self.has_seen_target = False         
        
        # --- Search & Memory Variables ---
        self.marker_world_x = 0.0
        self.marker_world_y = 0.0
        self.dist_to_memory = 0.0 
        
        self.search_leg = 0
        self.search_leg_start_time = time.monotonic()

        self.prev_error_forward = 0.0
        self.prev_error_left = 0.0
        self.off_center_angle_deg = 0.0 
        self.last_calc_time = time.monotonic()

        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,  
            depth=5
        )
        
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pos_cb, qos)
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(
            Image,  
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image',  
            self.image_cb,  
            qos
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_dict_obj = cv2.aruco.Dictionary_get(self.ARUCO_DICT)
        
        self.camera_matrix = np.array([
            [self.CAM_FX, 0.0,         self.CAM_CX],
            [0.0,         self.CAM_FY, self.CAM_CY],
            [0.0,         0.0,         1.0]
        ], dtype=np.float64)

        self.dist_coeffs = np.array([
            [0.09212235, -0.36553366, -0.01661773, 0.010594, 0.21629396]
        ], dtype=np.float64)

        self.get_logger().info('Initializing Tactical Landing Logic (Hybrid Search + Memory)...')
        self.control_timer = self.create_timer(1.0 / 20.0, self.mavros_control_loop)

    def state_cb(self, msg):
        self.current_mode = msg.mode

    def pos_cb(self, msg):
        self.x_pos = msg.pose.position.x
        self.y_pos = msg.pose.position.y
        self.z_pos = msg.pose.position.z
        
        # Extract Yaw from Quaternion
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def request_flight_mode(self, mode_string, force=False):
        if not force and (time.monotonic() - self.last_mode_request_time < 2.0):
            return   
            
        self.last_mode_request_time = time.monotonic()
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = mode_string
            self.set_mode_client.call_async(req)
            self.get_logger().info(f">>> Requesting ArduPilot Mode: {mode_string} <<<")

    def image_cb(self, msg):
        if self.flight_state == "TOUCHDOWN":
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
        
        orig_h, orig_w = frame.shape[:2]
        processing_w, processing_h = 640, 480
        small_frame = cv2.resize(frame, (processing_w, processing_h), interpolation=cv2.INTER_LINEAR)
        
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        
        corners, ids, _ = cv2.aruco.detectMarkers(blurred, self.aruco_dict_obj, parameters=self.aruco_params)

        target_detected = False

        if ids is not None and len(ids) > 0:
            i = 0  
            scale_x = orig_w / processing_w
            scale_y = orig_h / processing_h
            
            up_corners = corners[i].copy()
            up_corners[0][:, 0] *= scale_x
            up_corners[0][:, 1] *= scale_y
            
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [up_corners], self.MARKER_SIZE_M, self.camera_matrix, self.dist_coeffs)
            
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]
            
            target_detected = True
            self.has_seen_target = True
            self.last_target_time = time.monotonic()  
            
            self._calculate_landing_kinematics(rvec, tvec)

            cv2.aruco.drawDetectedMarkers(frame, [up_corners])
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.MARKER_SIZE_M / 2)

        if not target_detected:
            time_since_last_seen = time.monotonic() - self.last_target_time
            if time_since_last_seen > self.TARGET_TIMEOUT_SEC:
                self._handle_lost_target()
            else:
                self.flight_state = "COASTING"
                self.target_v_x = self.target_v_y = self.target_v_z = 0.0

        self._draw_hud(frame, target_detected)

        cv2.imshow("ArUco Targeting Camera", frame)
        cv2.waitKey(1)

        self.get_logger().info(
            f"STATE: {self.flight_state} | "
            f"CMD Vel: ({self.target_v_x:.2f}, {self.target_v_y:.2f}, {self.target_v_z:.2f}) | "
            f"Alt: {self.z_pos:.2f}m",  
            throttle_duration_sec=0.5
        )

    def _draw_hud(self, frame, target_detected):
        color_green = (0, 255, 0)
        color_red = (0, 0, 255)
        color_cyan = (255, 255, 0)
        color_orange = (0, 165, 255)
        
        state_color = color_green if target_detected else color_red
        
        cv2.putText(frame, f"STATE: {self.flight_state}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2)
        cv2.putText(frame, f"MODE:  {self.current_mode}", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_cyan, 2)
        
        if self.has_seen_target:
            cv2.putText(frame, f"Err X: {self.prev_error_forward:.2f}m", (20, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_green, 2)
            cv2.putText(frame, f"Err Y: {self.prev_error_left:.2f}m", (20, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_green, 2)
            ang_color = color_green if self.off_center_angle_deg <= self.ACCEPT_ANGLE_DEG else color_red
            cv2.putText(frame, f"Angle: {self.off_center_angle_deg:.1f} deg", (20, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.6, ang_color, 2)

        if self.flight_state in ["RECOVERY NAV", "MEMORY DESCENT"]:
             cv2.putText(frame, f"Dist to Marker Memory: {self.dist_to_memory:.2f}m", (20, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_orange, 2)

        cv2.putText(frame, f"Alt: {self.z_pos:.2f}m", (20, frame.shape[0] - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_cyan, 2)

    def _calculate_landing_kinematics(self, rvec, tvec):
        tvec = tvec.reshape(3, 1)
        rvec = rvec.reshape(3, 1)

        R, _ = cv2.Rodrigues(rvec)
        cam_pos_in_marker = -np.dot(R.T, tvec)
        smooth_pos = self.pose_filter.update(cam_pos_in_marker)

        # Body frame distance TO the marker
        error_forward = smooth_pos[1][0]   
        error_left = smooth_pos[0][0]      
        
        # --- Save true marker world coordinates ---
        marker_world_dx = (error_forward * math.cos(self.yaw)) - (error_left * math.sin(self.yaw))
        marker_world_dy = (error_forward * math.sin(self.yaw)) + (error_left * math.cos(self.yaw))
        self.marker_world_x = self.x_pos + marker_world_dx
        self.marker_world_y = self.y_pos + marker_world_dy

        cam_z = smooth_pos[2][0]           
        if cam_z > 5.0 and self.z_pos > 0.5:
            cam_z = self.z_pos

        current_time = time.monotonic()
        dt = current_time - self.last_calc_time
        
        if dt > 0.5 or dt <= 0.0:
            derivative_forward = derivative_left = 0.0
        else:
            derivative_forward = (error_forward - self.prev_error_forward) / dt
            derivative_left = (error_left - self.prev_error_left) / dt

        self.prev_error_forward = error_forward
        self.prev_error_left = error_left
        self.last_calc_time = current_time

        v_x_raw = (error_forward * self.KP_XY) + (derivative_forward * self.KD_XY)
        v_y_raw = (error_left * self.KP_XY) + (derivative_left * self.KD_XY)

        v_x = np.clip(v_x_raw, -self.MAX_VEL_XY, self.MAX_VEL_XY)
        v_y = np.clip(v_y_raw, -self.MAX_VEL_XY, self.MAX_VEL_XY)

        xy_error_mag = math.hypot(error_forward, error_left)
        self.off_center_angle_deg = math.degrees(math.atan2(xy_error_mag, cam_z))

        self.target_v_x = v_x
        self.target_v_y = v_y

        if self.off_center_angle_deg <= self.ACCEPT_ANGLE_DEG:
            self.flight_state = "TACTICAL DESCENT"
            self.target_v_z = self.DESCENT_VEL  
        else:
            self.flight_state = "CENTERING"
            self.target_v_z = 0.0  

    def _handle_lost_target(self):
        # =========================================================
        # PHASE 1: Never seen the target -> Expanding Square Search
        # =========================================================
        if not self.has_seen_target:
            if self.flight_state != "INITIAL SEARCH":
                self.flight_state = "INITIAL SEARCH"
                self.search_leg = 0
                self.search_leg_start_time = time.monotonic()

            multiplier = (self.search_leg // 2) + 1
            active_search_duration = self.SEARCH_BASE_DUR_SEC * multiplier

            elapsed_time = time.monotonic() - self.search_leg_start_time
            if elapsed_time > active_search_duration:
                self.search_leg += 1  
                self.search_leg_start_time = time.monotonic()

            direction = self.search_leg % 4
            if direction == 0:    self.target_v_x, self.target_v_y = self.SEARCH_VEL, 0.0
            elif direction == 1:  self.target_v_x, self.target_v_y = 0.0, -self.SEARCH_VEL
            elif direction == 2:  self.target_v_x, self.target_v_y = -self.SEARCH_VEL, 0.0
            elif direction == 3:  self.target_v_x, self.target_v_y = 0.0, self.SEARCH_VEL
            self.target_v_z = 0.0
            return

        # =========================================================
        # PHASE 3: Seen target but lost it -> Memory Recovery
        # =========================================================
        self.dist_to_memory = math.hypot(self.marker_world_x - self.x_pos, self.marker_world_y - self.y_pos)

        # If far away from the marker's physical location, fly straight back to it
        if self.dist_to_memory > 0.30:
            self.flight_state = "RECOVERY NAV"
            
            dx = self.marker_world_x - self.x_pos
            dy = self.marker_world_y - self.y_pos
            
            err_forward = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
            err_left    = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
            
            nav_kp = 0.5
            self.target_v_x = np.clip(err_forward * nav_kp, -self.MAX_VEL_XY, self.MAX_VEL_XY)
            self.target_v_y = np.clip(err_left * nav_kp, -self.MAX_VEL_XY, self.MAX_VEL_XY)
            self.target_v_z = 0.0
            
        # We arrived perfectly above the last known marker coordinates -> Drop down
        else:
            self.flight_state = "MEMORY DESCENT"
            self.target_v_x = 0.0
            self.target_v_y = 0.0
            self.target_v_z = self.DESCENT_VEL

    def mavros_control_loop(self):
        # --- Global Touchdown Failsafe ---
        if self.flight_state != "TOUCHDOWN" and 0.1 < self.z_pos < self.NATIVE_LAND_ALT_M:
            self.get_logger().info("Altitude threshold breached! Committing to TOUCHDOWN.")
            self.flight_state = "TOUCHDOWN"
            self.target_v_x = self.target_v_y = self.target_v_z = 0.0

        if self.flight_state == "TOUCHDOWN":
            if self.current_mode != "LAND":
                self.request_flight_mode("LAND", force=True)  
            return  

        if not self.initial_guided_requested and self.current_mode != "":
            if self.current_mode != "GUIDED":
                self.request_flight_mode("GUIDED", force=True)
            self.initial_guided_requested = True
            
        self.smooth_v_x = (self.vel_alpha * self.target_v_x) + ((1.0 - self.vel_alpha) * self.smooth_v_x)
        self.smooth_v_y = (self.vel_alpha * self.target_v_y) + ((1.0 - self.vel_alpha) * self.smooth_v_y)
        
        twist_msg = Twist()
        twist_msg.linear.x = float(self.smooth_v_x)
        twist_msg.linear.y = float(self.smooth_v_y)
        twist_msg.linear.z = float(self.target_v_z) 
        
        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArduPilotLandingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            cv2.destroyAllWindows()
            if rclpy.ok():
                try: node.cmd_vel_pub.publish(Twist())  
                except Exception: pass
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
