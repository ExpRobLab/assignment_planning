#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import math
import time
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion

from robot_manager.action import Scan
from robot_manager.action import Center 


class ActionPrimitivesServer(Node):
    def __init__(self):
        super().__init__('detection_primitives')

        # --- Parameters ---
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('scan_speed', 0.5)    # rad/s
        self.declare_parameter('center_kp', 0.5)     # Proportional gain
        self.declare_parameter('center_threshold', 0.02) # radians error to stop

        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.freq = self.get_parameter('control_frequency').value
        self.scan_speed = self.get_parameter('scan_speed').value
        self.kp = self.get_parameter('center_kp').value
        self.threshold = self.get_parameter('center_threshold').value

        # --- TF Listeners ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publisher ---
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Action Servers ---
        self._scan_server = ActionServer(
            self,
            Scan,
            'scan_environment',
            execute_callback=self.execute_scan_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self._center_server = ActionServer(
            self,
            Center,
            'center_on_marker',
            execute_callback=self.execute_center_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info("Action Primitives Server Ready (Scan & Center)")

    # =========================================================================
    # PRIMITIVE 2: Rotate Aimlessly (Scan)
    # =========================================================================
    def execute_scan_callback(self, goal_handle):
        self.get_logger().info(f"Executing Scan... Goal: {goal_handle.request.target_angle} rad")
        
        target_angle = goal_handle.request.target_angle
        # Default to 360 degrees (2*PI) if 0 is passed
        if target_angle == 0.0:
            target_angle = 2 * math.pi

        feedback_msg = Scan.Feedback()
        result_msg = Scan.Result()

        # Get initial yaw
        initial_yaw = self.get_yaw(self.odom_frame, self.base_frame)
        if initial_yaw is None:
            self.get_logger().error("Could not get initial TF for Scan.")
            goal_handle.abort()
            result_msg.success = False
            return result_msg

        current_angle_traveled = 0.0
        previous_yaw = initial_yaw
        
        rate = self.create_rate(self.freq)

        while rclpy.ok() and current_angle_traveled < target_angle:
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                self.get_logger().info("Scan canceled.")
                result_msg.success = False
                return result_msg

            # Command Rotation
            cmd = Twist()
            cmd.angular.z = self.scan_speed
            self.vel_pub.publish(cmd)

            # Update Odometry
            current_yaw = self.get_yaw(self.odom_frame, self.base_frame)
            if current_yaw is not None:
                # Calculate diff ensuring we handle wrap-around (-pi to pi)
                delta = current_yaw - previous_yaw
                # Normalize delta
                while delta < -math.pi: delta += 2*math.pi
                while delta > math.pi: delta -= 2*math.pi
                
                current_angle_traveled += abs(delta)
                previous_yaw = current_yaw

            feedback_msg.current_angle = current_angle_traveled
            goal_handle.publish_feedback(feedback_msg)
            
            rate.sleep()

        self.stop_robot()
        goal_handle.succeed()
        result_msg.success = True
        self.get_logger().info("Scan Complete.")
        return result_msg

    # =========================================================================
    # PRIMITIVE 3: Rotate to Center on Sign
    # =========================================================================
    def execute_center_callback(self, goal_handle):
        marker_id = goal_handle.request.marker_id
        frame_id = f"marker_{marker_id}"
        self.get_logger().info(f"Executing Center on {frame_id}...")

        feedback_msg = Center.Feedback()
        result_msg = Center.Result()
        
        rate = self.create_rate(self.freq)
        
        # Timeout variables to prevent infinite searching if marker is lost
        lost_marker_start = None
        lost_marker_timeout = 5.0 # seconds

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result_msg.success = False
                return result_msg

            # 1. Look up Transform (Robot -> Marker)
            try:
                # Use time 0 to get latest available transform
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    frame_id,
                    rclpy.time.Time()
                )
                lost_marker_start = None # Reset timeout if found
                
            except TransformException:
                # Handle temporary loss of sight
                if lost_marker_start is None:
                    lost_marker_start = self.get_clock().now()
                
                # Check timeout
                time_lost = (self.get_clock().now() - lost_marker_start).nanoseconds / 1e9
                if time_lost > lost_marker_timeout:
                    self.stop_robot()
                    self.get_logger().error(f"Marker {marker_id} lost for too long. Aborting.")
                    goal_handle.abort()
                    result_msg.success = False
                    return result_msg
                
                rate.sleep()
                continue

            # 2. Calculate Error (Visual Servoing)
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # Angle to marker in robot frame
            angle_error = math.atan2(y, x)
            
            feedback_msg.error_angle = angle_error
            goal_handle.publish_feedback(feedback_msg)

            # 3. Check Success Condition
            if abs(angle_error) < self.threshold:
                self.stop_robot()
                goal_handle.succeed()
                result_msg.success = True
                self.get_logger().info(f"Centered on {frame_id} successfully.")
                return result_msg

            # 4. Control Logic (P-Controller)
            cmd = Twist()
            cmd.angular.z = self.kp * angle_error
            
            # Clamp angular velocity
            max_ang = 1.0
            cmd.angular.z = max(-max_ang, min(max_ang, cmd.angular.z))
            
            self.vel_pub.publish(cmd)
            rate.sleep()

    # =========================================================================
    # Helpers
    # =========================================================================
    def get_yaw(self, parent, child):
        try:
            t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return yaw
        except TransformException:
            return None

    def stop_robot(self):
        cmd = Twist()
        self.vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    # Use MultiThreadedExecutor to allow Actions and TF listeners to run concurrently
    node = ActionPrimitivesServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()