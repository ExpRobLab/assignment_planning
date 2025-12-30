#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

import numpy as np
import cv2
import math
import os
import time

# ROS Messages
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from aruco_opencv_msgs.msg import ArucoDetection
from rclpy.qos import qos_profile_sensor_data

# ROS Action Interfaces
from nav2_msgs.action import NavigateToPose
from robot_manager.action import Scan, Center  # Assuming these match your primitives file

# TF
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_from_euler

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # --- Parameters ---
        self.declare_parameter('image_topic', '/camera/image')
        self.image_topic = self.get_parameter('image_topic').value

        # --- File System Setup for Images ---
        current_folder = os.path.dirname(os.path.abspath(__file__))
        workspace = os.path.abspath(os.path.join(current_folder, "..", "..", "..", ".."))
        self.dataset_path = os.path.join(workspace, "resources")
        os.makedirs(self.dataset_path, exist_ok=True)
        self.get_logger().info(f"Saving images to: {self.dataset_path}")

        # --- Data Structures ---
        self.waypoints = [
            {'x': -6.0, 'y': -7.0}, # marker 1
            {'x': -4.0, 'y': 6.0},  # marker 2
            {'x': 7.0, 'y': -7.0},  # marker 3
            {'x': 7.0, 'y': 6.0}    # marker 4
        ]

        # Dictionary to store found markers: {id: {'x': global_x, 'y': global_y}}
        self.detected_markers = {}
        self.last_image = None
        self.last_last_image = None 

        # --- TF Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscribers ---
        # 1. Image Subscriber (for saving photos)
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            1
        )
        # 2. ArUco Subscriber (for detection during scan)
        self.create_subscription(ArucoDetection, '/aruco_detections', self.detection_callback, 1)

        # --- Action Clients ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.scan_client = ActionClient(self, Scan, 'scan_environment')
        self.center_client = ActionClient(self, Center, 'center_on_marker')

        self.get_logger().info("Mission Controller Initialized. Waiting for Action Servers...")
        self.nav_client.wait_for_server()
        self.scan_client.wait_for_server()
        self.center_client.wait_for_server()
        self.get_logger().info("All Action Servers Ready.")

    # =========================================================================
    # Callbacks
    # =========================================================================
    def image_callback(self, msg):
        self.last_image = msg
        self.last_last_image = self.last_image

    def detection_callback(self, msg):
        """
        Runs continuously. If we see a marker we haven't mapped yet, 
        transform it to MAP frame and save it.
        """
        for marker in msg.markers:
            mid = marker.marker_id
            
            # If we already mapped this ID, skip (unless you want to average/refine)
            if mid in self.detected_markers:
                continue

            frame_id = f"marker_{mid}"
            try:
                # Look up transform from Map -> Marker
                # We use the latest available transform
                t = self.tf_buffer.lookup_transform(
                    'map',
                    frame_id,
                    rclpy.time.Time()
                )
                
                gx = t.transform.translation.x
                gy = t.transform.translation.y
                
                self.detected_markers[mid] = {'x': gx, 'y': gy}
                self.get_logger().info(f"Mapped Marker {mid} at global (x={gx:.2f}, y={gy:.2f})")

            except TransformException as ex:
                # Often happens if TF isn't ready yet or marker momentarily lost
                pass

    # =========================================================================
    # Action Wrappers (Helpers) 
    # =========================================================================
    def send_nav_goal(self, x, y):
        self.get_logger().info(f"Navigating to x:{x}, y:{y}...")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # goal.pose.pose.orientation.w = 1.0 # Default orientation
        
        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected.")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        self.get_logger().info(f"Navigated to x:{x}, y:{y}...")
        return True

    def perform_scan(self):
        self.get_logger().info("Starting 360 Scan...")
        goal = Scan.Goal()
        goal.target_angle = 2 * math.pi # 360 degrees
        
        send_future = self.scan_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def perform_center(self, marker_id):
        self.get_logger().info(f"Centering on Marker {marker_id}...")
        goal = Center.Goal()
        goal.marker_id = marker_id
        
        send_future = self.center_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"Center goal for {marker_id} rejected.")
            return False
            
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result
        return result.success

    # =========================================================================
    # Image Saving Logic
    # =========================================================================
    def capture_and_save(self, marker_id):
        if self.last_image is None:
            self.get_logger().warn("No image received to save!")
            return -1

        try:
            # Convert ROS Image to OpenCV
            h = self.last_image.height
            w = self.last_image.width
            # Assuming "bgr8" or "rgb8", reshaping raw data
            # Note: For production code, use CvBridge. Here we stick to numpy as per your sample
            cv_image = np.frombuffer(self.last_image.data, dtype=np.uint8).reshape(h, w, 3)
            
            # Draw Circle
            center = (w // 2, h // 2)
            radius = min(h, w) // 8
            # Make a copy to avoid modifying the buffer directly if read-only
            img_copy = cv_image.copy() 
            cv2.circle(img_copy, center, radius, (0, 255, 0), 3)

            # Save
            filename = f"marker_{marker_id}.png"
            full_path = os.path.join(self.dataset_path, filename)
            cv2.imwrite(full_path, img_copy)
            self.get_logger().info(f"Saved picture to {full_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")
        
        return 0

    # =========================================================================
    # Main Logic
    # =========================================================================
    def run_mission(self):
        # --- Phase 1: Exploration ---
        self.get_logger().info("=== Phase 1: Exploration & Detection ===")

        self.get_logger().info(f"Initial rotation for no reason")
        self.perform_scan()

        if self.last_image is None:
            self.get_logger().warn("No image received to save!")
            return -1
        try:
            # Convert ROS Image to OpenCV
            h = self.last_image.height
            w = self.last_image.width
            # Assuming "bgr8" or "rgb8", reshaping raw data
            # Note: For production code, use CvBridge. Here we stick to numpy as per your sample
            cv_image = np.frombuffer(self.last_image.data, dtype=np.uint8).reshape(h, w, 3)
            
            # Draw Circle
            center = (w // 2, h // 2)
            radius = min(h, w) // 8
            # Make a copy to avoid modifying the buffer directly if read-only
            img_copy = cv_image.copy() 
            cv2.circle(img_copy, center, radius, (0, 255, 0), 3)

            # Save
            filename = f"marker_shit.png"
            full_path = os.path.join(self.dataset_path, filename)
            cv2.imwrite(full_path, img_copy)
            self.get_logger().info(f"Saved picture to {full_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")
        
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f"Going to Waypoint {i+1}/{len(self.waypoints)}")
            
            # 1. Navigate to Waypoint
            self.send_nav_goal(wp['x'], wp['y'])
            
            # 2. Perform 360 Scan (Detection runs in background callback)
            self.perform_scan()
            
            # Small pause to let TF buffers settle or detections finish
            time.sleep(0.5)

        self.get_logger().info(f"Exploration Done. Found {len(self.detected_markers)} markers.")
        self.get_logger().info(f"IDs: {list(self.detected_markers.keys())}")

        # --- Phase 2: Sequential Visiting ---
        self.get_logger().info("=== Phase 2: Visiting & Capturing ===")
        
        # Sort IDs low -> high
        sorted_ids = sorted(self.detected_markers.keys())
        
        for mid in sorted_ids:
            target_pos = self.detected_markers[mid]
            self.get_logger().info(f"Processing Marker {mid}...")

            # 1. Navigate to the marker (Global Position)
            # Note: NavigateToPose might fail if the goal is exactly inside the wall/marker.
            # In a real scenario, we'd calculate an approach pose. 
            # For this assignment scope, we assume Nav2 handles "getting close".
            self.send_nav_goal(target_pos['x'], target_pos['y'])

            # 2. Center on the marker (Visual Servoing)
            success = self.perform_center(mid)
            
            # 3. Take Picture if centering was successful (or even if not, we try)
            if success:
                # Wait a split second for camera blur to settle
                # time.sleep(0.5) 
                for _ in range(5):
                    ret = self.capture_and_save(mid)
                    if ret == 0:
                        break
            else:
                self.get_logger().warn(f"Could not center on Marker {mid}. Skipping photo.")

        self.get_logger().info("=== Mission Complete ===")


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    
    try:
        # We run the mission in a separate logic flow, 
        # but we need spin() to keep callbacks (TF, Subs) alive.
        # A simple way is to use a MultiThreadedExecutor or just run mission inside the node.
        # Since run_mission is blocking (uses spin_until_future), we run it directly.
        node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()