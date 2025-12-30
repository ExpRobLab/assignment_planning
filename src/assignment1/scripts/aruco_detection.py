#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection
from sensor_msgs.msg import CompressedImage, Image
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException, LookupException, ConnectivityException, ExtrapolationException

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

VERBOSE = False


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('base_frame', 'base_footprint')

        image_topic = self.get_parameter('image_topic').value
        self.base_frame = self.get_parameter('base_frame').value

        # Path of the current script
        current_folder = os.path.dirname(os.path.abspath(__file__))
        # Go to workspace root
        workspace = os.path.abspath(os.path.join(current_folder, "..", "..","..",".."))
        # Join with a folder at workspace level
        self.dataset_path = os.path.join(workspace, "resources")

        # print("Current folder:", current_folder)
        # print("Workspace:", workspace)
        print("Dataset path:", self.dataset_path)

        os.makedirs(self.dataset_path, exist_ok=True)

        # Listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.subscriber = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.__detection_callback,
            1
        )
        self.detected_markers: dict = {}

        # Publishers cmd velocity
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.timer = self.create_timer(0.01, self.__control)

        # Market detection
        self.state = "detecting"
        self.num = 0
        
        # angular control params
        self.treshold = 0.01 
        self.Kp = .5
        self.max_angular = 1.0

        # marker tracking
        self.detected_ids = set()
        self.sorted_ids = []
        self.target_marker_id = None    

        # command to robot
        self.robot_cmd_vel: Twist = Twist()
        self.robot_cmd_vel.linear.x = 0.0
        self.robot_cmd_vel.angular.z = 0.5

        # Image handling: subscribe to camera and publish final frame
        self.last_image_msg: Image = Image()
        self.final_image_published = False

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.__image_callback,
            1
        )

        self.final_image_pub = self.create_publisher(
            Image,
            '/final_marker_image',
            1
        )

        self.local_z = None

        self.aruco_locations = [{'x': -6, 'y': -6}, {'x': -6, 'y': 6}, {'x': 6, 'y': -6}, {'x': 6, 'y': 6}]

    def send_goal(self, x, y, theta=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  # global frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
            
    def __detection_callback(self, msg):
        for marker in msg.markers:
            if self.target_marker_id is not None:
                if self.target_marker_id == marker.marker_id:
                    self.local_z    = marker.pose.position.x

        if self.state != "detecting":
            return
        
        # spin while detecting
        # self.vel_pub.publish(self.robot_cmd_vel)

        for marker in msg.markers:
            frame_id = f"marker_{marker.marker_id}"
            
            try:
                odom_T_arucomarker = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    frame_id,
                    rclpy.time.Time()
                )

                # store transform and numeric ID
                self.detected_markers[frame_id] = odom_T_arucomarker
                self.detected_ids.add(marker.marker_id)
                self.get_logger().info(f"state {self.state}, {frame_id} Detected here, {len(self.detected_ids)}, {self.detected_ids}, localopt {self.local_z} ")

                if len(self.detected_ids) == len(self.aruco_locations) and self.state == "detecting":
                    self.sorted_ids = sorted(list(self.detected_ids))
                    self.target_marker_id = self.sorted_ids[self.num]
                    self.state = "centering"
                    self.get_logger().info(
                        f"Detected 5 markers. IDs: {self.sorted_ids}. "
                        f"Target is marker_{self.target_marker_id}"
                    )

                if frame_id not in self.detected_markers:
                    self.get_logger().info(f"Detected new marker: {frame_id}")

            except TransformException:
                self.get_logger().info("ERROR")
                continue

    def __control(self):
        if self.state in ["done"]:
            return
        
        if self.state == 'detecting':
            self.robot_cmd_vel: Twist = Twist()
            self.robot_cmd_vel.linear.x = 0.0
            self.robot_cmd_vel.angular.z = 0.5
            self.vel_pub.publish(self.robot_cmd_vel)

        if self.target_marker_id is None:
            return

        frame_id = f"marker_{self.target_marker_id}"

        try:
            base_T_marker = self.tf_buffer.lookup_transform(
                self.base_frame,
                frame_id,
                rclpy.time.Time()
            )
        except TransformException as e :
           base_T_marker = self.detected_markers[frame_id]

        X = base_T_marker.transform.translation.x
        Y = base_T_marker.transform.translation.y
        angle_to_marker = math.atan2(Y, X)
        self.robot_cmd_vel.linear.x = 0.0

        # P-controller on angle in robot frame
        angular_cmd = self.Kp * angle_to_marker if self.state == "centering" else self.Kp * -self.local_z  # local optimization (visual servoing)
        angular_cmd = max(-self.max_angular, min(self.max_angular, angular_cmd))
        self.robot_cmd_vel.angular.z = angular_cmd
        self.vel_pub.publish(self.robot_cmd_vel)

        # mode 1 for switch: go to global optima until you see the target, then switch to visual servoing
        if self.local_z is not None and self.state == "centering":
            self.state = "localopt"

        # mode 2 for switch: go to global optima and switch to local optimization
        # if abs(angle_to_marker) < self.treshold and self.state == "centering":
        #     # change to local optimization (visual servoing)
        #     self.state = "localopt"

        self.get_logger().info(f"state {self.state}, {frame_id} target, {len(self.detected_ids)},angletomarker {round(angle_to_marker, 5)}  angularcmd {round(angular_cmd, 5)} localz {self.local_z} ")

        if self.local_z is None and self.state == "done":
            return
        
        if self.state == "localopt" and abs(self.local_z) < self.treshold:
            self.robot_cmd_vel.angular.z = 0.0
            self.vel_pub.publish(self.robot_cmd_vel)

            # process and save/publish the final frame for THIS marker
            if (not self.final_image_published) and (self.last_image_msg is not None):
                try:
                    # decode JPEG/PNG from CompressedImage.data
                    height = self.last_image_msg.height
                    width = self.last_image_msg.width
                    channels = 3  # oppure 1 se mono

                    cv_image = np.frombuffer(self.last_image_msg.data, dtype=np.uint8).reshape(height, width, channels)

                    cv_image = cv_image.reshape((height, width, 3))

                    if cv_image is None:
                        self.get_logger().warn("Could not decode compressed image, nothing to save.")
                    else:
                        h, w = cv_image.shape[:2]
                        center = (w // 2, h // 2)
                        radius = min(h, w) // 8
                        cv2.circle(cv_image, center, radius, (0, 0, 255), 3)
                        save_path = f"{self.dataset_path}/marker_{self.target_marker_id}.png"
                        success = cv2.imwrite(save_path, cv_image)
                        print("Write success:", success)
                        img_msg = Image()
                        img_msg.header = self.last_image_msg.header
                        img_msg.height, img_msg.width = cv_image.shape[:2]
                        img_msg.encoding = 'bgr8'
                        img_msg.is_bigendian = 0
                        img_msg.step = img_msg.width * 3
                        img_msg.data = cv_image.tobytes()
                        self.final_image_pub.publish(img_msg)
                        self.final_image_published = True
                        self.get_logger().info(
                            f"Centered on marker_{self.target_marker_id}. "
                            f"Drew circle, saved to {save_path} and published on /final_marker_image."
                        )

                except Exception as e:
                    self.get_logger().warn(f"Error converting/saving/publishing image: {e}")

            elif self.last_image_msg is None:
                self.get_logger().warn(
                    "Centered on marker but no image received yet, nothing to save."
                )

            # decide whether to move to next marker or finish
            if self.num < len(self.sorted_ids) - 1:
                self.num = self.num + 1
                self.target_marker_id = self.sorted_ids[self.num]
                self.state = "centering"
                self.local_z = None
                self.final_image_published = False
                self.get_logger().info(
                    f"Moving to next target marker_{self.target_marker_id}"
                )
            else:
                self.state = "done"
                self.local_z = None
                self.get_logger().info("All markers processed, state = done.")
                self.send_goal(1.0, 1.0)

    def __image_callback(self, msg: Image):
        # save last frame
        if self.state in ["detecting", "centering", "localopt"]:
            self.last_image_msg = msg

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Shutting down ImageFeature node")
    finally:
        if node is not None:
            node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
