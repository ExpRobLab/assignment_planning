#include <memory>
#include <fstream>
#include <vector>
#include <map>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "robot_manager/action/scan.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class ExploreAction : public plansys2::ActionExecutorClient
{
public:
  ExploreAction()
  : plansys2::ActionExecutorClient("explore", 500ms)
  {
    // Setup Action Clients
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    scan_client_ = rclcpp_action::create_client<robot_manager::action::Scan>(this, "scan_environment");
    
    // Setup Aruco Listener
    aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
      "/aruco_detections", 10, std::bind(&ExploreAction::detection_callback, this, std::placeholders::_1));

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Waypoints 
    waypoints_ = {{-6.0, -7.0}, {-4.0, 6.0}, {7.0, -7.0}, {7.0, 6.0}};
    
    current_wp_idx_ = 0;
  }

  void detection_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
    for (auto & marker : msg->markers) {
      if (detected_markers_.find(marker.marker_id) == detected_markers_.end()) {
        try {
            std::string frame_id = "marker_" + std::to_string(marker.marker_id);
            
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
              "map", frame_id, tf2::TimePointZero);
            
            detected_markers_[marker.marker_id] = {t.transform.translation.x, t.transform.translation.y};
            
            RCLCPP_INFO(get_logger(), "Mapped Marker %ld", (long)marker.marker_id);
        } catch (const tf2::TransformException & ex) {}
      }
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    current_wp_idx_ = 0;
    step_ = 0; // 0: Nav, 1: Scan
    detected_markers_.clear();
    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void do_work()
  {
    if ((size_t)current_wp_idx_ >= waypoints_.size()) {
      // Save markers to file for the next action
      std::ofstream file("/tmp/detected_markers.csv");
      for(auto const& [id, pos] : detected_markers_) {
          file << id << "," << pos.first << "," << pos.second << "\n";
      }
      file.close();
      finish(true, 1.0, "Exploration Complete");
      return;
    }

    if (step_ == 0) { // Navigate
        if (!navigation_goal_sent_) {
            auto goal = nav2_msgs::action::NavigateToPose::Goal();
            goal.pose.header.frame_id = "map";
            goal.pose.header.stamp = now();
            goal.pose.pose.position.x = waypoints_[current_wp_idx_].first;
            goal.pose.pose.position.y = waypoints_[current_wp_idx_].second;
            goal.pose.pose.orientation.w = 1.0;
            
            auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = [this](auto) { navigation_done_ = true; };
            nav_client_->async_send_goal(goal, send_goal_options);
            navigation_goal_sent_ = true;
            navigation_done_ = false;
            RCLCPP_INFO(get_logger(), "Going to WP %d", current_wp_idx_);
        }
        if (navigation_done_) {
            navigation_goal_sent_ = false;
            step_ = 1; // Move to Scan
        }
    } 
    else if (step_ == 1) { // Scan
        if (!scan_goal_sent_) {
             auto goal = robot_manager::action::Scan::Goal();
             goal.target_angle = 6.28;
             auto opts = rclcpp_action::Client<robot_manager::action::Scan>::SendGoalOptions();
             opts.result_callback = [this](auto) { scan_done_ = true; };
             scan_client_->async_send_goal(goal, opts);
             scan_goal_sent_ = true;
             scan_done_ = false;
             RCLCPP_INFO(get_logger(), "Scanning...");
        }
        if (scan_done_) {
            scan_goal_sent_ = false;
            current_wp_idx_++;      // Next WP
            step_ = 0;              // Back to Nav
        }
    }
  }

private:
  int step_ = 0;
  int current_wp_idx_ = 0;
  bool navigation_goal_sent_ = false;
  bool navigation_done_ = false;
  bool scan_goal_sent_ = false;
  bool scan_done_ = false;

  std::vector<std::pair<double, double>> waypoints_;
  std::map<long, std::pair<double, double>> detected_markers_;
  
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<robot_manager::action::Scan>::SharedPtr scan_client_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExploreAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "explore"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}