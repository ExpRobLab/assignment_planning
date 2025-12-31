#include <memory>
#include <fstream>
#include <vector>
#include <map>
#include <regex>
#include <string>

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

class NavigateAndRotateAction : public plansys2::ActionExecutorClient
{
public:
  NavigateAndRotateAction()
  : plansys2::ActionExecutorClient("navigate_and_rotate", 500ms)
  {
    // Setup Action Clients
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    scan_client_ = rclcpp_action::create_client<robot_manager::action::Scan>(this, "scan_environment");

    // Setup Aruco Listener
    aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
      "/aruco_detections", 10,
      std::bind(&NavigateAndRotateAction::detection_callback, this, std::placeholders::_1));

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Default waypoints (same ones you had inside ExploreAction)
    waypoint_map_["wp1"] = {-6.0, -7.0};
    waypoint_map_["wp2"] = {-4.0,  6.0};
    waypoint_map_["wp3"] = { 7.0, -7.0};
    waypoint_map_["wp4"] = { 7.0,  6.0};
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    step_ = 0;
    navigation_goal_sent_ = false;
    navigation_done_ = false;
    scan_goal_sent_ = false;
    scan_done_ = false;

    // Resolve waypoint from planner arguments
    auto args = get_arguments();
    if (args.size() >= 2) {
      target_wp_ = args[1];
    } else if (!args.empty()) {
      target_wp_ = args.back();
    } else {
      target_wp_.clear();
    }

    target_ok_ = resolve_waypoint_(target_wp_, target_x_, target_y_);

    // Merge any previously saved detections (so multiple navigate_and_rotate calls accumulate)
    load_markers_from_file_("/tmp/detected_markers.csv");

    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void do_work()
  {
    if (!target_ok_) {
      finish(false, 0.0, "navigate_and_rotate: could not resolve waypoint to (x,y)");
      return;
    }

    if (step_ == 0) { // Navigate
      if (!navigation_goal_sent_) {
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = now();
        goal.pose.pose.position.x = target_x_;
        goal.pose.pose.position.y = target_y_;
        goal.pose.pose.orientation.w = 1.0;

        auto send_goal_options =
          rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](auto) { navigation_done_ = true; };

        nav_client_->async_send_goal(goal, send_goal_options);
        navigation_goal_sent_ = true;
        navigation_done_ = false;

        RCLCPP_INFO(get_logger(), "navigate_and_rotate: going to %s (x=%.3f, y=%.3f)",
                    target_wp_.c_str(), target_x_, target_y_);
      }

      if (navigation_done_) {
        navigation_goal_sent_ = false;
        step_ = 1;
      }
    }
    else if (step_ == 1) { // Rotate/Scan
      if (!scan_goal_sent_) {
        auto goal = robot_manager::action::Scan::Goal();
        goal.target_angle = 6.28;

        auto opts = rclcpp_action::Client<robot_manager::action::Scan>::SendGoalOptions();
        opts.result_callback = [this](auto) { scan_done_ = true; };

        scan_client_->async_send_goal(goal, opts);
        scan_goal_sent_ = true;
        scan_done_ = false;

        RCLCPP_INFO(get_logger(), "navigate_and_rotate: scanning...");
      }

      if (scan_done_) {
        scan_goal_sent_ = false;

        // Persist detections for the capture action(s)
        save_markers_to_file_("/tmp/detected_markers.csv");

        finish(true, 1.0, "navigate_and_rotate complete");
      }
    }
  }

private:
  bool resolve_waypoint_(const std::string & wp, double & x, double & y)
  {
    auto it = waypoint_map_.find(wp);
    if (it != waypoint_map_.end()) {
      x = it->second.first;
      y = it->second.second;
      return true;
    }

    std::regex num_re(R"((-?\d+(?:\.\d+)?))");
    std::sregex_iterator begin(wp.begin(), wp.end(), num_re), end;
    std::vector<double> nums;
    for (auto i = begin; i != end; ++i) {
      try {
        nums.push_back(std::stod((*i)[1].str()));
      } catch (...) {}
    }

    if (nums.size() >= 2) {
      x = nums[0];
      y = nums[1];
      return true;
    }

    return false;
  }

  void load_markers_from_file_(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      return;
    }

    long id;
    char comma;
    double x, y;
    while (file >> id >> comma >> x >> comma >> y) {
      detected_markers_[id] = {x, y};
    }
  }

  void save_markers_to_file_(const std::string & path)
  {
    std::ofstream file(path);
    if (!file.is_open()) {
      RCLCPP_WARN(get_logger(), "navigate_and_rotate: could not open %s for writing", path.c_str());
      return;
    }

    for (auto const & kv : detected_markers_) {
      file << kv.first << "," << kv.second.first << "," << kv.second.second << "\n";
    }
  }

  void detection_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
  {
    for (auto & marker : msg->markers) {
      if (detected_markers_.find(marker.marker_id) != detected_markers_.end()) {
        continue;
      }

      try {
        std::string frame_id = "marker_" + std::to_string(marker.marker_id);
        auto t = tf_buffer_->lookupTransform("map", frame_id, tf2::TimePointZero);

        detected_markers_[marker.marker_id] = {t.transform.translation.x, t.transform.translation.y};
        RCLCPP_INFO(get_logger(), "navigate_and_rotate: mapped marker %ld",
                    static_cast<long>(marker.marker_id));
      } catch (const tf2::TransformException & ex) {
        (void)ex;
      }
    }
  }

private:
  int step_ = 0;

  std::string target_wp_;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  bool target_ok_ = false;

  bool navigation_goal_sent_ = false;
  bool navigation_done_ = false;
  bool scan_goal_sent_ = false;
  bool scan_done_ = false;

  std::map<long, std::pair<double, double>> detected_markers_;
  std::map<std::string, std::pair<double, double>> waypoint_map_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<robot_manager::action::Scan>::SharedPtr scan_client_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateAndRotateAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "navigate_and_rotate"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
