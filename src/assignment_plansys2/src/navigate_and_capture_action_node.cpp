#include <memory>
#include <fstream>
#include <map>
#include <vector>
#include <algorithm>
#include <regex>
#include <string>
#include <cmath>
#include <cstdlib>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "robot_manager/action/center.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

using namespace std::chrono_literals;

class NavigateAndCaptureAction : public plansys2::ActionExecutorClient
{
public:
  NavigateAndCaptureAction()
  : plansys2::ActionExecutorClient("navigate_and_capture", 500ms)
  {
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    center_client_ = rclcpp_action::create_client<robot_manager::action::Center>(this, "center_on_marker");

    auto qos = rclcpp::SensorDataQoS();
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { last_img_ = msg; });

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
    nav_goal_sent_ = false;
    nav_done_ = false;
    center_goal_sent_ = false;
    center_done_ = false;
    center_success_ = false;

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

    // Load detected markers (written by navigate_and_rotate)
    load_markers_from_file_("/tmp/detected_markers.csv");

    // Pick closest marker to the requested (x,y), so we can still "center_on_marker"
    has_marker_ = false;
    target_marker_id_ = -1;
    if (target_ok_ && !markers_.empty()) {
      pick_closest_marker_(target_x_, target_y_, 1.0 /* meters */);
    }

    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void do_work()
  {
    if (!target_ok_) {
      finish(false, 0.0, "navigate_and_capture: could not resolve waypoint to (x,y)");
      return;
    }

    if (step_ == 0) { // Navigate
      if (!nav_goal_sent_) {
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = now();
        goal.pose.pose.position.x = target_x_;
        goal.pose.pose.position.y = target_y_;
        goal.pose.pose.orientation.w = 1.0;

        auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        opts.result_callback = [this](auto) { nav_done_ = true; };

        nav_client_->async_send_goal(goal, opts);
        nav_goal_sent_ = true;
        nav_done_ = false;

        RCLCPP_INFO(get_logger(), "navigate_and_capture: going to %s (x=%.3f, y=%.3f)",
                    target_wp_.c_str(), target_x_, target_y_);
      }

      if (nav_done_) {
        nav_goal_sent_ = false;
        step_ = has_marker_ ? 1 : 2; // center only if we found a close marker
      }
    }
    else if (step_ == 1) { // Center on marker (optional)
      if (!center_goal_sent_) {
        auto goal = robot_manager::action::Center::Goal();
        goal.marker_id = target_marker_id_;

        auto opts = rclcpp_action::Client<robot_manager::action::Center>::SendGoalOptions();
        opts.result_callback = [this](auto r) {
          center_success_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED);
          center_done_ = true;
        };

        center_client_->async_send_goal(goal, opts);
        center_goal_sent_ = true;
        center_done_ = false;

        RCLCPP_INFO(get_logger(), "navigate_and_capture: centering on marker %ld", target_marker_id_);
      }

      if (center_done_) {
        center_goal_sent_ = false;
        step_ = 2;
      }
    }
    else if (step_ == 2) { // Save image
      if (!last_img_) {
        finish(false, 0.0, "navigate_and_capture: no camera image received yet");
        return;
      }

      try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_img_, sensor_msgs::image_encodings::BGR8);
        cv::circle(cv_ptr->image,
                   cv::Point(cv_ptr->image.cols / 2, cv_ptr->image.rows / 2),
                   50, CV_RGB(0, 255, 0), 3);

        const char * home = std::getenv("HOME");
        if (!home) {
          finish(false, 0.0, "navigate_and_capture: HOME env var not set");
          return;
        }

        std::string dir_path = std::string(home) + "/assignment1_bundle/resources/";
        std::filesystem::create_directories(dir_path);

        std::string file_path;
        if (has_marker_) {
          file_path = dir_path + "marker_" + std::to_string(target_marker_id_) + ".png";
        } else {
          file_path = dir_path + "capture_" + target_wp_ + ".png";
        }

        if (!cv::imwrite(file_path, cv_ptr->image)) {
          finish(false, 0.0, "navigate_and_capture: failed to write image");
          return;
        }

        RCLCPP_INFO(get_logger(), "navigate_and_capture: saved %s", file_path.c_str());
      } catch (const std::exception & e) {
        (void)e;
        finish(false, 0.0, "navigate_and_capture: exception while saving image");
        return;
      }

      finish(true, 1.0, "navigate_and_capture complete");
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
    markers_.clear();

    std::ifstream file(path);
    if (!file.is_open()) {
      return;
    }

    long id;
    char comma;
    double x, y;
    while (file >> id >> comma >> x >> comma >> y) {
      markers_.push_back({id, {x, y}});
    }

    std::sort(markers_.begin(), markers_.end());
  }

  void pick_closest_marker_(double x, double y, double max_dist)
  {
    double best_d = 1e9;
    long best_id = -1;

    for (auto const & m : markers_) {
      double dx = m.second.first - x;
      double dy = m.second.second - y;
      double d = std::sqrt(dx * dx + dy * dy);
      if (d < best_d) {
        best_d = d;
        best_id = m.first;
      }
    }

    if (best_id >= 0 && best_d <= max_dist) {
      has_marker_ = true;
      target_marker_id_ = best_id;
      RCLCPP_INFO(get_logger(), "navigate_and_capture: closest marker %ld (dist=%.3f)",
                  target_marker_id_, best_d);
    }
  }

private:
  int step_ = 0;

  std::string target_wp_;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  bool target_ok_ = false;

  bool has_marker_ = false;
  long target_marker_id_ = -1;

  bool nav_goal_sent_ = false;
  bool nav_done_ = false;
  bool center_goal_sent_ = false;
  bool center_done_ = false;
  bool center_success_ = false;

  std::vector<std::pair<long, std::pair<double, double>>> markers_;
  std::map<std::string, std::pair<double, double>> waypoint_map_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<robot_manager::action::Center>::SharedPtr center_client_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  sensor_msgs::msg::Image::SharedPtr last_img_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateAndCaptureAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "navigate_and_capture"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
