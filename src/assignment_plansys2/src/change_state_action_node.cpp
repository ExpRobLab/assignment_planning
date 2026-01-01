#include <memory>
#include <string>
#include <map>
#include <fstream>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class ChangeStateAction : public plansys2::ActionExecutorClient
{
public:
  ChangeStateAction()
  : plansys2::ActionExecutorClient("change_state", 500ms)
  {
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    marker_waypoints_["marker0"] = {0.0, 0.0};
    marker_waypoints_["marker1"] = {-6.0, -7.0};
    marker_waypoints_["marker2"] = {-4.0, 6.0};
    marker_waypoints_["marker3"] = {7.0, -7.0};
    marker_waypoints_["marker4"] = {7.0, 6.0};
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    step_ = 0;
    nav_goal_sent_ = false;
    nav_done_ = false;

    auto args = get_arguments();   // (change_state ?r ?to ?from)
    if (args.size() >= 3) {
      robot_ = args[0];
      to_marker_ = args[1];
      from_marker_ = args[2];
    } else {
      robot_.clear();
      to_marker_.clear();
      from_marker_.clear();
    }

    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void do_work()
  {
    if (to_marker_.empty()) {
      finish(false, 1.0, "Missing action parameters");
      return;
    }

    double tx = 0.0, ty = 0.0;
    if (!resolve_target_xy_(to_marker_, tx, ty)) {
      finish(false, 1.0, "Unknown target marker (no waypoint/pose)");
      return;
    }

    if (step_ == 0) {
      if (!nav_goal_sent_) {
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = now();
        goal.pose.pose.position.x = tx;
        goal.pose.pose.position.y = ty;
        goal.pose.pose.orientation.w = 1.0;

        auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        opts.result_callback = [this](auto) { nav_done_ = true; };

        nav_client_->async_send_goal(goal, opts);
        nav_goal_sent_ = true;
        nav_done_ = false;

        RCLCPP_INFO(get_logger(), "change_state: navigating %s -> %s",
          from_marker_.c_str(), to_marker_.c_str());
      }

      if (nav_done_) {
        nav_goal_sent_ = false;
        finish(true, 1.0, "State changed");
        return;
      }
    }
  }

private:
  bool resolve_target_xy_(const std::string & marker_name, double & x, double & y)
  {
    // 1) Prefer fixed waypoints
    auto it = marker_waypoints_.find(marker_name);
    if (it != marker_waypoints_.end()) {
      x = it->second.first;
      y = it->second.second;
      return true;
    }

    // 2) Fallback: if marker pose exists in /tmp/detected_markers.csv, use it
    long id = marker_name_to_id_(marker_name);
    if (id < 0) return false;

    std::ifstream file("/tmp/detected_markers.csv");
    if (!file.is_open()) return false;

    long rid; char comma; double rx, ry;
    while (file >> rid >> comma >> rx >> comma >> ry) {
      if (rid == id) {
        x = rx;
        y = ry;
        return true;
      }
    }
    return false;
  }

  static long marker_name_to_id_(const std::string & name)
  {
    if (name.rfind("marker", 0) != 0 || name.size() <= 6) return -1;
    try { return std::stol(name.substr(6)); } catch (...) { return -1; }
  }

private:
  int step_ = 0;

  bool nav_goal_sent_ = false;
  bool nav_done_ = false;

  std::string robot_;
  std::string to_marker_;
  std::string from_marker_;

  std::map<std::string, std::pair<double, double>> marker_waypoints_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChangeStateAction>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
