#include <memory>
#include <fstream>
#include <vector>
#include <map>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
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
  ExploreAction(const std::string & action_name)
  : plansys2::ActionExecutorClient(action_name, 500ms), current_action_(action_name)
  {
    // Setup Action Clients
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    scan_client_ = rclcpp_action::create_client<robot_manager::action::Scan>(this, "scan_environment");
    
    // Setup Aruco Listener
    aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
      "/aruco_detections", 10, std::bind(&ExploreAction::detection_callback, this, std::placeholders::_1));

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Marker -> waypoint mapping (tuned for your map)
    marker_waypoints_["marker0"] = {0.0, 0.0};
    marker_waypoints_["marker1"] = {-6.0, -7.0};
    marker_waypoints_["marker2"] = {-4.0, 6.0};
    marker_waypoints_["marker3"] = {7.0, -7.0};
    marker_waypoints_["marker4"] = {7.0, 6.0};
  }

  static long marker_name_to_id(const std::string & name)
  {
    // Expected format: marker<integer>
    if (name.rfind("marker", 0) != 0 || name.size() <= 6) {
      return -1;
    }
    try {
      return std::stol(name.substr(6));
    } catch (...) {
      return -1;
    }
  }

  void load_detected_markers_file()
  {
    std::ifstream file("/tmp/detected_markers.csv");
    if (!file.is_open()) {
      return;
    }
    long id; char comma; double x, y;
    while (file >> id >> comma >> x >> comma >> y) {
      detected_markers_[id] = {x, y};
    }
  }

  void save_detected_markers_file()
  {
    std::ofstream file("/tmp/detected_markers.csv");
    for (auto const & [id, pos] : detected_markers_) {
      file << id << "," << pos.first << "," << pos.second << "\n";
    }
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
    step_ = 0; // 0: Nav, 1: Scan
    navigation_goal_sent_ = false;
    navigation_done_ = false;
    scan_goal_sent_ = false;
    scan_done_ = false;
    detected_markers_.clear();
    load_detected_markers_file();

    // Cache current dispatch arguments
    auto args = get_arguments();
    if (args.size() >= 3) {
      robot_ = args[0];
      to_marker_ = args[1];
      from_marker_ = args[2];
    } else {
      robot_.clear();
      to_marker_.clear();
      from_marker_.clear();
    }
    expected_marker_id_ = marker_name_to_id(to_marker_);

    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void do_work()
  {
    // Sanity checks
    if (to_marker_.empty()) {
      finish(false, 1.0, "Missing action parameters");
      return;
    }
    if (marker_waypoints_.find(to_marker_) == marker_waypoints_.end()) {
      finish(false, 1.0, "Unknown target marker");
      return;
    }

    if (step_ == 0) { // Navigate to ?to
        if (!navigation_goal_sent_) {
            auto goal = nav2_msgs::action::NavigateToPose::Goal();
            goal.pose.header.frame_id = "map";
            goal.pose.header.stamp = now();
            goal.pose.pose.position.x = marker_waypoints_[to_marker_].first;
            goal.pose.pose.position.y = marker_waypoints_[to_marker_].second;
            goal.pose.pose.orientation.w = 1.0;
            
            auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = [this](auto) { navigation_done_ = true; };
            nav_client_->async_send_goal(goal, send_goal_options);
            navigation_goal_sent_ = true;
            navigation_done_ = false;
            RCLCPP_INFO(get_logger(), "Navigating to %s", to_marker_.c_str());
        }
        if (navigation_done_) {
            navigation_goal_sent_ = false;
            // change_state has no scanning; finish after reaching ?to
            if (current_action_ == "change_state") {
              finish(true, 1.0, "State changed");
              return;
            }
            step_ = 1; // Move to Scan (move_to_detect)
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

            // Force a final TF lookup for the expected marker id (if any)
            if (expected_marker_id_ >= 0) {
              try {
                std::string frame_id = "marker_" + std::to_string(expected_marker_id_);
                geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                  "map", frame_id, tf2::TimePointZero);
                detected_markers_[expected_marker_id_] = {t.transform.translation.x, t.transform.translation.y};
              } catch (const tf2::TransformException & ex) {
                (void)ex;
              }
            }

            save_detected_markers_file();

            // Only succeed move_to_detect if we actually detected the requested marker
            if (expected_marker_id_ >= 0 && detected_markers_.find(expected_marker_id_) == detected_markers_.end()) {
              finish(false, 1.0, "Marker not detected");
              return;
            }

            finish(true, 1.0, "Detection complete");
            return;
        }
    }
  }

private:
  int step_ = 0;
  bool navigation_goal_sent_ = false;
  bool navigation_done_ = false;
  bool scan_goal_sent_ = false;
  bool scan_done_ = false;

  std::string current_action_;
  std::string robot_;
  std::string to_marker_;
  std::string from_marker_;
  long expected_marker_id_ = -1;

  std::map<std::string, std::pair<double, double>> marker_waypoints_;
  std::map<long, std::pair<double, double>> detected_markers_;
  
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<robot_manager::action::Scan>::SharedPtr scan_client_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

static std::string parse_action_name(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if ((a == "--action-name" || a == "--action_name") && i + 1 < argc) {
      return std::string(argv[i + 1]);
    }
    if (a == "-p" && i + 1 < argc) {
      std::string p(argv[i + 1]);
      const std::string key = "action_name:=";
      if (p.rfind(key, 0) == 0) {
        return p.substr(key.size());
      }
    }
    const std::string key = "action_name:=";
    auto pos = a.find(key);
    if (pos != std::string::npos) {
      return a.substr(pos + key.size());
    }
  }
  return "move_to_detect";
}

int main(int argc, char ** argv)
{
  std::string action_name = parse_action_name(argc, argv);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExploreAction>(action_name);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
