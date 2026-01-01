#include <memory>
#include <fstream>
#include <map>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "robot_manager/action/center.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <cstdlib>
#include <filesystem>

using namespace std::chrono_literals;

class CaptureAction : public plansys2::ActionExecutorClient
{
public:
  CaptureAction()
  : plansys2::ActionExecutorClient("move_to_photograph", 500ms)
  {
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    center_client_ = rclcpp_action::create_client<robot_manager::action::Center>(this, "center_on_marker");
    
    auto qos = rclcpp::SensorDataQoS();
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", qos, 
        [this](const sensor_msgs::msg::Image::SharedPtr msg){ last_img_ = msg; });
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    // Cache current dispatch arguments (move_to_photograph ?r ?to ?from)
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

    target_marker_id_ = marker_name_to_id(to_marker_);

    // Load marker poses from file (written during move_to_detect)
    marker_poses_.clear();
    std::ifstream file("/tmp/detected_markers.csv");
    long id; char comma; double x, y;
    while (file >> id >> comma >> x >> comma >> y) {
      marker_poses_[id] = {x, y};
    }

    has_target_pose_ = (target_marker_id_ >= 0 && marker_poses_.find(target_marker_id_) != marker_poses_.end());

    step_ = 0;
    nav_goal_sent_ = false;
    nav_done_ = false;
    center_goal_sent_ = false;
    center_done_ = false;
    center_success_ = false;

    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void do_work()
  {
    if (to_marker_.empty() || target_marker_id_ < 0) {
      finish(false, 1.0, "Missing/invalid action parameters");
      return;
    }
    if (!has_target_pose_) {
      finish(false, 1.0, "No stored pose for target marker");
      return;
    }

    long mid = target_marker_id_;
    double mx = marker_poses_[target_marker_id_].first;
    double my = marker_poses_[target_marker_id_].second;

    if (step_ == 0) { // Navigate
        if (!nav_goal_sent_) {
            auto goal = nav2_msgs::action::NavigateToPose::Goal();
            goal.pose.header.frame_id = "map";
            goal.pose.header.stamp = now();
            goal.pose.pose.position.x = mx;
            goal.pose.pose.position.y = my;
            goal.pose.pose.orientation.w = 1.0;
            
            auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            opts.result_callback = [this](auto) { nav_done_ = true; };
            nav_client_->async_send_goal(goal, opts);
            nav_goal_sent_ = true;
            nav_done_ = false;
            RCLCPP_INFO(get_logger(), "Going to Marker %ld", mid);
        }
        if (nav_done_) { nav_goal_sent_ = false; step_ = 1; }
    }
    else if (step_ == 1) { // Center
        if (!center_goal_sent_) {
            auto goal = robot_manager::action::Center::Goal();
            goal.marker_id = mid;
            auto opts = rclcpp_action::Client<robot_manager::action::Center>::SendGoalOptions();
            opts.result_callback = [this](auto r) { 
                center_success_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED); 
                center_done_ = true; 
            };
            center_client_->async_send_goal(goal, opts);
            center_goal_sent_ = true;
            center_done_ = false;
            RCLCPP_INFO(get_logger(), "Centering on %ld", mid);
        }
        if (center_done_) {
            center_goal_sent_ = false;
            if (center_success_) step_ = 2; // Capture
            else {
                finish(false, 1.0, "Centering failed");
                return;
            }
        }
    }
    else if (step_ == 2) { // Save Image
        if (last_img_) {
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_img_, sensor_msgs::image_encodings::BGR8);
                cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2), 
                           50, CV_RGB(0, 255, 0), 3);
                
                const char* home = std::getenv("HOME");
                if (home) {
                    // Construct path: ~/assignment1_bundle/resources/marker_X.png
                    std::string dir_path = std::string(home) + "/assignment1_bundle/resources/";
                    std::string file_path = dir_path + "marker_" + std::to_string(mid) + ".png";
                    
                    try {
                        std::filesystem::create_directories(dir_path);
                        if (cv::imwrite(file_path, cv_ptr->image)) {
                            RCLCPP_INFO(get_logger(), "Saved to %s", file_path.c_str());
                        } else {
                            RCLCPP_ERROR(get_logger(), "Failed to write image to %s", file_path.c_str());
                        }
                    } catch (const std::filesystem::filesystem_error& e) {
                        RCLCPP_ERROR(get_logger(), "Filesystem error: %s", e.what());
                    }
                } else {
                    RCLCPP_ERROR(get_logger(), "Could not determine HOME directory to save images.");
                }
            } catch(...) {}
        }
        finish(true, 1.0, "Photograph captured");
        return;
    }
  }

private:
  int step_ = 0;
  bool nav_goal_sent_ = false; bool nav_done_ = false;
  bool center_goal_sent_ = false; bool center_done_ = false; bool center_success_ = false;

  std::string robot_;
  std::string to_marker_;
  std::string from_marker_;
  long target_marker_id_ = -1;
  bool has_target_pose_ = false;

  std::map<long, std::pair<double, double>> marker_poses_;

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
  
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<robot_manager::action::Center>::SharedPtr center_client_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  sensor_msgs::msg::Image::SharedPtr last_img_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CaptureAction>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}