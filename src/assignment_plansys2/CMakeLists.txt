cmake_minimum_required(VERSION 3.8)
project(assignment_plansys2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_lifecycle_manager REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(explore_action_node src/explore_action_node.cpp)
ament_target_dependencies(explore_action_node 
  rclcpp rclcpp_action plansys2_executor nav2_msgs robot_manager aruco_opencv_msgs tf2_ros tf2_geometry_msgs geometry_msgs)

add_executable(capture_action_node src/capture_action_node.cpp)
ament_target_dependencies(capture_action_node 
  rclcpp rclcpp_action plansys2_executor nav2_msgs robot_manager sensor_msgs cv_bridge)
target_link_libraries(capture_action_node ${OpenCV_LIBRARIES})

install(TARGETS
  explore_action_node
  capture_action_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY pddl launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
