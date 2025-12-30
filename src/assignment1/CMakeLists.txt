cmake_minimum_required(VERSION 3.8)
project(assignment1)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/aruco_detection.py
  scripts/detection_primitives.py
  scripts/mission_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
