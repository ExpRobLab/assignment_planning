# Experimental Robotics Laboratory — Assignment 1
<!-- TOD talk about https://github.com/CarmineD8/erl1_sensors.git -->
<!-- TOD talk about https://husarion.com/manuals/rosbot/?utm_source=chatgpt.com -->
## Task:
- [ ] Spawn a robot (you can use the one built in class) in an environment with 5 Aruco markers (with different IDs). Put them all in circle, so that only by rotating the robot can find all of them.
- [ ] Develop a node that, after all IDs have been identified, moves the robot to the marker with the lower ID, so that the marker is in the center of the image, and publishes a new image on a custom topic, with a circle around the marker found.
- [ ] The robot repeats the same behaviour for all the other markers, in ascending order.
- [ ] Optional: repeat the same behaviour, but changing the controller of the robot with a skid-steer drive controller (the robot should then have 4 wheels).
- [ ] Optional: implement the behaviour also with the real robot.
