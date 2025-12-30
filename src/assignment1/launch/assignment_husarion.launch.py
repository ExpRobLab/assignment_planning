import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # World argument
    gazebo_models_path = os.path.join(get_package_share_directory("worlds_manager"), "models")
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path

    pkg_map = get_package_share_directory('ros2_navigation')
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_map, 'launch', 'mapping.launch.py'),
        )
    )
    pkg_loc = get_package_share_directory('ros2_navigation')
    loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_loc, 'launch', 'localization.launch.py'),
        )
    )
    pkg_nav = get_package_share_directory('ros2_navigation')
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py'),
        )
    )

    pkg_world = get_package_share_directory('worlds_manager')
    world_path = os.path.join(pkg_world, 'worlds', 'my_world_assignment.sdf')
    world_arg = DeclareLaunchArgument(
        'gz_world', default_value=world_path,
        description='Name of the Gazebo world file to load'
    )

    pkg_robot = get_package_share_directory('rosbot_gazebo')
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'simulation.launch.py'),
        )
        ,
        launch_arguments={
        'robot_model': 'rosbot',
        'mecanum': 'True',
        'rviz': 'True',
        'gz_world': LaunchConfiguration('gz_world'),
        'joy_vel': '/cmd_vel',
        'use_sim_time': 'True',
        'x':'2',
        'y':'2',
        'z':'0',
        }.items()
    )

    camera_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/color/image_raw',
        description='Name of the image topic that aruco has to check'
    )
    remap_camera = TimerAction(
        period=2.0,
        actions=[Node(
        package='topic_tools',
        executable='relay',
        name='oak_color_relay',
        arguments=['/oak/rgb/color', LaunchConfiguration('image_topic')],
        output='screen'
        )]
    )

    
    # Launch aruco_tracker
    pkg_aruco_opencv = get_package_share_directory('aruco_opencv')
    aruco_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_aruco_opencv, 'launch', 'aruco_tracker_husarion.launch.xml'),
        )
    )
    # Add a dealy of 5 sec to ensuring that the topic is present
    delayed_aruco_launch = TimerAction(period=5.0, actions=[aruco_launch])

    # Bridge /cmd_vel from and to gazebo and ros topics
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    # Run the scripts of the assignment
    marker_detection = Node(
        package="assignment1",
        executable="aruco_detection.py",
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'base_frame': 'base_link'
        }]
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(robot_launch)
    launchDescriptionObject.add_action(camera_arg)
    # launchDescriptionObject.add_action(remap_camera)
    launchDescriptionObject.add_action(delayed_aruco_launch)    
    launchDescriptionObject.add_action(cmd_vel_bridge)    
    launchDescriptionObject.add_action(marker_detection)    
    launchDescriptionObject.add_action(map_launch)    
    launchDescriptionObject.add_action(loc_launch)    
    launchDescriptionObject.add_action(nav_launch)  

    return launchDescriptionObject
