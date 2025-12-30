import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    pkg_plansys = get_package_share_directory('assignment_plansys2')
    plan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_plansys, 'launch', 'plansys2_assignment.launch.py'),
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_models_path = os.path.join(get_package_share_directory("worlds_manager"), "models")
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path

    pkg_map = get_package_share_directory('ros2_navigation')
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_map, 'launch', 'mapping.launch.py'),
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    pkg_loc = get_package_share_directory('ros2_navigation')
    loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_loc, 'launch', 'localization.launch.py'),
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    pkg_nav = get_package_share_directory('ros2_navigation')
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py'),
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # World spawn
    world_arg = DeclareLaunchArgument(
        'world', default_value='my_world_assignment.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_world = get_package_share_directory('worlds_manager')
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_world, 'launch', 'my_launch_assignment.py'),
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )

    pkg_robot = get_package_share_directory('robot_manager')
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'spawn_robot_assignment.launch.py'),
        ),
        launch_arguments={
            'x': '1',
            'y': '1'
        }.items()
    )

    # Launch the aruco tracker
    pkg_aruco_opencv = get_package_share_directory('aruco_opencv')
    aruco_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_aruco_opencv, 'launch', 'aruco_tracker.launch.xml'),
        )
    )
    
    # Run scripts of the assignment
    marker_detection = Node(
        package="assignment1",
        executable="aruco_detection.py",
        output='screen',
        parameters=[{
            'image_topic': '/camera/image',
            'base_frame': 'base_footprint',
            'use_sim_time': True
        }]
        # parameters=[
        #     {'use_sim_time': LaunchConfiguration('use_sim_time')},
        # ]
    )

    action_primitives = Node(
        package="assignment1",
        executable="detection_primitives.py",
        output='screen',
        parameters=[{
            'image_topic': '/camera/image',
            'base_frame': 'base_footprint',
            'use_sim_time': True
        }]
        # parameters=[
        #     {'use_sim_time': LaunchConfiguration('use_sim_time')},
        # ]
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(robot_launch)
    launchDescriptionObject.add_action(aruco_launch)
    # launchDescriptionObject.add_action(marker_detection)   
    launchDescriptionObject.add_action(action_primitives)
    launchDescriptionObject.add_action(map_launch)    
    # launchDescriptionObject.add_action(loc_launch)    
    launchDescriptionObject.add_action(nav_launch)    
    launchDescriptionObject.add_action(plan_launch)

    return launchDescriptionObject
