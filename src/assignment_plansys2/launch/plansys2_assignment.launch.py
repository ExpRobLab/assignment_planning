import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'assignment_plansys2'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Path to PDDL files
    domain_file = os.path.join(pkg_share, 'pddl', 'domain.pddl')
    problem_file = os.path.join(pkg_share, 'pddl', 'problem.pddl')

    # PlanSys2 Launch
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'), 'launch', 'plansys2_bringup_launch_distributed.py')),
        launch_arguments={
            'model_file': domain_file,
            'problem_file': problem_file
        }.items()
    )

    # Action Executors
    move_to_detect_node = Node(
        package=pkg_name,
        executable='explore_action_node',
        name='move_to_detect_executor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    change_state_node = Node(
        package=pkg_name,
        executable='change_state_action_node',
        name='change_state_executor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    move_to_photograph_node = Node(
        package=pkg_name,
        executable='capture_action_node',
        name='move_to_photograph_executor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        plansys2_cmd,
        move_to_detect_node,
        change_state_node,
        move_to_photograph_node
    ])