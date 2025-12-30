import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
	world_arg = DeclareLaunchArgument(
		'world', default_value='my_world_assignment.sdf',
		description='Name of the Gazebo world file to load'
	)

	pkg_erl1= get_package_share_directory('worlds_manager')
	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
	 
	# Add your own gazebo library path here
	gazebo_models_path =get_package_share_directory("worlds_manager")
	gazebo_models_path, ignore_last_dir = os.path.split(gazebo_models_path)
	gazebo_models_path = os.path.join(gazebo_models_path, 'worlds_manager','models')
	# print(gazebo_models_path)
	if "GZ_SIM_RESOURCE_PATH" in os.environ:
		os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
	else:
		os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path
	# print("GZ_SIM_RESOURCE_PATH:", os.environ["GZ_SIM_RESOURCE_PATH"])

	gazebo_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
		),
		launch_arguments={'gz_args': [PathJoinSubstitution([
			pkg_erl1,
			'worlds',
			LaunchConfiguration('world')
		]),
		#TextSubstitution(text=' -r -v -v1 --render-engine ogre')],
		TextSubstitution(text=' -r -v -v1')],
		'on_exit_shutdown': 'true'}.items()
	)
	
	launchDescriptionObject = LaunchDescription()
	launchDescriptionObject.add_action(world_arg)
	launchDescriptionObject.add_action(gazebo_launch)
	return launchDescriptionObject
