import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")

    declare_config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("diffdrive_bringup"), "config"
        ]),
        description="Full path to the configuration file to load",
    )

    declare_config_arg = DeclareLaunchArgument(
        "config",
        default_value="cartographer.lua",
        description="Full path to the configuration file to load",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.05",
        description="Resolution of the map",
    )

    declare_publish_period_sec_arg = DeclareLaunchArgument(
        "publish_period_sec",
        default_value="1.0",
        description="Publish period in seconds",
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            "-configuration_directory",
            LaunchConfiguration("config_path"),
            "-configuration_basename",
            config,
        ],
        remappings = [
            ('points2', 'lidar')],
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    return LaunchDescription([
        declare_config_path_arg,
        declare_config_arg,
        declare_use_sim_time,
        declare_resolution_arg,
        declare_publish_period_sec_arg,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])