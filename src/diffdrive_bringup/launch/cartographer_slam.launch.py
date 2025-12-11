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
    pbstream_filename = LaunchConfiguration("pbstream_filename")

    declare_config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("diffdrive_bringup"), "config"]
        ),
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

    pbstream_filename_arg = DeclareLaunchArgument(
        "pbstream_filename",
        default_value="",
        description="Filename to load the state",
    )

    # Conditional argument for pbstream_filename if it's not empty
    cartographer_arguments = [
        "-configuration_directory",
        LaunchConfiguration("config_path"),
        "-configuration_basename",
        config,
    ]

    # Add the pbstream_filename argument if it is provided
    if pbstream_filename != "":
        cartographer_arguments.append("-load_state_filename")
        cartographer_arguments.append(pbstream_filename)

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=cartographer_arguments,
        remappings=[("points2", "lidar")],
    )

    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-resolution",
            resolution,
            "-publish_period_sec",
            publish_period_sec,
        ],
    )

    # down_sample_node = Node(
    #     package='pcl_ros',
    #     executable='voxel_grid',
    #     name='voxel_downsample',
    #     parameters=[{'input': '/lidar', 'output': '/temp_points', 'leaf_size': 0.05}]
    # )

    # pass_through_node = Node(package='pcl_ros',
    #     executable='passthrough',
    #     name='z_height_filter',
    #     parameters=[{
    #         'input': '/temp_points',
    #         'output': '/filtered_points',
    #         'filter_field_name': 'z',
    #         'filter_limit_min': -0.12,
    #         'filter_limit_max': 2.30,
    #     }]
    # )

    return LaunchDescription(
        [
            declare_config_path_arg,
            declare_config_arg,
            declare_use_sim_time,
            declare_resolution_arg,
            declare_publish_period_sec_arg,
            pbstream_filename_arg,
            # down_sample_node,
            # pass_through_node,
            cartographer_node,
            cartographer_occupancy_grid_node,
        ]
    )
