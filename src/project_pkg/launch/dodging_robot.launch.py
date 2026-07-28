"""
Launches the full detection + tracking pipeline, plus RViz2 for visualization.

Assumes TurtleBot4 bringup (LiDAR driver, odom -> base_link TF, etc.) is
already running on the robot itself. This file only starts nodes meant to
run on a separate lab computer connecting to the robot over the network.

Usage:
    ros2 launch project_pkg dodging_robot.launch.py

Optional arguments:
    fixed_frame:=<frame>     frame tracked objects are published in (default: odom)
    launch_rviz:=false       skip starting RViz2 (default: true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fixed_frame_arg = DeclareLaunchArgument(
        'fixed_frame',
        default_value='odom',
        description='Fixed frame that /tracked_objects is published in'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to start RViz2 alongside the pipeline'
    )

    detector_node = Node(
        package='project_pkg',
        executable='lidar_object_detector_node',
        name='lidar_object_detector',
        output='screen',
    )

    tracker_node = Node(
        package='project_pkg',
        executable='object_tracker_node',
        name='object_tracker',
        output='screen',
        parameters=[{'fixed_frame': LaunchConfiguration('fixed_frame')}],
    )

    oa_node = Node(
        package='project_pkg',
        executable='oa_node',
        name='oa_node',
        output='screen',
    )

    visualizer_node = Node(
        package='project_pkg',
        executable='tracked_objects_visualizer',
        name='tracked_objects_visualizer',
        output='screen',
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('project_pkg'),
        'rviz',
        'dodging_robot.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        fixed_frame_arg,
        launch_rviz_arg,
        detector_node,
        tracker_node,
        oa_node,
        visualizer_node,
        rviz_node,
    ])
