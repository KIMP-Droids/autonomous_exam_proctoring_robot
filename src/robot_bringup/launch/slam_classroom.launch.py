#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz if true'
    )

    # Package paths
    bringup_share = get_package_share_directory('robot_bringup')
    sim_share = get_package_share_directory('robot_sim')
    slam_share = get_package_share_directory('robot_slam')

    # Include sim_classroom launch (includes world, robot, bridges)
    sim_classroom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim_classroom.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': 'false'  # We'll launch RViz separately with map display
        }.items()
    )

    # SLAM Toolbox node
    slam_config = os.path.join(slam_share, 'config', 'slam_toolbox_online_async.yaml')
    
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz config path (with map display)
    rviz_config = os.path.join(bringup_share, 'config', 'rviz', 'slam.rviz')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(rviz),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        sim_classroom_launch,
        slam_toolbox_node,
        rviz_node,
    ])

