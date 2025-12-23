#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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

    bringup_share = get_package_share_directory('robot_bringup')
    slam_share = get_package_share_directory('robot_slam')

    sim_classroom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim_classroom.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': 'false'
        }.items()
    )

    slam_config = os.path.join(slam_share, 'config', 'slam_toolbox_online_async.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {
                'use_sim_time': use_sim_time,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
            }
        ],
    )

    rviz_config = os.path.join(bringup_share, 'config', 'rviz', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Delay SLAM a bit so /clock + /tf exist
    slam_delayed = TimerAction(period=2.0, actions=[slam_toolbox_node])
    rviz_delayed = TimerAction(period=3.0, actions=[rviz_node])

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        sim_classroom_launch,
        slam_delayed,
        rviz_delayed,
    ])
