#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    desc_share = get_package_share_directory('robot_description')
    
    # Set Gazebo resource path so meshes can be found
    import os
    gz_resource_path = os.path.join(desc_share, '..')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path + ':' + os.environ['GZ_SIM_RESOURCE_PATH']
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    # Include robot_sim world launch
    sim_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'classroom_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Bridge for /clock (Gazebo -> ROS 2) - ESSENTIAL for use_sim_time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # Bridge for cmd_vel (ROS 2 -> Gazebo)
    # Format: /ros_topic@ros_msg_type@gz.msgs.GzType
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Bridge for odom (Gazebo -> ROS 2)
    # Note: Topic names may need adjustment based on Gazebo plugin output
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Bridge for scan (Gazebo -> ROS 2)
    # Note: Gazebo LiDAR may publish to different topic, adjust if needed
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Controller spawner - spawns joint_state_broadcaster from ros2_control
    # This publishes joint states from Gazebo simulation
    # NOTE: Don't pass -p params file - controller config is already loaded by gz_ros2_control plugin
    # NOTE: Use full --controller-manager flag to match working example
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )
    
    # Spawn joint_state_broadcaster after robot spawns (delay to ensure controller_manager is ready)
    # Using TimerAction with delay after sim_world_launch starts
    load_joint_state_broadcaster = TimerAction(
        period=8.0,  # Wait 8 seconds for robot spawn and controller_manager to be ready
        actions=[joint_state_broadcaster_spawner]
    )

    # Bridge for imu (Gazebo -> ROS 2)
    # NOTE: IMU bridge commented out as IMU sensor plugin is temporarily disabled
    # Uncomment when IMU sensor is enabled in URDF
    # imu_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='imu_bridge',
    #     arguments=[
    #         '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
    #     ],
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }],
    #     output='screen'
    # )

    # RViz config path
    rviz_config = os.path.join(bringup_share, 'config', 'rviz', 'sim.rviz')

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
        sim_world_launch,
        clock_bridge,  # MUST be first - provides /clock for use_sim_time
        cmd_vel_bridge,
        odom_bridge,
        scan_bridge,
        load_joint_state_broadcaster,  # Spawns joint_state_broadcaster from ros2_control
        # imu_bridge,  # Commented out - IMU sensor temporarily disabled
        rviz_node,
    ])

