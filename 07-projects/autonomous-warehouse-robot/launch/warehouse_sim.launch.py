#!/usr/bin/env python3
"""
Warehouse Simulation Launch File

Launches the complete autonomous warehouse robot simulation including:
- Isaac Sim simulation environment
- ROS2 nodes for navigation, SLAM, and manipulation
- Warehouse environment with packages and obstacles
- Multi-robot coordination (optional)

Author: Tanmay Pancholi
Date: 2024
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description for warehouse simulation"""
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='1',
        description='Number of robots in simulation'
    )
    
    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM system'
    )
    
    enable_manipulation_arg = DeclareLaunchArgument(
        'enable_manipulation',
        default_value='true',
        description='Enable manipulation system'
    )
    
    enable_navigation_arg = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Enable navigation system'
    )
    
    warehouse_size_arg = DeclareLaunchArgument(
        'warehouse_size',
        default_value='large',
        description='Warehouse size: small, medium, large'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_count = LaunchConfiguration('robot_count')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_manipulation = LaunchConfiguration('enable_manipulation')
    enable_navigation = LaunchConfiguration('enable_navigation')
    warehouse_size = LaunchConfiguration('warehouse_size')
    
    # Get package directory
    pkg_share = FindPackageShare('warehouse_robot_sim').find('warehouse_robot_sim')
    
    # Isaac Sim launch
    isaac_sim_launch = ExecuteProcess(
        cmd=[
            'python3', 
            PathJoinSubstitution([
                pkg_share, 'scripts', 'start_isaac_sim.py'
            ]),
            '--warehouse_size', warehouse_size,
            '--robot_count', robot_count
        ],
        name='isaac_sim',
        output='screen'
    )
    
    # ROS2 Bridge launch
    ros2_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_bridge'),
                'launch',
                'isaac_ros_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(
                PathJoinSubstitution([
                    pkg_share, 'models', 'warehouse_robot', 'warehouse_robot.urdf'
                ])
            ).read(),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Navigation system
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share, 'launch', 'navigation.launch.py'
            ])
        ]),
        condition=IfCondition(enable_navigation),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # SLAM system
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share, 'launch', 'slam.launch.py'
            ])
        ]),
        condition=IfCondition(enable_slam),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Manipulation system
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share, 'launch', 'manipulation.launch.py'
            ])
        ]),
        condition=IfCondition(enable_manipulation),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Warehouse manager
    warehouse_manager = Node(
        package='warehouse_robot_sim',
        executable='warehouse_manager',
        name='warehouse_manager',
        parameters=[{
            'warehouse_size': warehouse_size,
            'robot_count': robot_count,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Performance monitor
    performance_monitor = Node(
        package='warehouse_robot_sim',
        executable='performance_monitor',
        name='performance_monitor',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # RViz for visualization
    rviz_config_file = PathJoinSubstitution([
        pkg_share, 'config', 'warehouse_sim.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        robot_count_arg,
        enable_slam_arg,
        enable_manipulation_arg,
        enable_navigation_arg,
        warehouse_size_arg,
        
        # Core systems
        isaac_sim_launch,
        ros2_bridge_launch,
        robot_state_publisher,
        joint_state_publisher,
        
        # Robot systems
        navigation_launch,
        slam_launch,
        manipulation_launch,
        
        # Management systems
        warehouse_manager,
        performance_monitor,
        
        # Visualization
        rviz
    ])
