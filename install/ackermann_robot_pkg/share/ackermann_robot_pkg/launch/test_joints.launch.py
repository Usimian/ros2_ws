#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ackermann_robot_pkg')
    world_file = os.path.join(pkg_dir, 'worlds', 'ackermann_world.sdf')
    
    # Set Gazebo model path
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_dir, 'models')
    )
    
    # Set GZ_SIM_RESOURCE_PATH for Gazebo Sim
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_dir, 'models')
    )
    
    # Enable verbose Gazebo logging
    gz_verbose = SetEnvironmentVariable(
        name='GZ_VERBOSE',
        value='3'
    )
    
    # Start Gazebo with verbose output
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen'
    )
    
    # Bridge for wheel velocity control
    fl_wheel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fl_wheel_bridge',
        arguments=['/front_left_wheel_joint/velocity@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    
    fr_wheel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fr_wheel_bridge',
        arguments=['/front_right_wheel_joint/velocity@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    
    rl_wheel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rl_wheel_bridge',
        arguments=['/rear_left_wheel_joint/velocity@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    
    rr_wheel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rr_wheel_bridge',
        arguments=['/rear_right_wheel_joint/velocity@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    
    # Bridge for steering control
    left_steer_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='left_steer_bridge',
        arguments=['/front_left_steer_joint/cmd_position@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    
    right_steer_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='right_steer_bridge',
        arguments=['/front_right_steer_joint/cmd_position@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    
    # Run our test node
    wheel_driver = Node(
        package='ackermann_robot_pkg',
        executable='test_direct_drive',
        name='wheel_driver',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_model_path,
        gz_sim_resource_path,
        gz_verbose,
        gazebo,
        fl_wheel_bridge,
        fr_wheel_bridge,
        rl_wheel_bridge,
        rr_wheel_bridge,
        left_steer_bridge,
        right_steer_bridge,
        wheel_driver
    ]) 