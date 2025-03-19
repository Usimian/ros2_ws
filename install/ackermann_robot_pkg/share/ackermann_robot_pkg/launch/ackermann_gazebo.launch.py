#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ackermann_robot_pkg')
    world_file = os.path.join(pkg_dir, 'worlds', 'ackermann_world.sdf')
    
    # Set Gazebo model path
    model_path = os.path.join(pkg_dir, 'models')
    
    # Environment variables for the model path
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_dir, 'models')
    )
    
    # Set GZ_SIM_RESOURCE_PATH for Gazebo Sim
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_dir, 'models')
    )
    
    # Print the model paths for debugging
    print(f"Setting GAZEBO_MODEL_PATH to: {os.path.join(pkg_dir, 'models')}")
    print(f"Setting GZ_SIM_RESOURCE_PATH to: {os.path.join(pkg_dir, 'models')}")
    
    # Start Gazebo with verbose output
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen'
    )
    
    # Start the Ackermann controller
    controller = Node(
        package='ackermann_robot_pkg',
        executable='control_ackermann',
        name='ackermann_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/cmd_vel', '/model/ackermann_robot/cmd_vel')
        ]
    )
    
    # Teleop keyboard - Use standard cmd_vel topic
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/cmd_vel', '/model/ackermann_robot/cmd_vel'),
        ]
    )
    
    # Bridge Gazebo camera to ROS 2
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )
    
    # Bridge Gazebo ultrasonic sensor to ROS 2
    ultrasonic_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ultrasonic_bridge',
        arguments=['/ultrasonic@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )
    
    # Bridge ROS 2 cmd_vel to Gazebo (standard)
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/model/ackermann_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )
    
    # Bridge for front left steering joint
    left_steer_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='left_steer_bridge',
        arguments=['/front_left_steer_joint/cmd_position@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    
    # Bridge for front right steering joint
    right_steer_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='right_steer_bridge',
        arguments=['/front_right_steer_joint/cmd_position@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    
    # Add bridge for odometry
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=['/model/ackermann_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )
    
    # Add bridge for tf
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        arguments=['/model/ackermann_robot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_model_path,
        gz_sim_resource_path,
        gazebo,
        camera_bridge,
        ultrasonic_bridge,
        cmd_vel_bridge,
        left_steer_bridge,
        right_steer_bridge,
        odom_bridge,
        tf_bridge,
        controller,
        teleop_node
    ])