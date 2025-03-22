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
    
    # Get the current GAZEBO_MODEL_PATH from the environment
    current_model_path = "/home/mw/gazebo_models_worlds_collection/models"
    
    # Environment variables for the model path - include both paths
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=f"{os.path.join(pkg_dir, 'models')}:{current_model_path}"
    )
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{os.path.join(pkg_dir, 'models')}:{current_model_path}"
    )
        # Enable verbose Gazebo logging
    gz_verbose = SetEnvironmentVariable(
        name='GZ_VERBOSE',
        value='3'
    )
    
    # Print the model paths for debugging
    print(f"Setting GAZEBO_MODEL_PATH to: {os.path.join(pkg_dir, 'models')}:{current_model_path}")
    
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
    
    # Auto mode toggle node
    auto_mode_toggle = Node(
        package='ackermann_robot_pkg',
        executable='auto_mode_toggle',
        name='auto_mode_toggle',
        output='screen',
        prefix='xterm -e',
        parameters=[{'use_sim_time': True}]
    )
    
    # Teleop keyboard - Use model-specific cmd_vel topic
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
    
    # Bridge ROS 2 cmd_vel to Gazebo
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
    
    # Bridge for auto mode control
    auto_mode_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='auto_mode_bridge',
        arguments=['/auto_mode@std_msgs/msg/Bool@gz.msgs.Boolean'],
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
    
    # Add bridge for clock
    # The [gz.msgs.Clock] syntax means it's a subscriber to Gazebo clock
    # which then publishes to ROS2 /clock as rosgraph_msgs/msg/Clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Print usage instructions
    print("\n========== AUTONOMOUS MODE INSTRUCTIONS ==========")
    print("In the auto_mode_toggle terminal window:")
    print("- Press 'a' to toggle autonomous obstacle avoidance mode")
    print("- Press 'q' to quit the auto mode toggle node")
    print("================================================\n")
    
    return LaunchDescription([
        gazebo_model_path,
        gz_sim_resource_path,
        gz_verbose,
        gazebo,
        camera_bridge,
        ultrasonic_bridge,
        cmd_vel_bridge,
        left_steer_bridge,
        right_steer_bridge,
        auto_mode_bridge,
        odom_bridge,
        tf_bridge,
        clock_bridge,
        controller,
        teleop_node,
        auto_mode_toggle
    ])