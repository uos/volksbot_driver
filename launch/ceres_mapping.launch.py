import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource

from pathlib import Path

import xacro

def evaluate_volksbot_joy_config(context: LaunchContext, joy_launch_config):

    # Setup name of configuration file
    config_file_name = context.perform_substitution(joy_launch_config)
    volksbot_driver_dir = get_package_share_directory('volksbot_driver')
    joystick_config_file = os.path.join(volksbot_driver_dir, 'config/joystick/', config_file_name)
  
    # Return list with joystic node generated from 
    # imported lauch file with joystick parameter file
    return [GroupAction(
        actions=[
#            SetRemap(src='/odom_combined',dst='/odom'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(volksbot_driver_dir + '/launch/volksbot_joy.py'),
                launch_arguments={'config_filepath': joystick_config_file}.items(),
            )
        ])
    ]

def evaluate_volksbot_slam_config(context: LaunchContext, slam_launch_config):

    # Get joystick package directory
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Setup name of configuration file
    config_file_name = context.perform_substitution(slam_launch_config)
    slam_config_file = os.path.join(get_package_share_directory('volksbot_driver'), 'config/slam_toolbox/', config_file_name)
  
    # Return list with slam toolbox nodes generated from 
    # imported lauch file with joystick parameter file
    return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_dir + '/launch/online_async_launch.py'),
                launch_arguments={'slam_params_file': slam_config_file}.items(),
    )]      

def generate_launch_description():

    urg_driver_dir = get_package_share_directory('urg_node2')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joystick_config',
            default_value='8bitdo.config.yaml',
            description='Joystick configuration file found in /config/joystick'
        ),
        DeclareLaunchArgument(
            'slam_config',
            default_value='online_async.yaml',
            description='SLAM toolbox configuration file found in /config/slam_toolbox'
        ),
        # Opaque function call to evaluate the joystick configuration
        # and pass it on to the volks_bot_joy.launch.py file
        OpaqueFunction(
            function=evaluate_volksbot_joy_config, 
            args=[LaunchConfiguration('joystick_config')]
        ),
        # Define tf between base_link and laser 
        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "base_link", "laser"]
        ),
        # Define tf between odom frame coming from the slam toolbox
        # and base_footprint
        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_footprint"]
        ),
        # Lauch urg laser node
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(urg_driver_dir + '/launch/urg_node2.launch.py'),        
        ),
        # Call slam_toolbox launch file with custom parameter file
        OpaqueFunction(
            function=evaluate_volksbot_slam_config, 
            args=[LaunchConfiguration('slam_config')]
        ),
    ])
