import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from pathlib import Path

import xacro


def evaluate_joystick_config(context: LaunchContext, joy_launch_config):

    # Get joystick package directory
    teleop_twist_dir = get_package_share_directory('teleop_twist_joy')

    # Setup name of configuration file
    config_file_name = context.perform_substitution(joy_launch_config)
    joystick_config_file = os.path.join(get_package_share_directory('volksbot_driver'), 'config/joystick/', config_file_name)
  
    # Return list with joystic node generated from 
    # imported lauch file with joystick parameter file
    return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_twist_dir + '/launch/teleop-launch.py'),
                launch_arguments={'config_filepath': joystick_config_file}.items(),
    )]



def generate_launch_description():

    # Declare launch configuratuin parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    num_wheels = LaunchConfiguration('num_wheels', default=4)
    wheel_radius = LaunchConfiguration('wheel_radius', default=0.0985)
    publish_tf = LaunchConfiguration('publish_tf', default='false')
    #tf_prefix = LaunchConfiguration("tf_prefix", '')

    # Get Volksbot URDF / xacro file and parse into valid URDF 
    # description
    urdf_file_name = 'urdf/volksbot.urdf.xacro'
    urdf = os.path.join(
        get_package_share_directory('volksbot_driver'),
        urdf_file_name)

    doc = xacro.process_file(urdf)
    robot_desc = doc.toprettyxml(indent='  ')

    # Generate lauch description consisting of launch time 
    # arguments and node configurations. Here we launch a 
    # robot state publisher and volksbot instance
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'num_wheels',
            default_value='4',
            description='Number of wheels of robot base'),
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.0985',
            description='Volksbot wheel radius'),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Publish tf data'),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='tf prefix. Attention: currently not used in driver!'),
        DeclareLaunchArgument(
            'joystick_config',
            default_value='8bitdo.config.yaml',
            description='Joystick configuration file found in /config/joystick'
        ),
        OpaqueFunction(
            function=evaluate_joystick_config, 
            args=[LaunchConfiguration('joystick_config')]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='volksbot_driver',
            executable='volksbot',
            name='volksbot',
            parameters=[{'num_wheels': num_wheels, 'wheel_radius': wheel_radius,'robot_description': robot_desc}],
            output='screen'),
        
    ])
