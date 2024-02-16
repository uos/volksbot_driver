import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml
import ament_index_python.packages

def generate_launch_description():

    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('volksbot_driver'),
        'config')
    param_config = os.path.join(config_directory, 'volksbot.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['volksbot']['ros__parameters']

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
        
        Node(
            package='volksbot_driver',
            executable='volksbot',
            name='volksbot',
            parameters=[params],
            output='screen'),
    ])

