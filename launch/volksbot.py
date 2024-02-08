import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

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
            'axis_length',
            default_value='0.435',
            description='Volksbot axis length'),

        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Publish tf data'),

        DeclareLaunchArgument(
            'turning_adaptation',
            default_value='0.95',
            description='turning_adaptation'),

        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='tf prefix. Attention: currently not used in driver!'),

        Node(
            package='volksbot_driver',
            executable='volksbot',
            name='volksbot',
            parameters=[{'num_wheels': num_wheels, 'wheel_radius': wheel_radius,'robot_description': robot_desc}],
            output='screen'),
    ])

