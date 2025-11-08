import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    description_pkg = 'kusari_bot_description'
    bringup_pkg = 'kusari_bot_bringup'


    urdf_path = os.path.join(get_package_share_directory(description_pkg), 'URDF', 'kusari.urdf.xacro')
    rviz_config_file = os.path.join(get_package_share_directory(bringup_pkg), 'config', 'rviz_visualization_config.rviz')


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
        ),  

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),


    ])
