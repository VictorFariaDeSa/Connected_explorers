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
    gazebo_config_path = os.path.join(get_package_share_directory(bringup_pkg), 'config', 'gazebo_bridge.yaml')

    # Spawn robots
    spawn_robot1 = Node(
        package='ros_gz_sim', 
        executable='create', 
        name='spawn_kusari1', 
        arguments=['-name', 'kusari_bot1', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    spawn_robot2 = Node(
        package='ros_gz_sim', 
        executable='create', 
        name='spawn_kusari2', 
        arguments=['-name', 'kusari_bot2', '-topic', 'robot_description', '-x', '2', '-y', '0', '-z', '0'],
        output='screen'
    )

    spawn_robot3 = Node(
        package='ros_gz_sim', 
        executable='create', 
        name='spawn_kusari3', 
        arguments=['-name', 'kusari_bot3', '-topic', 'robot_description', '-x', '4', '-y', '0', '-z', '0'],
        output='screen'
    )

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),

        # Publica um frame world -> odom estático
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint'],
            output='screen'
        ),


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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': 'empty.sdf'}.items()
        ),

        Node(
            package='ros_gz_sim', 
            executable='create', 
            name='spawn_kusari', 
            arguments=['-name', 'kusari_bot', '-topic', 'robot_description'],
            output='screen'
        ),

        spawn_robot1,
        spawn_robot2,
        spawn_robot3,


        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{'config_file': gazebo_config_path}],
            output='screen'
        ),

        # Publica os marcadores da linha entre os robôs
        Node(
            package='line_viewer',
            executable='robot_link_visualizer',  # deve ser entry_point do setup.py
            name='robot_link_visualizer',
            output='screen',
            emulate_tty=True,
            parameters=[{'fixed_frame': 'world'}]  # opcional, se quiser passar o frame como parâmetro
        )

    ])
