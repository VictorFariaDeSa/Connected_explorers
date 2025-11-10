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
    gazebo_config_path = os.path.join(get_package_share_directory(bringup_pkg), 'config', 'gazebo_bridge.yaml')
    rviz_config_file = os.path.join(get_package_share_directory(bringup_pkg), 'config', 'rviz_visualization_config.rviz')
    world_path = os.path.join(get_package_share_directory(bringup_pkg), 'worlds', 'test_world.world')

    robots = [
        {'name': 'robot1', 'x': 0.0, 'y': 0.0},
        {'name': 'robot2', 'x': 4.0, 'y': 0.0},
        {'name': 'robot3', 'x': 2.0, 'y': 3.0},
        {'name': 'robot4', 'x': -1.0, 'y': 3.0},
        {'name': 'robot5', 'x': 5.0, 'y': 3.0},
    ]

    ld_nodes = []

    # Use simulation time
    ld_nodes.append(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'))

    # Launch Gazebo
    ld_nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'{world_path} -r'}.items()
        )
    )

    for robot in robots:
        name = robot['name']


    

        # Static transform: world -> base_footprint
        ld_nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=name,
            name='world_to_base',
            arguments=[str(robot['x']), str(robot['y']), '0', '0', '0', '0', 'world', f'{name}/base_footprint'],
            output='screen'
        ))


        # Robot State Publisher with frame_prefix for unique TFs
        ld_nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            name='state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'use_sim_time': True,
                'frame_prefix': f'{name}/'
            }]
        ))

        ld_nodes.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=name,
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
        ))


        # Spawn robot in Gazebo
        ld_nodes.append(Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{name}',
            arguments=['-name', name, '-topic', f'{name}/robot_description', '-x', str(robot['x']), '-y', str(robot['y']), '-z', '0'],
            output='screen'
        ))

    # ROS-Gazebo bridge
    ld_nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': gazebo_config_path}],
        output='screen'
    ))

    # RViz
    ld_nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    ))

    ld_nodes.append(Node(
        package='line_viewer',
        executable='robot_link_visualizer',  # deve ser entry_point do setup.py
        name='robot_link_visualizer',
        output='screen',
        emulate_tty=True,
        parameters=[{'fixed_frame': 'world'}]  # opcional, se quiser passar o frame como parâmetro
    ))

    return LaunchDescription(ld_nodes)
