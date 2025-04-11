import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package
    package_name='pal_robot'

    bridge_file=os.path.join(get_package_share_directory(package_name), 'config', 'bridge.yaml')
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Configure Gazebo Harmonic world
    world_path = os.path.join(get_package_share_directory(package_name), 'config', 'warehousev2.sdf')
    
    # Robot state publisher launch
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), 
                launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include the Gazebo Harmonic launch file
    # Note: Gazebo Harmonic uses gz_sim package instead of gazebo_ros
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={
                        'gz_args': ['-r ', world_path],
                        'on_exit_shutdown': 'true'
                    }.items()
             )

    # Bridge to connect ROS 2 and Gazebo Harmonic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{'config_file': bridge_file}],  # Load bridges from YAML
        output='screen'
    )

    # Spawn entity in Gazebo Harmonic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robit',
            '-topic', '/robot_description',
            '-x', '1.00',
            '-y', '0.00',
            '-z', '0.5',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '1.57'
        ],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'),
        gazebo,
        rsp,
        bridge,
        spawn_entity
    ])
