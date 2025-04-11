import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument ,RegisterEventHandler,LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import (
    OnProcessExit,
    OnProcessStart
)
from launch.events.process import ProcessExited, ProcessStarted
from launch.launch_context import LaunchContext
from launch_ros.actions import Node 
import requests

def is_target_process(process_action, target_list):
    """Return True if the process_action is in the target list."""
    return process_action in target_list

def on_spawn_success(event: ProcessStarted, context: LaunchContext):
    """Callback for when spawn process starts successfully"""
    try:
        requests.get("http://localhost:8000/robot/intiallized")
        return [LogInfo(msg="Robot initialized successfully")]
    except Exception as e:
        print(f"Error in on_spawn_success: {e}")
        return []

def on_process_exit(event: ProcessExited, context: LaunchContext):
    """Callback for when a monitored process exits"""
    try:
        requests.get("http://localhost:8000/robot/error")
        return [LogInfo(msg=f"Process exited: {event.action}")]
    except Exception as e:
        print(f"Error in on_process_exit: {e}")
        return []

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
        rsp,
        gazebo,
        bridge,
        spawn_entity,
        RegisterEventHandler(
            OnProcessStart(
                target_action=spawn_entity,
                on_start=on_spawn_success
                
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=lambda action: action in [gazebo, rsp, bridge],
                on_exit=on_process_exit
            )
        )
    ])