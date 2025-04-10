import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo,RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart
from launch.events.process import ProcessStarted
from launch.launch_context import LaunchContext
import requests

def is_target_process(process_action, target_list):
    """Return True if the process_action is in the target list."""
    return process_action in target_list


def on_slam_ready(event: ProcessStarted, context: LaunchContext):
    """Callback for when SLAM toolbox process starts successfully"""
    try:
        requests.get("http://localhost:8000/mapping/intiallized")
        return [LogInfo(msg="SLAM toolbox initialized successfully")]
    except Exception as e:
        print(f"Error in on_slam_ready: {e}")
        return []


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=start_async_slam_toolbox_node,
                on_start=on_slam_ready
            )
        )
    )

    return ld
