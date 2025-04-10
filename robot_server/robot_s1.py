from launch import LaunchService
import time
from multiprocessing import Process
import launch_sim
import localization_launch_nav
import navigation_launch
from fastapi import FastAPI, HTTPException, Body
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import signal
import psutil
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from threading import Thread
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from fastapi.staticfiles import StaticFiles
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import os
from typing import Dict, Any
from pydantic import BaseModel
from std_srvs.srv import Empty
from nav2_msgs.srv import SaveMap
import tempfile
import shutil
import subprocess
import online_async_launch

# Data models for API requests
class Pose2D(BaseModel):
    x: float
    y: float
    theta: float  # yaw in radians

class NavGoal(BaseModel):
    x: float
    y: float
    theta: float
    frame_id: str = "map"

# Dictionary to keep track of running processes
process_registry = {
    "simulation": None,
    "localization": None,
    "navigation": None,
    "mapping": None  # Add mapping process
}
robot_status = None
process_des = None
localization_status = None
navigation_status = None
mapping_status = None

# Initialize ROS2
rclpy.init()

# Create ROS2 node for lifecycle checking
class LifecycleClient(Node):
    def __init__(self):
        super().__init__('lifecycle_client')
        self.callback_group = ReentrantCallbackGroup()
        
        # Existing clients
        self.amcl_client = self.create_client(
            GetState, 
            '/amcl/get_state',
            callback_group=self.callback_group
        )
        
        self.map_server_client = self.create_client(
            GetState, 
            '/map_server/get_state',
            callback_group=self.callback_group
        )

        # Add navigation nodes clients
        self.controller_client = self.create_client(
            GetState,
            '/controller_server/get_state',
            callback_group=self.callback_group
        )

        self.planner_client = self.create_client(
            GetState,
            '/planner_server/get_state',
            callback_group=self.callback_group
        )

        self.behavior_client = self.create_client(
            GetState,
            '/behavior_server/get_state',
            callback_group=self.callback_group
        )

        self.bt_navigator_client = self.create_client(
            GetState,
            '/bt_navigator/get_state',
            callback_group=self.callback_group
        )
        self.slam_toolbox_client = self.create_client(
            GetState,
            '/slam_toolbox/get_state',
            callback_group=self.callback_group
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create publisher for sending pose estimate to AMCL
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,  # Add a depth parameter
            history=rclpy.qos.HistoryPolicy.KEEP_LAST  # Add a history policy
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            pose_qos
        )

        # Create action client for navigation goals
        self.nav_to_pose_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        
        # Flag to track if navigation goal is active
        self.nav_goal_active = False
        self.current_goal_id = None
        
    def publish_velocity(self, linear_x=0.5, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published velocity: linear_x={linear_x}, angular_z={angular_z}")
    
    def publish_initial_pose(self, x, y, theta):
        """Publish an initial pose estimate to AMCL"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set the pose position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Set the pose orientation using quaternion
        # Convert theta (yaw) to quaternion
        import math
        cy = math.cos(theta * 0.5)
        sy = math.sin(theta * 0.5)
        cp = math.cos(0.0)  # pitch = 0
        sp = math.sin(0.0)
        cr = math.cos(0.0)  # roll = 0
        sr = math.sin(0.0)
        
        msg.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr
        msg.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr
        msg.pose.pose.orientation.y = cy * sp * cr + sy * cp * sr
        msg.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        # Set covariance (default values - identity matrix with small variance)
        covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        
        msg.pose.covariance = covariance
        
        # Publish the message
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f"Published initial pose: x={x}, y={y}, theta={theta}")
        return True
        
    async def send_nav_goal(self, x, y, theta, frame_id="map"):
        """Send a navigation goal to Nav2"""
        # Check if nav2 is active
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False, "Navigation action server not available"
        
        # Create the goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation (convert from euler to quaternion)
        import math
        cy = math.cos(theta * 0.5)
        sy = math.sin(theta * 0.5)
        cp = math.cos(0.0)  # pitch = 0
        sp = math.sin(0.0)
        cr = math.cos(0.0)  # roll = 0
        sr = math.sin(0.0)
        
        goal_msg.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr
        goal_msg.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr
        goal_msg.pose.pose.orientation.y = cy * sp * cr + sy * cp * sr
        goal_msg.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        self.get_logger().info(f"Sending navigation goal: x={x}, y={y}, theta={theta}")
        
        # Send the goal
        self.nav_goal_active = True
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        
        # Wait for goal response
        try:
            goal_response = await send_goal_future
            if not goal_response.accepted:
                self.nav_goal_active = False
                self.get_logger().error('Navigation goal was rejected')
                return False, "Navigation goal was rejected"
            
            self.current_goal_id = goal_response.goal_id
            self.get_logger().info('Navigation goal accepted')
            return True, "Navigation goal accepted"
            
        except Exception as e:
            self.nav_goal_active = False
            self.get_logger().error(f'Error sending navigation goal: {str(e)}')
            return False, f"Error sending navigation goal: {str(e)}"
    
    async def get_node_state(self, client):
        """Get the lifecycle state of a ROS2 node."""
        if not client.wait_for_service(timeout_sec=1.0):
            return "UNKNOWN"
        
        request = GetState.Request()
        future = client.call_async(request)
        
        try:
            response = await future
            # Map ROS2 lifecycle state IDs to string names
            if response.current_state.id == State.PRIMARY_STATE_UNCONFIGURED:
                return "UNCONFIGURED"
            elif response.current_state.id == State.PRIMARY_STATE_INACTIVE:
                return "INACTIVE"
            elif response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                return "ACTIVE"
            elif response.current_state.id == State.PRIMARY_STATE_FINALIZED:
                return "FINALIZED"
            else:
                return "UNKNOWN"
        except Exception:
            return "UNKNOWN"

# Create lifecycle client
lifecycle_client = LifecycleClient()

# Start ROS2 spinning in a separate thread
ros_thread = Thread(target=lambda: rclpy.spin(lifecycle_client))
ros_thread.daemon = True
ros_thread.start()

def run_launch_service(ld):
    ls = LaunchService()
    ls.include_launch_description(ld)
    # Run in the main thread of this process
    global process_des
    process_des=None
    return ls.run()


def run_simulation(process_des):
    ld=None
    if process_des=="simulation":
        ld = launch_sim.generate_launch_description()
    elif process_des=="localization":
        ld = localization_launch_nav.generate_launch_description()
    elif process_des=="navigation":
        ld = navigation_launch.generate_launch_description()
    elif process_des=="mapping":
        ld = online_async_launch.generate_launch_description()
    run_launch_service(ld)


def Process_launch(process_name):
    if process_registry[process_name] is not None and process_registry[process_name].is_alive():
        print(f"{process_name} is already running.")
        return False
    global process_des
    process_des=process_name
    p = Process(target=run_simulation,args=(process_des,))
    p.start()
    process_registry[process_name] = p
    print(f"Launch started: {process_name} (PID: {p.pid})")
    return True


def terminate_process(process_name):
    """Safely terminate a process and its children"""
    if process_registry[process_name] is None or not process_registry[process_name].is_alive():
        return False
    if process_name=="simulation":
        # Get the process object
        proc = process_registry[process_name]
        pid = proc.pid
        
        try:
            # First try to terminate all child processes
            parent = psutil.Process(pid)
            children = parent.children(recursive=True)
            
            # Send SIGINT to give processes a chance to clean up
            for child in children:
                try:
                    child.send_signal(signal.SIGINT)
                except psutil.NoSuchProcess:
                    pass
            
            # Give processes time to terminate gracefully
            time.sleep(2)
            
            # If they're still running, be more forceful (SIGTERM)
            for child in children:
                try:
                    if child.is_alive():
                        child.send_signal(signal.SIGTERM)
                except psutil.NoSuchProcess:
                    pass
            
            # Finally terminate the parent process
            proc.terminate()
            proc.join(timeout=5)
            
            # If it's still alive, kill it
            if proc.is_alive():
                proc.kill()
                proc.join()
            
            # Clear from registry
            process_registry[process_name] = None
            print(f"Process {process_name} (PID: {pid}) has been terminated.")
            return True
        
        except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
            print(f"Error terminating {process_name}: {e}")
            # Still clear from registry as it might be zombie
            process_registry[process_name] = None
            return False
    elif process_name=="localization" or process_name=="navigation":
        localization_process = process_registry[process_name]
        if localization_process is not None:
                try:
                    exit_code = localization_process.exitcode
                    if exit_code is None:  # Process is still running
                        localization_process.terminate()
                        # Wait for process to terminate (with timeout)
                        localization_process.join(timeout=5)
                        # If process didn't terminate within timeout, kill it forcefully
                        if localization_process.exitcode is None:
                            localization_process.kill()
                        return True
                    else:
                        # Process already terminated
                        return True
                except Exception as e:
                    print(f"Error checking process status: {e}")
                    return False
        return False


app = FastAPI(title="Robot Launch Server", 
              description="API to manage ROS 2 robot simulation",
              version="1.0.0")

# Add CORS middleware to allow cross-origin requests
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

# Create a directory for static files if it doesn't exist
static_dir = os.path.join(os.path.dirname(__file__), "static")
os.makedirs(static_dir, exist_ok=True)

# Mount static files directory
app.mount("/static", StaticFiles(directory=static_dir), name="static")


@app.get("/", response_model=dict)
async def root():
    """Root endpoint with API information"""
    return {
        "name": "Robot Launch Server API",
        "version": "1.0.0",
        "endpoints": [
            {"path": "/", "method": "GET", "description": "API information"},
            {"path": "/robot/launch", "method": "GET", "description": "Start the robot simulation"},
            {"path": "/robot/stop", "method": "GET", "description": "Stop the robot simulation"},
            {"path": "/robot/status", "method": "GET", "description": "Get current simulation status"},
            {"path": "/robot/process/status", "method": "GET", "description": "Get status of all managed processes"},
            {"path": "/robot/intiallized", "method": "GET", "description": "Register robot as initialized"},
            {"path": "/robot/error", "method": "GET", "description": "Register robot error state"},
            {"path": "/move/{linear_x}&{angular_z}", "method": "GET", "description": "Move robot with specified velocity"},
            {"path": "/localization/launch", "method": "GET", "description": "Start the localization system"},
            {"path": "/localization/stop", "method": "GET", "description": "Stop the localization system"},
            {"path": "/localization/intiallized", "method": "GET", "description": "Register localization as initialized"},
            {"path": "/navigation/launch", "method": "GET", "description": "Start the navigation system"},
            {"path": "/navigation/stop", "method": "GET", "description": "Stop the navigation system"},
            {"path": "/navigation/intiallized", "method": "GET", "description": "Register navigation as initialized"},
            {"path": "/lifecycle/status", "method": "GET", "description": "Check all nodes lifecycle states"},
            {"path": "/localization/set_pose", "method": "POST", "description": "Set initial pose estimate for AMCL"},
            {"path": "/navigation/send_goal", "method": "POST", "description": "Send navigation goal to Nav2"}
        ]
    }

@app.get("/robot/launch")
async def launch_robot():
    """Launch the robot simulation"""
    success = Process_launch("simulation")
    if success:
        return {"status": "success", "message": "Robot simulation launched"}
    else:
        raise HTTPException(status_code=409, detail="Simulation is already running")


@app.get("/robot/stop")
async def stop_robot():
    """Stop the robot simulation"""
    success = terminate_process("simulation")
    if success:
        return {"status": "success", "message": "Robot simulation stopped"}
    else:
        return {"status": "warning", "message": "No robot simulation running"}


@app.get("/robot/process/status")
async def get_process_status():
    """Get the status of all managed processes"""
    statuses = {}
    
    for name, process in process_registry.items():
        if process is not None and process.is_alive():
            statuses[name] = "running"
        else:
            statuses[name] = "stopped"
    
    return {
        "status": "success",
        "processes": statuses
    }
@app.get("/robot/intiallized")
async def robot_intialized():
    global robot_status
    robot_status="intiallized"
    print(robot_status)
    return "registered"

@app.get("/robot/error")
async def robot_error():
    global robot_status
    robot_status="error"
    print(robot_status)
    return "registered"

@app.get("/robot/status")
async def robot_statu():
    return robot_status


@app.get("/move/{linear_x}&{angular_z}")
def move(linear_x: float = 0.5, angular_z: float = 0.0):
    if robot_status=="intiallized":
        lifecycle_client.publish_velocity(linear_x, angular_z)
        return {"status": "moving", "linear_x": linear_x, "angular_z": angular_z}
    else:
        return {"status": "error", "linear_x": None, "angular_z": None}


@app.get("/localization/launch")
async def launch_localization():
    """Launch the robot simulation"""
    success = Process_launch("localization")
    if success:
        return {"status": "success", "message": "localization launched"}
    else:
        raise HTTPException(status_code=409, detail="localization is already running")


@app.get("/localization/stop")
async def stop_localization():
    """Stop the robot simulation"""
    success = terminate_process("localization")
    if success:
        return {"status": "success", "message": "localization stopped"}
    else:
        return {"status": "warning", "message": "No robot localization running"}


@app.get("/localization/intiallized")
async def localization_intialized():
    global localization_status
    localization_status="intiallized"
    print(localization_status)
    return "registered"

@app.get("/localization/status")
async def localization_statu():
    global localization_status
    return localization_status


@app.get("/navigation/launch")
async def launch_navigation():
    """Launch the robot navigation"""
    success = Process_launch("navigation")
    if success:
        return {"status": "success", "message": "navigation launched"}
    else:
        raise HTTPException(status_code=409, detail="navigation is already running")


@app.get("/navigation/stop")
async def stop_navigation():
    """Stop the robot simulation"""
    success = terminate_process("navigation")
    if success:
        return {"status": "success", "message": "navigation stopped"}
    else:
        return {"status": "warning", "message": "No robot navigation running"}


@app.get("/navigation/intiallized")
async def navigation_intialized():
    global navigation_status
    navigation_status="intiallized"
    print(navigation_status)
    return "registered"
@app.get("/navigation/status")
async def navigation_statu():
    global navigation_status
    return navigation_status


# New endpoint to check lifecycle states of amcl and map_server
@app.get("/lifecycle/status")
async def check_lifecycle():
    """Check if all required nodes are in active state."""
    try:
        # Get localization nodes states
        amcl_state = await lifecycle_client.get_node_state(lifecycle_client.amcl_client)
        map_server_state = await lifecycle_client.get_node_state(lifecycle_client.map_server_client)
        
        # Get navigation nodes states
        controller_state = await lifecycle_client.get_node_state(lifecycle_client.controller_client)
        planner_state = await lifecycle_client.get_node_state(lifecycle_client.planner_client)
        behavior_state = await lifecycle_client.get_node_state(lifecycle_client.behavior_client)
        bt_navigator_state = await lifecycle_client.get_node_state(lifecycle_client.bt_navigator_client)
        
        # Check if all nodes are active
        all_active = all(state == "ACTIVE" for state in [
            amcl_state, 
            map_server_state,
            controller_state,
            planner_state,
            behavior_state,
            bt_navigator_state,
        ])
        
        if all_active:
            return {
                "status": "success",
                "message": "all nodes are active",
                "details": {
                    "localization": {
                        "amcl": amcl_state,
                        "map_server": map_server_state
                    },
                    "navigation": {
                        "controller": controller_state,
                        "planner": planner_state,
                        "behavior": behavior_state,
                        "bt_navigator": bt_navigator_state
                    }
                }
            }
        else:
            return {
                "status": "error",
                "message": "Not all nodes are active",
                "details": {
                    "localization": {
                        "amcl": amcl_state,
                        "map_server": map_server_state
                    },
                    "navigation": {
                        "controller": controller_state,
                        "planner": planner_state,
                        "behavior": behavior_state,
                        "bt_navigator": bt_navigator_state
                    }
                }
            }
    except Exception as e:
        return {
            "status": "error",
            "message": f"Error checking lifecycle states: {str(e)}"
        }


# NEW ENDPOINT: Set initial pose for AMCL
@app.post("/localization/set_pose")
async def set_initial_pose(pose: Pose2D):
    """Send a 2D pose estimate to AMCL"""
    # Check if localization is running
    if localization_status != "intiallized":
        return {
            "status": "error",
            "message": "Localization system is not initialized"
        }
    
    try:
        # Send the pose to AMCL via ROS2
        success = lifecycle_client.publish_initial_pose(pose.x, pose.y, pose.theta)
        
        if success:
            return {
                "status": "success",
                "message": f"Initial pose set: x={pose.x}, y={pose.y}, theta={pose.theta}"
            }
        else:
            return {
                "status": "error",
                "message": "Failed to publish initial pose"
            }
    except Exception as e:
        return {
            "status": "error",
            "message": f"Error setting initial pose: {str(e)}"
        }

# NEW ENDPOINT: Send navigation goal to Nav2
@app.post("/navigation/send_goal")
async def send_navigation_goal(goal: NavGoal):
    """Send a navigation goal to Nav2"""
    # Check if navigation is running
    if navigation_status != "intiallized":
        return {
            "status": "error",
            "message": "Navigation system is not initialized"
        }
    
    try:
        # Send the goal to Nav2 via ROS2
        success, message = await lifecycle_client.send_nav_goal(
            goal.x, goal.y, goal.theta, goal.frame_id
        )
        
        if success:
            return {
                "status": "success",
                "message": f"Navigation goal sent: x={goal.x}, y={goal.y}, theta={goal.theta}",
                "details": message
            }
        else:
            return {
                "status": "error",
                "message": message
            }
    except Exception as e:
        return {
            "status": "error",
            "message": f"Error sending navigation goal: {str(e)}"
        }
    
@app.get("/mapping/launch")
async def launch_mapping():
    """Launch the SLAM toolbox for mapping"""
    success = Process_launch("mapping")
    if success:
        return {"status": "success", "message": "SLAM toolbox mapping launched"}
    else:
        raise HTTPException(status_code=409, detail="Mapping is already running")

@app.get("/mapping/stop")
async def stop_mapping():
    """Stop the SLAM toolbox mapping process"""
    success = terminate_process("mapping")
    if success:
        return {"status": "success", "message": "Mapping stopped"}
    else:
        return {"status": "warning", "message": "No mapping process running"}

@app.get("/mapping/intiallized")
async def mapping_intialized():
    global mapping_status
    mapping_status = "intiallized"
    print(mapping_status)
    return "registered"

@app.get("/mapping/status")
async def mapping_statu():
    return mapping_status

@app.post("/mapping/save")
async def save_map(map_name: str = Body(..., embed=True)):
    """Save the current map to disk with the specified name"""
    if mapping_status != "intiallized":
        return {
            "status": "error",
            "message": "Mapping system is not initialized"
        }
    
    try:
        # Call the save_map service from Nav2/SLAM toolbox
        map_client = lifecycle_client.create_client(SaveMap, '/slam_toolbox/save_map')
        
        if not map_client.wait_for_service(timeout_sec=1.0):
            return {
                "status": "error",
                "message": "Save map service is not available"
            }
        
        # Create request
        request = SaveMap.Request()
        request.map_url = map_name  # Map name without extension
        
        # Send request
        future = map_client.call_async(request)
        
        # We can't await directly due to rclpy's event loop, so we'll return immediately
        # The service call will complete in the background
        return {
            "status": "success",
            "message": f"Map saving initiated with name: {map_name}",
            "details": "The map will be saved to the ROS package directory."
        }
    except Exception as e:
        return {
            "status": "error",
            "message": f"Error saving map: {str(e)}"
        }

@app.post("/mapping/serialize_map")
async def serialize_map(map_name: str = Body(..., embed=True)):
    """Serialize the current map to a file with the specified name"""
    if mapping_status != "intiallized":
        return {
            "status": "error",
            "message": "Mapping system is not initialized"
        }
    
    try:
        # Call the serialize_map service from SLAM toolbox
        serialize_client = lifecycle_client.create_client(Empty, '/slam_toolbox/serialize_map')
        
        if not serialize_client.wait_for_service(timeout_sec=1.0):
            return {
                "status": "error",
                "message": "Serialize map service is not available"
            }
        
        # Create request
        request = Empty.Request()
        
        # Send request
        future = serialize_client.call_async(request)
        
        # We can't await directly due to rclpy's event loop, so we'll return immediately
        return {
            "status": "success",
            "message": f"Map serialization initiated",
            "details": "The serialized map will be saved according to SLAM toolbox configuration."
        }
    except Exception as e:
        return {
            "status": "error",
            "message": f"Error serializing map: {str(e)}"
        }


if __name__ == '__main__':
    # Make sure to install dependencies: pip install fastapi uvicorn psutil
    uvicorn.run("robot_s1:app", host="0.0.0.0", port=8000, reload=True)