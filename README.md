# ROS 2 Robot Control Server

[![Task Demo Video](https://img.youtube.com/vi/Hc1XsQzDstM/0.jpg)](https://youtu.be/Hc1XsQzDstM)

Click the image above to watch the task demonstration video.

## Overview

This project implements a FastAPI-based server for managing a ROS 2 robot system, offering a comprehensive web API to control various aspects of robot operation including:

- Robot simulation
- Localization
- Navigation
- Mapping (SLAM)

The server facilitates launching and stopping various ROS 2 processes, monitoring node lifecycle states, and providing easy control over robot movement and navigation through HTTP endpoints.

## System Architecture

The system consists of several main components:

1. **FastAPI Server**: Provides HTTP endpoints for controlling all robot functionality
2. **ROS 2 Integration**: Manages lifecycle nodes and messaging between ROS 2 and the API
3. **Simulation Management**: Controls Gazebo simulation for the robot
4. **Navigation Stack**: Manages localization, path planning, and movement control

## Key Components

### Main Server Modules

- `robot_server.py`: The original implementation with basic functionality
- `robot_s1.py`: Enhanced implementation with complete navigation and mapping capabilities
- `launch_sim.py`: Handles the simulation launch process
- `localization_launch_nav.py`: Manages localization nodes (AMCL, map server)
- `navigation_launch.py`: Controls the ROS 2 Navigation2 stack components
- `online_async_launch.py`: Manages SLAM toolbox for mapping

## API Endpoints

### Robot Control

- `GET /robot/launch`: Start the robot simulation
- `GET /robot/stop`: Stop the robot simulation
- `GET /robot/status`: Get current simulation status
- `GET /robot/process/status`: Get status of all managed processes
- `GET /move/{linear_x}&{angular_z}`: Move robot with specified velocity

### Localization

- `GET /localization/launch`: Start the localization system
- `GET /localization/stop`: Stop the localization system
- `GET /localization/status`: Check localization status
- `POST /localization/set_pose`: Set initial pose estimate for AMCL

### Navigation

- `GET /navigation/launch`: Start the navigation system
- `GET /navigation/stop`: Stop the navigation system
- `GET /navigation/status`: Check navigation status
- `POST /navigation/send_goal`: Send navigation goal to Nav2

### Mapping

- `GET /mapping/launch`: Launch the SLAM toolbox for mapping
- `GET /mapping/stop`: Stop the SLAM toolbox mapping process
- `GET /mapping/status`: Check mapping status
- `POST /mapping/save`: Save the current map using map_saver_cli
- `POST /mapping/serialize_map`: Serialize the current map to a file

## System Lifecycle Management

The system includes comprehensive lifecycle management for ROS 2 nodes:

1. Nodes are launched using ROS 2 launch files
2. The server tracks process state and can safely terminate processes
3. Node lifecycles are monitored (unconfigured, inactive, active, finalized)
4. Status callbacks update the server when processes initialize successfully

## Process Registry

The server maintains a registry of running processes:
```python
process_registry = {
    "simulation": None,
    "localization": None,
    "navigation": None,
    "mapping": None
}
```

## Lifecycle Node Management

For proper operation and monitoring, the system tracks lifecycle node statuses:
- Localization nodes (map_server, amcl)
- Navigation nodes (controller, planner, behavior, bt_navigator)
- Mapping nodes (slam_toolbox)

## Usage

### Starting the Server

```bash
# Install dependencies
pip install fastapi uvicorn psutil rclpy

# Run the server
python robot_s1.py
```

### Example API Usage

```bash
# Start simulation
curl http://localhost:8000/robot/launch

# Start localization
curl http://localhost:8000/localization/launch

# Set initial pose
curl -X POST http://localhost:8000/localization/set_pose \
  -H "Content-Type: application/json" \
  -d '{"x": 1.0, "y": 0.5, "theta": 0.0}'

# Start navigation
curl http://localhost:8000/navigation/launch

# Send navigation goal
curl -X POST http://localhost:8000/navigation/send_goal \
  -H "Content-Type: application/json" \
  -d '{"x": 5.0, "y": 5.0, "theta": 0.0, "frame_id": "map"}'
```

## Requirements

- ROS 2 (tested with Humble)
- FastAPI
- Uvicorn
- Python 3.8+
- ROS 2 Navigation2 stack
- SLAM Toolbox
- Gazebo Harmonic

## License

This project uses code from multiple sources:
- Navigation2 code is licensed under Apache License 2.0
- Custom code is available under MIT License unless otherwise specified

## Notes

- This server should be run on the same machine as ROS 2
- All endpoints return JSON responses
- Error handling is implemented for most failure cases
- The system uses multiprocessing to manage ROS 2 launch files
- Status callbacks ensure synchronization between processes and the API