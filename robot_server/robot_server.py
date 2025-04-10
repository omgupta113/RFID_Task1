from launch import LaunchService
import time
from multiprocessing import Process
import launch_sim
import localization_launch_nav
import navigation_launch
from fastapi import FastAPI, HTTPException
import uvicorn
import os
import signal
import psutil
import subprocess
import json
from typing import Dict, List, Optional


# Dictionary to keep track of running processes
process_registry = {
    "simulation": None,
    "localization": None,
    "navigation": None
}
robot_status = None

# Dictionary to track lifecycle node statuses
lifecycle_nodes = {
    "localization": {
        "map_server": "unknown",
        "amcl": "unknown",
        "lifecycle_manager_localization": "unknown"
    }
}

def run_launch_service(ld):
    ls = LaunchService()
    ls.include_launch_description(ld)
    # Run in the main thread of this process
    return ls.run()


def Process_launch(process_name, ld):
    if process_registry[process_name] is not None and process_registry[process_name].is_alive():
        print(f"{process_name} is already running.")
        return False
    
    p = Process(target=run_launch_service, args=(ld,))
    p.start()
    process_registry[process_name] = p
    print(f"Launch started: {process_name} (PID: {p.pid})")
    return True


def terminate_process(process_name):
    """Safely terminate a process and its children"""
    if process_registry[process_name] is None or not process_registry[process_name].is_alive():
        return False
    
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
        
        # Reset lifecycle node statuses if terminating localization
        if process_name == "localization":
            for node in lifecycle_nodes["localization"]:
                lifecycle_nodes["localization"][node] = "unknown"
                
        return True
    
    except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
        print(f"Error terminating {process_name}: {e}")
        # Still clear from registry as it might be zombie
        process_registry[process_name] = None
        return False


def get_lifecycle_node_status(node_name: str, namespace: str = "") -> str:
    """Get the lifecycle state of a node using ros2 command line"""
    try:
        full_node_name = f"{namespace}/{node_name}" if namespace else node_name
        cmd = f"ros2 lifecycle get {full_node_name}"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"Error getting lifecycle state for {full_node_name}: {result.stderr}")
            return "unknown"
        
        # Parse the output to extract the state
        output = result.stdout.strip()
        if "unconfigured" in output:
            return "unconfigured"
        elif "inactive" in output:
            return "inactive"
        elif "active" in output:
            return "active"
        elif "finalized" in output:
            return "finalized"
        else:
            return "unknown"
            
    except Exception as e:
        print(f"Exception getting lifecycle state for {node_name}: {e}")
        return "error"


def update_localization_node_statuses():
    """Update the status of all localization lifecycle nodes"""
    for node in lifecycle_nodes["localization"]:
        lifecycle_nodes["localization"][node] = get_lifecycle_node_status(node)
    
    # Check if all nodes are active
    all_active = all(status == "active" for status in lifecycle_nodes["localization"].values())
    any_error = any(status in ["unconfigured", "inactive"] for status in lifecycle_nodes["localization"].values())
    
    if all_active:
        return "active"
    elif any_error:
        return "error"
    else:
        return "initializing"


app = FastAPI(title="Robot Launch Server", 
              description="API to manage ROS 2 robot simulation and localization",
              version="1.0.0")


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
            {"path": "/localization/launch", "method": "GET", "description": "Start localization nodes"},
            {"path": "/localization/stop", "method": "GET", "description": "Stop localization nodes"},
            {"path": "/localization/status", "method": "GET", "description": "Get localization nodes status"}
        ]
    }

@app.get("/robot/launch")
async def launch_robot():
    """Launch the robot simulation"""
    success = Process_launch("simulation", launch_sim.generate_launch_description())
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


@app.get("/localization/launch")
async def launch_localization():
    """Launch the localization nodes"""
    if process_registry["simulation"] is None or not process_registry["simulation"].is_alive():
        raise HTTPException(status_code=400, detail="Simulation must be running before launching localization")
    
    success = Process_launch("localization", localization_launch_nav.generate_launch_description())
    if success:
        # Give nodes time to start up
        time.sleep(5)
        status = update_localization_node_statuses()
        return {"status": "success", "message": f"Localization launched with status: {status}"}
    else:
        raise HTTPException(status_code=409, detail="Localization is already running")


@app.get("/localization/stop")
async def stop_localization():
    """Stop the localization nodes"""
    success = terminate_process("localization")
    if success:
        return {"status": "success", "message": "Localization stopped"}
    else:
        return {"status": "warning", "message": "No localization running"}


@app.get("/localization/status")
async def get_localization_status():
    """Get the status of localization nodes"""
    if process_registry["localization"] is None or not process_registry["localization"].is_alive():
        return {
            "status": "stopped",
            "nodes": lifecycle_nodes["localization"]
        }
    
    # Update node statuses
    overall_status = update_localization_node_statuses()
    
    return {
        "status": overall_status,
        "nodes": lifecycle_nodes["localization"]
    }


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
    robot_status = "intiallized"
    print(robot_status)
    return "registered"


@app.get("/robot/error")
async def robot_error():
    global robot_status
    robot_status = "error"
    print(robot_status)
    return "registered"


@app.get("/robot/status")
async def robot_statu():
    return robot_status


if __name__ == '__main__':
    # Make sure to install dependencies: pip install fastapi uvicorn psutil
    uvicorn.run("robot_server:app", host="0.0.0.0", port=8000, reload=True)