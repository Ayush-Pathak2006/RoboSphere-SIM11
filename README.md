# Auto-Driving Robot Controller

## Purpose

This repository contains code to control a 2D robot simulator via Flask API and WebSocket.  
It uses dynamic pathfinding (A* algorithm) to navigate a moving goal while avoiding obstacles detected via canvas image capture.

## Installation Dependencies

Python 3.8 or higher is required.

Install required packages with:

pip install -r requirements.txt


## Configuration

- The controller communicates with the simulator backend running on:
  - API Server: `http://localhost:5001`
  - WebSocket Server: `ws://localhost:8080`

- The canvas size is fixed at 650x600 pixels.

- Obstacle and robot sizes are predefined as per simulator defaults.

## Running the Robot Controller

Run the robot controller with the unified command:

python run_robot.py


The script will:

- Connect to the API server
- Capture canvas snapshot to detect obstacles
- Run the dynamic A* pathfinding and follow the moving goal
- Print progress and stop upon reaching the goal or failure

## Expected Output

Sample console output during run:

🤖 Dynamic A* Controller Initializing...
Capturing initial canvas to detect obstacles...
🗺️ Collision grid created based on visual detection.
Sent move command to: (123, 456)
✅ Path found and smoothed to 10 waypoints.
🏆 Goal has been reached! Controller is stopping.


## Notes

- Ensure the simulator backend and front-end are running before starting the controller.
- Adjust `API_BASE_URL` in `controller.py` if your server runs on a different host or port.
