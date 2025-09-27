import asyncio
import json
import websockets
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import random
import time

# --- Flask App Setup ---
app = Flask(__name__)
CORS(app) # Enable Cross-Origin Resource Sharing

# --- Globals ---
connected_simulators = set()
async_loop = None
collision_count = 0
goal_reached = False
latest_canvas_capture = None

# --- WebSocket Handler ---
async def ws_handler(websocket, path=None):
    global collision_count, goal_reached, latest_canvas_capture
    connected_simulators.add(websocket)
    print("Simulator connected via WebSocket.")
    try:
        async for message in websocket:
            data = json.loads(message)
            if data.get("type") == "collision":
                collision_count += 1
                print(f"Collision detected! Total: {collision_count}")
            elif data.get("type") == "goal_reached":
                goal_reached = True
                print("Goal has been reached!")
            elif data.get("type") == "canvas_captured":
                latest_canvas_capture = data
                print("Canvas captured.")
    except websockets.exceptions.ConnectionClosed:
        print("Simulator disconnected.")
    finally:
        connected_simulators.remove(websocket)

def broadcast_to_simulators(message):
    if not connected_simulators:
        return
    for websocket in connected_simulators:
        asyncio.run_coroutine_threadsafe(
            websocket.send(json.dumps(message)), async_loop
        )

# --- API Endpoints ---
@app.route('/move', methods=['POST'])
def move():
    global goal_reached
    goal_reached = False
    data = request.get_json()
    broadcast_to_simulators({"command": "move", "target": data})
    return jsonify({"status": "move command sent"})

@app.route('/obstacles/random', methods=['POST'])
def set_random_obstacles():
    data = request.get_json() or {}
    count = data.get('count', 8)
    obstacles = []
    for _ in range(count):
        obstacles.append({
            "x": random.randint(50, 600),
            "y": random.randint(50, 550)
        })
    broadcast_to_simulators({"command": "set_obstacles", "obstacles": obstacles})
    return jsonify({"status": "random obstacles set", "obstacles": obstacles})

@app.route('/capture', methods=['GET'])
def capture():
    global latest_canvas_capture
    latest_canvas_capture = None # Clear previous capture
    broadcast_to_simulators({"command": "capture_canvas"})
    
    # Wait up to 3 seconds for the simulator to respond
    timeout = time.time() + 3
    while time.time() < timeout:
        if latest_canvas_capture:
            return jsonify({**latest_canvas_capture, "status": "success"})
        time.sleep(0.1)
    return jsonify({"status": "error", "message": "Timeout waiting for simulator"}), 504

@app.route('/collisions', methods=['GET'])
def get_collisions():
    return jsonify({'count': collision_count})

# --- Server Threads ---
def run_flask_app():
    app.run(host='0.0.0.0', port=5001)

async def run_websocket_server():
    global async_loop
    async_loop = asyncio.get_running_loop()
    async with websockets.serve(ws_handler, "localhost", 8080):
        await asyncio.Future() # run forever

if __name__ == "__main__":
    print("🚀 Starting Server...")
    flask_thread = threading.Thread(target=run_flask_app, daemon=True)
    flask_thread.start()
    
    print("Flask API running on http://localhost:5001")
    print("WebSocket Server running on ws://localhost:8080")
    
    try:
        asyncio.run(run_websocket_server())
    except KeyboardInterrupt:
        print("🛑 Server stopped.")