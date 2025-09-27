# File: pathfinding_controller.py

import requests
import time
import heapq
import math

# In controller.py, at the top
from PIL import Image
import base64
import io

# --- Configuration ---
API_BASE_URL = "http://localhost:5001"
CANVAS_WIDTH = 650
CANVAS_HEIGHT = 600
ROBOT_RADIUS = 18
OBSTACLE_SIZE = 25 
CELL_SIZE = 20 # The grid cell size for pathfinding
# Safety margin around obstacles (obstacle radius + robot radius + buffer)
OBSTACLE_CLEARANCE = (OBSTACLE_SIZE / 2) + ROBOT_RADIUS + 5 

# --- 1. API Client: Interacts with the Flask Server ---
# In controller.py, replace your existing APIClient class with this one:

class APIClient:
    """A simple client to communicate with the robot simulator's API."""
    def __init__(self, base_url):
        self.base_url = base_url

    def get_current_state(self):
        """Fetches robot and goal state using the /capture endpoint."""
        try:
            response = requests.get(f"{self.base_url}/capture", timeout=3)
            if response.status_code == 200:
                return response.json()
            return None
        except requests.exceptions.RequestException:
            return None

    # THIS IS THE MISSING FUNCTION
    def get_collision_count(self):
        """Fetches the current collision count from the /collisions endpoint."""
        try:
            response = requests.get(f"{self.base_url}/collisions", timeout=1)
            if response.status_code == 200:
                return response.json().get('count', 0)
        except requests.exceptions.RequestException:
            # If server is down, assume 0
            return 0
        return 0

    def move_robot_to(self, x, y):
        """Sends a command to move the robot to an absolute position."""
        try:
            requests.post(f"{self.base_url}/move", json={"x": x, "y": y}, timeout=2)
            print(f"Sent move command to: ({int(x)}, {int(y)})")
        except requests.exceptions.RequestException as e:
            print(f"Error sending move command: {e}")

    def set_random_obstacles(self, count=42):
        """Generates random obstacles and returns their positions."""
        try:
            response = requests.post(f"{self.base_url}/obstacles/random", json={"count": count})
            if response.status_code == 200:
                print(f"Successfully set {count} random obstacles.")
                return response.json().get("obstacles", [])
        except requests.exceptions.RequestException as e:
            print(f"Error setting obstacles: {e}")
        return []

    # In controller.py, inside the APIClient class
    def get_goal_status(self):
        """Checks if the goal has been reached."""
        try:
            response = requests.get(f"{self.base_url}/goal/status", timeout=1)
            if response.status_code == 200:
                return response.json().get('goal_reached', False)
        except requests.exceptions.RequestException:
            pass
        return False



# --- 2. A* Pathfinding Implementation (Corrected & Improved from your snippet) ---
class Node:
    """Represents a node in the pathfinding grid."""
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to this node
        self.h = 0  # Heuristic cost from goal
        self.f = 0  # Total cost (g + h)

    # Corrected __eq__ for comparing nodes
    def __eq__(self, other):
        return self.position == other.position
    
    # Added for heapq to handle nodes with the same 'f' value
    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    """Calculates the Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(grid, start, end):
    """Finds the shortest path using the A* algorithm."""
    start_node = Node(start)
    goal_node = Node(end)
    
    open_list = []
    # Using a set for closed_list is much faster for lookups
    closed_set = set()
    
    # heapq stores tuples; we push the node's 'f' value for priority
    heapq.heappush(open_list, (start_node.f, start_node))
    
    while open_list:
        # Get the node with the lowest 'f' score
        current_node = heapq.heappop(open_list)[1]
        
        # If we've already processed this node, skip it
        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)
        
        # Path found
        if current_node == goal_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        (x, y) = current_node.position
        # Check all 8 neighbors for more fluid movement
        for move_x, move_y in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_pos = (x + move_x, y + move_y)
            if (0 <= new_pos[0] < len(grid[0]) and 0 <= new_pos[1] < len(grid)) and grid[new_pos[1]][new_pos[0]] == 0:
                neighbor = Node(new_pos, current_node)
                neighbor.g = current_node.g + 1
                neighbor.h = heuristic(neighbor.position, goal_node.position)
                neighbor.f = neighbor.g + neighbor.h
                heapq.heappush(open_list, (neighbor.f, neighbor))

        # Check diagonal directions
        for move_x, move_y in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_pos = (x + move_x, y + move_y)
            # Check that the diagonal is clear AND the path to it isn't blocked by corners
            if (0 <= new_pos[0] < len(grid[0]) and 0 <= new_pos[1] < len(grid)) and grid[new_pos[1]][new_pos[0]] == 0:
                # PREVENT CORNER CUTTING: Check adjacent cells
                if grid[y + move_y][x] == 0 and grid[y][x + move_x] == 0:
                    neighbor = Node(new_pos, current_node)
                    neighbor.g = current_node.g + 1.414 # Diagonal cost
                    neighbor.h = heuristic(neighbor.position, goal_node.position)
                    neighbor.f = neighbor.g + neighbor.h
                    heapq.heappush(open_list, (neighbor.f, neighbor))

    return None # No path found

# --- 3. Grid and Coordinate Helpers ---
def create_grid(obstacles):
    """Creates a 2D grid representing the canvas, marking obstacles."""
    grid_width = CANVAS_WIDTH // CELL_SIZE
    grid_height = CANVAS_HEIGHT // CELL_SIZE
    grid = [[0 for _ in range(grid_width)] for _ in range(grid_height)]
    
    for obs in obstacles:
        for r in range(grid_height):
            for c in range(grid_width):
                cell_center_x = c * CELL_SIZE + CELL_SIZE / 2
                cell_center_y = r * CELL_SIZE + CELL_SIZE / 2
                distance = math.sqrt((cell_center_x - obs['x'])**2 + (cell_center_y - obs['y'])**2)
                if distance < OBSTACLE_CLEARANCE:
                    grid[r][c] = 1 # Mark as unwalkable
    return grid

def pixel_to_grid(pixel_pos):
    """Converts pixel coordinates to grid coordinates."""
    return (int(pixel_pos['x'] / CELL_SIZE), int(pixel_pos['y'] / CELL_SIZE))

def grid_to_pixel(grid_pos):
    """Converts grid coordinates to the center pixel of the cell."""
    return (grid_pos[0] * CELL_SIZE + CELL_SIZE / 2, grid_pos[1] * CELL_SIZE + CELL_SIZE / 2)

def print_grid_to_terminal(grid, start_pos, end_pos):
    """Prints a visual representation of the grid to the console."""
    print("--- Algorithm's View of the Maze ---")
    for r_idx, row in enumerate(grid):
        row_str = ""
        for c_idx, cell in enumerate(row):
            if (c_idx, r_idx) == start_pos:
                row_str += " R " # Robot
            elif (c_idx, r_idx) == end_pos:
                row_str += " G " # Goal
            elif cell == 1:
                row_str += "[#]" # Obstacle
            else:
                row_str += " . " # Walkable path
        print(row_str)
    print("------------------------------------")

def has_line_of_sight(grid, start_pos, end_pos):
    """Checks if there is a direct, obstacle-free line between two grid points."""
    x0, y0 = start_pos
    x1, y1 = end_pos
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = 1 if x0 < x1 else -1, 1 if y0 < y1 else -1
    err = dx + dy
    
    while True:
        if grid[y0][x0] == 1: # Check for obstacle
            return False
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return True

# NEW FUNCTION TO SMOOTH THE A* PATH
def smooth_path(path, grid):
    """Removes unnecessary waypoints from a path if a direct line of sight exists."""
    if not path or len(path) < 3:
        return path
    
    smoothed_path = [path[0]]
    current_index = 0
    
    while current_index < len(path) - 1:
        for lookahead_index in range(len(path) - 1, current_index, -1):
            if has_line_of_sight(grid, path[current_index], path[lookahead_index]):
                smoothed_path.append(path[lookahead_index])
                current_index = lookahead_index
                break
        else:
            # Failsafe in case no line of sight is found
            current_index += 1
            if current_index < len(path):
                smoothed_path.append(path[current_index])
    
    return smoothed_path

# In controller.py, add this new function

def detect_obstacles_from_image(base64_data):
    """
    Decodes a base64 image string and detects obstacles by their color.
    Returns a list of obstacle coordinates.
    """
    # 1. Decode the base64 string
    # The string format is "data:image/png;base64, ACTUAL_DATA"
    header, encoded = base64_data.split(",", 1)
    image_data = base64.b64decode(encoded)
    image = Image.open(io.BytesIO(image_data)).convert("RGB")
    pixels = image.load()
    
    width, height = image.size
    obstacles = []
    
    # 2. A set to keep track of pixels we've already assigned to an obstacle
    visited = set()

    # 3. Scan the image for dark pixels
    # We scan every 10 pixels for efficiency
    for x in range(0, width, 10):
        for y in range(0, height, 10):
            if (x, y) in visited:
                continue

            r, g, b = pixels[x, y]
            
            # Obstacles are dark grey. We check if the pixel is dark enough.
            # The color is roughly (47,53,66), so a sum below 200 is a safe bet.
            if r + g + b < 200:
                # We found part of an obstacle!
                # Now we find the center of this obstacle blob
                blob_pixels = []
                q = [(x,y)] # Queue for flood-fill
                visited.add((x,y))
                
                head = 0
                while head < len(q):
                    px, py = q[head]
                    head += 1
                    blob_pixels.append((px,py))
                    
                    # Check neighbors
                    for nx in range(px-10, px+11, 10):
                        for ny in range(py-10, py+11, 10):
                            if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
                                nr, ng, nb = pixels[nx, ny]
                                if nr + ng + nb < 200:
                                    visited.add((nx, ny))
                                    q.append((nx,ny))

                # Calculate the center of the detected blob
                if blob_pixels:
                    avg_x = sum(p[0] for p in blob_pixels) / len(blob_pixels)
                    avg_y = sum(p[1] for p in blob_pixels) / len(blob_pixels)
                    obstacles.append({'x': avg_x, 'y': avg_y})

    print(f"👁️  Vision system detected {len(obstacles)} obstacles.")
    return obstacles
def set_obstacle_positions(self, obstacles):
        """Sends a specific list of obstacle positions to the server."""
        payload = {"obstacles": obstacles}
        try:
            response = requests.post(f"{self.base_url}/obstacles/positions", json=payload)
            if response.status_code == 200:
                print(f"Successfully set {len(obstacles)} obstacles from image file.")
                return True
        except requests.exceptions.RequestException as e:
            print(f"Error setting obstacle positions: {e}")
        return False

# --- 4. Main Controller Logic ---
def run_controller():
    """The main control loop for the dynamic pathfinding agent."""
    api = APIClient(API_BASE_URL)
    
    print("🤖 Dynamic A* Controller Initializing...")

    print("Capturing initial canvas to detect obstacles...")
    initial_state = None
    while not initial_state:
        initial_state = api.get_current_state()
        if not initial_state:
            print("Waiting for simulator to connect to get first image...")
            time.sleep(1)

    # 2. Detect obstacles from the image
    obstacles = detect_obstacles_from_image(initial_state['image_data'])
    if not obstacles:
        print("⚠️ No obstacles detected. Assuming an open field.")
    
    # 3. Create the grid based on what the controller "sees"
    collision_grid = create_grid(obstacles)
    print("🗺️  Collision grid created based on visual detection.")


    obstacles = api.set_random_obstacles(8)
    if not obstacles:
        print("❌ Could not set obstacles. Is the server running?")
        print_grid_to_terminal(collision_grid, start_grid, end_grid)
        # //remove this
        return
        
    collision_grid = create_grid(obstacles)
    print("🗺️  Collision grid created.")
    
    last_goal_pos = None
    # current_path = []
    last_collision_count = api.get_collision_count()
    print(f"Initial collision count: {last_collision_count}")
    
    while True:

        if api.get_goal_status():
            print("🏆 Goal has been reached! Controller is stopping.")
            break # Exit the chase loop

        state = api.get_current_state()
        
        if not state or state.get('status') != 'success':
            print("Waiting for simulator to connect...")
            time.sleep(2)
            continue
            
        robot_pos = state.get('robot_position')
        goal_pos = state.get('goal_position')
        current_collisions = api.get_collision_count()

        goal_has_moved = goal_pos != last_goal_pos
        new_collision_detected = current_collisions > last_collision_count

        # Only re-plan if needed
        if goal_has_moved or new_collision_detected:
            if goal_has_moved:
                print(f"🎯 New goal detected at {goal_pos}. Recalculating path...")
                last_goal_pos = goal_pos
            if new_collision_detected:
                print(f"💥 Collision detected! (Count: {current_collisions}). Finding new route...")
                last_collision_count = current_collisions
            
            # 1. Find the raw path
            start_grid, end_grid = pixel_to_grid(robot_pos), pixel_to_grid(goal_pos)
            raw_path = a_star_search(collision_grid, start_grid, end_grid)
            
            if raw_path:
                # 2. Smooth the path
                path_waypoints = smooth_path(raw_path, collision_grid)
                print(f"✅ Path found and smoothed to {len(path_waypoints)} waypoints.")
                
                # 3. Execute the smoothed path
                for waypoint in path_waypoints[1:]: # Skip the first waypoint (current pos)
                    target_x, target_y = grid_to_pixel(waypoint)
                    api.move_robot_to(target_x, target_y)
                    
                    # INTELLIGENT WAITING LOOP
                    while True:
                        time.sleep(0.1) # Poll every 100ms
                        live_state = api.get_current_state()
                        if not live_state: break
                        
                        live_robot_pos = live_state.get('robot_position')
                        dist_to_target = math.sqrt((live_robot_pos['x'] - target_x)**2 + (live_robot_pos['y'] - target_y)**2)
                        
                        # Check if robot has arrived or a new collision happened
                        if dist_to_target < 20 or api.get_collision_count() > last_collision_count:
                            break
                        
                         
            else:
                print("❌ No path to goal could be found.")
                print_grid_to_terminal(collision_grid, start_grid, end_grid)
                time.sleep(1) # Wait before trying again if no path is found
        else:
            time.sleep(0.5) # Wait before polling again if nothing has changed

if __name__ == "__main__":
    run_controller()