#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
# Import the Action definition
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionServer
import numpy as np
import heapq
from math import sqrt

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.get_logger().info("A* Planner node is initializing...")
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.get_logger().info("Subscribed to /map topic.")
        
        # Action Server for ComputePathToPose
        # The action server will handle requests to compute a path
        self.action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose', # The name of the action
            self.execute_callback # The callback function to execute the action
        )
        self.get_logger().info("Created ComputePathToPose action server.")
        
        # Publisher for the global path (for visualization and local planner)
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.get_logger().info("Created /global_path publisher.")
        
        self.map_data = None
        self.map_info = None
        self.get_logger().info("A* Planner node initialized successfully.")
    
    def map_callback(self, msg):
        """
        Callback for the OccupancyGrid map.
        Reshapes the map data for easier access.
        """
        self.get_logger().info("Received map data in map_callback.")
        try:
            self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
            self.map_info = msg.info
            self.get_logger().info(f"Map loaded: {self.map_info.width}x{self.map_info.height} at {self.map_info.resolution} m/pixel.")
        except Exception as e:
            self.get_logger().error(f"Error processing map data: {e}")
    
    def world_to_grid(self, world_x, world_y):
        """Converts world coordinates to grid coordinates."""
        if self.map_info is None:
            self.get_logger().error("Map info not available for world_to_grid conversion.")
            return None, None
        grid_x = int((world_x - self.map_info.origin.position.x) / self.map_info.resolution)
        grid_y = int((world_y - self.map_info.origin.position.y) / self.map_info.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Converts grid coordinates to world coordinates."""
        if self.map_info is None:
            self.get_logger().error("Map info not available for grid_to_world conversion.")
            return None, None
        world_x = grid_x * self.map_info.resolution + self.map_info.origin.position.x
        world_y = grid_y * self.map_info.resolution + self.map_info.origin.position.y
        return world_x, world_y
    
    def heuristic(self, a, b):
        """Euclidean distance heuristic for A*."""
        return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    
    def is_valid(self, x, y):
        """Checks if a grid cell is within map bounds and not an obstacle."""
        if self.map_info is None or self.map_data is None:
            self.get_logger().warn("is_valid called before map data is available.")
            return False # Cannot validate without map data
        if (x < 0 or x >= self.map_info.width or 
            y < 0 or y >= self.map_info.height):
            return False
        # Assuming map_data values: 0-49 are free, 50-99 are unknown, 100 are occupied
        return self.map_data[y, x] < 50  # Assuming < 50 is free space
    
    def astar(self, start, goal):
        """
        A* pathfinding algorithm.
        Returns a list of (grid_x, grid_y) tuples representing the path.
        """
        self.get_logger().info(f"A* search started from {start} to {goal}.")
        if not self.is_valid(start[0], start[1]):
            self.get_logger().warn(f"A* start point {start} is invalid (obstacle or out of bounds).")
            return None
        if not self.is_valid(goal[0], goal[1]):
            self.get_logger().warn(f"A* goal point {goal} is invalid (obstacle or out of bounds).")
            return None

        open_set = []
        heapq.heappush(open_set, (0, start)) # (f_score, node)
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        # 8-directional movement
        directions = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        
        while open_set:
            current_f_score, current = heapq.heappop(open_set)
            
            # If we've already found a better path to 'current', skip
            if current_f_score > f_score.get(current, float('inf')):
                continue

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                self.get_logger().info(f"A* found path with {len(path)} points.")
                return path[::-1] # Reverse to get path from start to goal
            
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not self.is_valid(neighbor[0], neighbor[1]):
                    continue
                
                # Cost to move to neighbor (diagonal moves have sqrt(2) cost)
                move_cost = self.heuristic(current, neighbor) # distance between current and neighbor
                tentative_g_score = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        self.get_logger().warn("A* failed to find a path.")
        return None # No path found
    
    def execute_callback(self, goal_handle):
        """
        Callback for the ComputePathToPose action.
        This function is executed when a new goal request is received.
        """
        self.get_logger().info("Received path computation action goal.")
        request = goal_handle.request # The goal request contains start and goal poses

        if self.map_data is None or self.map_info is None:
            self.get_logger().warn("Map data not available. Cannot compute path.")
            goal_handle.abort() # Abort the action
            return ComputePathToPose.Result() # Return an empty result

        start_x, start_y = self.world_to_grid(
            request.start.pose.position.x, request.start.pose.position.y)
        goal_x, goal_y = self.world_to_grid(
            request.goal.pose.position.x, request.goal.pose.position.y)

        if start_x is None or start_y is None or goal_x is None or goal_y is None:
            self.get_logger().error("Failed to convert world coordinates to grid coordinates. Check map info.")
            goal_handle.abort()
            return ComputePathToPose.Result()

        self.get_logger().info(f"Attempting to compute path from world ({request.start.pose.position.x:.2f},{request.start.pose.position.y:.2f}) [grid: ({start_x},{start_y})] to world ({request.goal.pose.position.x:.2f},{request.goal.pose.position.y:.2f}) [grid: ({goal_x},{goal_y})].")
        
        path = self.astar((start_x, start_y), (goal_x, goal_y))
        
        result = ComputePathToPose.Result()
        feedback = ComputePathToPose.Feedback()

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        if path:
            for grid_x, grid_y in path:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                world_x, world_y = self.grid_to_world(grid_x, grid_y)
                if world_x is None or world_y is None:
                    self.get_logger().error("Failed to convert grid coordinates to world coordinates during path construction. Path aborted.")
                    path_msg.poses = [] # Clear path if conversion fails
                    break
                pose.pose.position.x = world_x
                pose.pose.position.y = world_y
                pose.pose.orientation.w = 1.0 # Default to no rotation
                path_msg.poses.append(pose)
            
            result.path = path_msg
            self.path_pub.publish(path_msg) # Publish the path for the local planner
            self.get_logger().info(f"Published global path with {len(path_msg.poses)} poses.")
            
            # Send feedback (optional, but good for long-running actions)
            feedback.partial_path = path_msg
            goal_handle.publish_feedback(feedback)

            goal_handle.succeed() # Mark the action as successful
        else:
            self.get_logger().warn("A* planner did not find a path. Aborting action.")
            goal_handle.abort() # Mark the action as aborted
        
        return result # Return the result of the action

def main(args=None):
    rclpy.init(args=args)
    astar_planner = AStarPlanner()
    rclpy.spin(astar_planner)
    astar_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
