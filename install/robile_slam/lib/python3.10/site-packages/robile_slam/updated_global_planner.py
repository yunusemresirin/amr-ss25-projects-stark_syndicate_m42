#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import heapq
import math
import numpy as np
from collections import deque
from std_msgs.msg import String

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.start_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.start_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.feedback_pub = self.create_publisher(String, '/goal_feedback', 10)
        self.map = None
        self.inflated_map_data = None
        self.start = None
        self.goal = None
        
        # Declare inflation parameter (in meters)
        self.declare_parameter('inflation_radius', 0.2)  # Reduced to 0.2m
        self.inflation_radius = self.get_parameter('inflation_radius').value
        
        self.get_logger().info('Global Planner node started. Waiting for map, start, and goal. Inflation radius: %.2f m' % self.inflation_radius)

    def map_callback(self, msg):
        self.get_logger().info('Received map data. Height: %d, Width: %d, Resolution: %.2f' % (msg.info.height, msg.info.width, msg.info.resolution))
        self.map = msg
        self.inflate_map()
        self.plan_if_possible()

    def start_callback(self, msg):
        self.get_logger().info('Received initial pose: Position (x: %.2f, y: %.2f)' % (msg.pose.pose.position.x, msg.pose.pose.position.y))
        self.start = msg.pose.pose
        self.plan_if_possible()

    def goal_callback(self, msg):
        self.get_logger().info('Received goal pose: Position (x: %.2f, y: %.2f)' % (msg.pose.position.x, msg.pose.position.y))
        self.goal = msg.pose
        self.plan_if_possible()

    def inflate_map(self):
        if self.map is None:
            self.get_logger().warn('Cannot inflate: No map available.')
            return
        
        res = float(self.map.info.resolution)
        inflation_cells = int(self.inflation_radius / res)
        self.get_logger().info('Inflating obstacles by %.2f m (%d cells).' % (self.inflation_radius, inflation_cells))
        
        width = self.map.info.width
        height = self.map.info.height
        data = np.array(self.map.data, dtype=np.int8).reshape(height, width)
        
        # Inflate occupied cells (>=50)
        occupied = (data >= 50)
        from scipy.ndimage import binary_dilation
        inflated = binary_dilation(occupied, structure=np.ones((2 * inflation_cells + 1, 2 * inflation_cells + 1)))
        self.inflated_map_data = np.where(inflated, 100, np.where(data == -1, -1, 0)).flatten()
        self.get_logger().info('Map inflation complete. Occupied cells after inflation: %d' % np.sum(self.inflated_map_data == 100))

    def plan_if_possible(self):
        if self.map is None:
            self.get_logger().warn('Cannot plan: Map not received yet.')
            return
        if self.inflated_map_data is None:
            self.get_logger().warn('Cannot plan: Inflated map not ready.')
            return
        if self.start is None:
            self.get_logger().warn('Cannot plan: Start pose not received yet.')
            return
        if self.goal is None:
            self.get_logger().warn('Cannot plan: Goal pose not received yet.')
            return
        
        self.get_logger().info('Starting path planning with A*...')
        start_grid = self.pose_to_grid(self.start)
        goal_grid = self.pose_to_grid(self.goal)
        self.get_logger().info('Start grid: (%d, %d), Goal grid: (%d, %d)' % (start_grid[0], start_grid[1], goal_grid[0], goal_grid[1]))
        
        start_value = self.get_map_value(start_grid)
        goal_value = self.get_map_value(goal_grid)
        self.get_logger().info('Map value at start: %d, at goal: %d' % (start_value, goal_value))
        
        # Adjust start to nearest free/unknown cell if in obstacle
        adjusted_start = start_grid
        if start_value >= 50:
            adjusted_start = self.find_nearest_free(start_grid)
            if adjusted_start is None:
                self.get_logger().warn('No free or unknown cell near start. Cannot plan.')
                feedback = String()
                feedback.data = "invalid_goal"
                self.feedback_pub.publish(feedback)
                return
            self.get_logger().info('Adjusted start to nearest valid cell: (%d, %d)' % adjusted_start)
        
        # Adjust goal to nearest free/unknown cell if in obstacle
        adjusted_goal = goal_grid
        if goal_value >= 50:
            adjusted_goal = self.find_nearest_free(goal_grid)
            if adjusted_goal is None:
                self.get_logger().warn('No free or unknown cell near goal. Cannot plan.')
                feedback = String()
                feedback.data = "invalid_goal"
                self.feedback_pub.publish(feedback)
                return
            self.get_logger().info('Adjusted goal to nearest valid cell: (%d, %d)' % adjusted_goal)
        
        path_grids = self.a_star(adjusted_start, adjusted_goal)
        if path_grids:
            self.get_logger().info('Path found with %d grid points.' % len(path_grids))
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses = [self.grid_to_pose(grid) for grid in path_grids]
            self.path_pub.publish(path_msg)
            self.get_logger().info('Path published to /global_path.')
        else:
            self.get_logger().warn('No path found after A* search.')
            feedback = String()
            feedback.data = "invalid_goal"
            self.feedback_pub.publish(feedback)

    def find_nearest_free(self, start):
        """Find the nearest free or unknown cell using BFS with a larger search radius."""
        width = self.map.info.width
        height = self.map.info.height
        x, y = start
        visited = set()
        queue = deque([(x, y, 0)])  # (x, y, distance)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        max_distance = 50  # Search up to 50 cells away
        while queue:
            cx, cy, dist = queue.popleft()
            if (cx, cy) in visited or dist > max_distance:
                continue
            visited.add((cx, cy))
            if 0 <= cx < width and 0 <= cy < height:
                idx = cy * width + cx
                if self.inflated_map_data[idx] <= 0:  # Free (0) or unknown (-1)
                    return (cx, cy)
                for dx, dy in directions:
                    queue.append((cx + dx, cy + dy, dist + 1))
        return None

    def a_star(self, start, goal):
        self.get_logger().debug('A* started from %s to %s' % (str(start), str(goal)))
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 8-way movement
        visited_count = 0
        while open_set:
            visited_count += 1
            if visited_count % 1000 == 0:
                self.get_logger().debug('A* processing: Visited %d nodes, open set size: %d' % (visited_count, len(open_set)))
            _, current = heapq.heappop(open_set)
            if current == goal:
                self.get_logger().info('A* found goal after visiting %d nodes.' % visited_count)
                return self.reconstruct_path(came_from, current)
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if (0 <= neighbor[0] < self.map.info.width and
                    0 <= neighbor[1] < self.map.info.height and
                    self.get_map_value(neighbor) <= 0):  # Free or unknown
                    tentative_g = g_score[current] + (1.4 if dx and dy else 1.0)  # Diagonal cost
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        self.get_logger().warn('A* exhausted open set after %d nodes. No path.' % visited_count)
        return None

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def get_map_value(self, pos):
        x, y = pos
        idx = y * self.map.info.width + x
        if self.inflated_map_data is None:
            self.get_logger().warn('Inflated map not available. Falling back to original map.')
            if 0 <= idx < len(self.map.data):
                return self.map.data[idx]
            else:
                self.get_logger().warn('Grid position %s out of bounds. Treating as obstacle.' % str(pos))
                return 100
        else:
            if 0 <= idx < len(self.inflated_map_data):
                return self.inflated_map_data[idx]
            else:
                self.get_logger().warn('Grid position %s out of bounds. Treating as obstacle.' % str(pos))
                return 100

    def pose_to_grid(self, pose):
        res = float(self.map.info.resolution)
        ox = float(self.map.info.origin.position.x)
        oy = float(self.map.info.origin.position.y)
        gx = int((pose.position.x - ox) / res)
        gy = int((pose.position.y - oy) / res)
        self.get_logger().debug('Converted pose (%.2f, %.2f) to grid (%d, %d)' % (pose.position.x, pose.position.y, gx, gy))
        return (gx, gy)

    def grid_to_pose(self, grid):
        res = float(self.map.info.resolution)
        ox = float(self.map.info.origin.position.x)
        oy = float(self.map.info.origin.position.y)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(grid[0] * res + ox + res / 2.0)
        pose_stamped.pose.position.y = float(grid[1] * res + oy + res / 2.0)
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        self.get_logger().info('Reconstructed path with %d points.' % len(path))
        return path

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()