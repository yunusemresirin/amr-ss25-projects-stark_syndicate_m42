import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import heapq
import math

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, rclpy.qos.qos_profile_sensor_data)
        self.start_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.start_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.map = None
        self.inflated_map_data = None
        self.start = None
        self.goal = None
        
        # Declare inflation parameter (in meters)
        self.declare_parameter('inflation_radius', 0.6)  # Default 0.2m buffer
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
        
        res = self.map.info.resolution
        inflation_cells = int(self.inflation_radius / res)
        self.get_logger().info('Inflating obstacles by %.2f m (%d cells).' % (self.inflation_radius, inflation_cells))
        
        width = self.map.info.width
        height = self.map.info.height
        data = list(self.map.data)  # Copy original data
        
        # Find all occupied cells
        occupied_indices = [i for i, val in enumerate(self.map.data) if val >= 50]
        self.get_logger().info('Found %d occupied cells to inflate.' % len(occupied_indices))
        
        for idx in occupied_indices:
            x = idx % width
            y = idx // width
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    # Check if within circular radius (optional: use if math.sqrt(dx**2 + dy**2) <= inflation_cells for circle)
                    if abs(dx) + abs(dy) > inflation_cells * 1.5:  # Approximate diamond to reduce computation
                        continue
                    nx = x + dx
                    ny = y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        nidx = ny * width + nx
                        if data[nidx] < 50:  # Only inflate free space, preserve unknown/occupied
                            data[nidx] = 100  # Set as occupied
        
        self.inflated_map_data = data
        self.get_logger().info('Map inflation complete.')

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
        
        if start_value >= 50 or goal_value >= 50:
            self.get_logger().warn('Start or goal is in an obstacle (values >=50). Cannot plan.')
            return
        
        path_grids = self.a_star(start_grid, goal_grid)
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

    def a_star(self, start, goal):
        self.get_logger().debug('A* started from %s to %s' % (str(start), str(goal)))
        open_set = []
        heapq.heappush(open_set, (0, start))  # (priority, position)
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-way movement
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
                    self.get_map_value(neighbor) < 50):  # Free space
                    tentative_g = g_score[current] + 1  # Unit cost
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
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        gx = int((pose.position.x - ox) / res)
        gy = int((pose.position.y - oy) / res)
        self.get_logger().debug('Converted pose (%.2f, %.2f) to grid (%d, %d)' % (pose.position.x, pose.position.y, gx, gy))
        return (gx, gy)

    def grid_to_pose(self, grid):
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
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