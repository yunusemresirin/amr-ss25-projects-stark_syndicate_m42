#!/usr/bin/env python3

import rclpy
import numpy as np
import math
import random
from collections import deque
import heapq
from scipy.ndimage import binary_dilation

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist, Pose, Point
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from tf_transformations import euler_from_quaternion
import tf2_ros
from rclpy.duration import Duration

class CombinedExplorationNode(Node):
    """
    A ROS2 node combining frontier-based exploration and A* path planning.
    Selects farthest frontier goal, plans A* path, and handles invalid goals.
    """
    def __init__(self):
        super().__init__('combined_exploration_node')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=5
        )

        # Subscriptions
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF Listener for pose from SLAM
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State variables
        self.map = None
        self.map_info = None
        self.occupancy_grid = None
        self.inflated_map_data = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.current_goal = None
        self.current_path = None
        self.goal_dist_thresh = 0.5
        self.initial_pose_published = False
        self.min_free_cells = 100  # Minimum free cells to start exploration
        self.candidate_goals = []  # List of candidate frontier goals, sorted farthest to closest
        self.fallback_movement = False
        self.fallback_start_time = None
        self.fallback_duration = 5.0  # 5 seconds of forward movement

        # Parameters
        self.min_frontier_size = 1
        self.exploration_rate = 1.0
        self.inflation_radius = 0.2  # Inflation radius in meters
        self.create_timer(self.exploration_rate, self._explore_cb)
        self.create_timer(0.1, self._update_pose_from_tf)

        self.get_logger().info('Combined exploration node initialized.')

    def _map_cb(self, msg: OccupancyGrid):
        self.map = msg
        self.map_info = msg.info
        self.occupancy_grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self._inflate_map()
        
        free_cells = np.sum(self.occupancy_grid == 0)
        unknown_cells = np.sum(self.occupancy_grid == -1)
        occupied_cells = np.sum(self.occupancy_grid >= 50)
        self.get_logger().info(
            f'Map received: {free_cells} free cells, {unknown_cells} unknown cells, {occupied_cells} occupied cells'
        )

        if not self.initial_pose_published and self.x is not None:
            self._publish_initial_pose()
            self.initial_pose_published = True

    def _odom_cb(self, msg: Odometry):
        # Optional fallback if TF is not available
        pass

    def _update_pose_from_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=1.0))
            self.x = float(trans.transform.translation.x)
            self.y = float(trans.transform.translation.y)
            quat = trans.transform.rotation
            _, _, self.yaw = euler_from_quaternion([float(quat.x), float(quat.y), float(quat.z), float(quat.w)])
            self.get_logger().debug(f'Updated pose from TF: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {str(e)}')

    def _publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = self.x
        initial_pose.pose.pose.position.y = self.y
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = self._euler_to_quat(self.yaw, 0.0, 0.0)
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]
        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info(f'Published initial pose: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}')

    def _explore_cb(self):
        if self.map_info is None or self.occupancy_grid is None or self.inflated_map_data is None:
            self.get_logger().warn('No map available for exploration.')
            return

        # Check if map has enough free cells
        free_cells = np.sum(self.occupancy_grid == 0)
        if free_cells < self.min_free_cells:
            self.get_logger().warn(f'Insufficient free cells ({free_cells} < {self.min_free_cells}). Initiating fallback movement.')
            self._fallback_movement()
            return

        if self.fallback_movement:
            elapsed = self.get_clock().now().seconds_nanoseconds()[0] - self.fallback_start_time
            if elapsed < self.fallback_duration:
                cmd = Twist()
                cmd.linear.x = 0.2  # Move forward at 0.2 m/s
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().debug('Fallback movement: Moving forward')
                return
            else:
                self.cmd_vel_pub.publish(Twist())  # Stop movement
                self.fallback_movement = False
                self.get_logger().info('Fallback movement complete.')

        if self.current_goal is not None:
            dist_to_goal = math.dist((self.x, self.y), (self.current_goal.pose.position.x, self.current_goal.pose.position.y))
            if dist_to_goal > self.goal_dist_thresh:
                return

        # Generate candidates if none or all tried
        if not self.candidate_goals:
            self.candidate_goals = self._generate_candidate_goals()
            if not self.candidate_goals:
                self.get_logger().warn('No candidate goals generated. Initiating fallback movement.')
                self._fallback_movement()
                return

        # Try candidates until a valid path is found
        while self.candidate_goals:
            goal = self.candidate_goals.pop(0)  # Pop the farthest
            goal_grid = self._pose_to_grid(goal)
            if self._get_map_value(goal_grid, inflated=True) < 50:  # Check inflated map
                start_grid = self._pose_to_grid(PoseStamped(pose=Pose(position=Point(x=self.x, y=self.y))))
                start_value = self._get_map_value(start_grid, inflated=True)
                adjusted_start = start_grid
                if start_value >= 50:
                    adjusted_start = self._find_nearest_free(start_grid)
                    if adjusted_start is None:
                        self.get_logger().warn('No free or unknown cell near start. Trying next goal.')
                        continue
                    self.get_logger().info('Adjusted start to: (%d, %d)' % adjusted_start)
                
                path_grids = self._a_star(adjusted_start, goal_grid)
                if path_grids:
                    self.current_goal = goal
                    self.current_path = path_grids
                    path_msg = Path()
                    path_msg.header.frame_id = 'map'
                    path_msg.header.stamp = self.get_clock().now().to_msg()
                    path_msg.poses = [self._grid_to_pose(grid) for grid in path_grids]
                    self.path_pub.publish(path_msg)
                    self.get_logger().info(f'Published path to goal at ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) with {len(path_grids)} points.')
                    return
                else:
                    self.get_logger().warn(f'No path to goal at ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}). Trying next candidate.')
            else:
                self.get_logger().warn(f'Goal at ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) is in obstacle. Trying next candidate.')
        
        self.get_logger().warn('No valid goals with paths available. Initiating fallback movement.')
        self._fallback_movement()

    def _fallback_movement(self):
        if not self.fallback_movement:
            self.fallback_movement = True
            self.fallback_start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.get_logger().info('Starting fallback movement to build map.')

    def _generate_candidate_goals(self):
        """Generate sorted list of frontier goals, farthest first, excluding obstacle cells."""
        if self.occupancy_grid is None:
            self.get_logger().warn('Cannot generate goals: No map available.')
            return []

        grid = self.occupancy_grid
        height, width = grid.shape
        free = (grid == 0)
        unknown = (grid == -1)

        # Find frontier cells
        frontier_cells = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for y in range(height):
            for x in range(width):
                if free[y, x]:
                    for dy, dx in directions:
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < height and 0 <= nx < width and unknown[ny, nx]:
                            frontier_cells.append((x, y))
                            break

        self.get_logger().info(f'Found {len(frontier_cells)} frontier cells.')

        if not frontier_cells:
            return self._generate_random_candidates()

        # Cluster frontiers using flood-fill
        def flood_fill(start_x, start_y, cluster):
            stack = [(start_x, start_y)]
            visited = set()
            while stack:
                x, y = stack.pop()
                if (x, y) in visited or not (0 <= x < width and 0 <= y < height) or (x, y) not in frontier_set:
                    continue
                visited.add((x, y))
                cluster.append((x, y))
                for dy, dx in directions:
                    stack.append((x + dx, y + dy))
            return cluster

        frontier_set = set(frontier_cells)
        clusters = []
        while frontier_set:
            x, y = frontier_set.pop()
            cluster = flood_fill(x, y, [])
            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)
            for pt in cluster:
                frontier_set.discard(pt)

        self.get_logger().info(f'Found {len(clusters)} frontier clusters with min size {self.min_frontier_size}.')

        if not clusters:
            # Fallback to single frontier cells
            frontier_candidates = []
            for x, y in frontier_cells:
                if self._get_map_value((x, y), inflated=False) < 50:  # Check raw map
                    wx, wy = self._grid_to_world(float(x), float(y))
                    yaw = random.uniform(-math.pi, math.pi)
                    quat = self._euler_to_quat(yaw, 0.0, 0.0)
                    goal = PoseStamped()
                    goal.header.frame_id = 'map'
                    goal.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.position.x = float(wx)
                    goal.pose.position.y = float(wy)
                    goal.pose.position.z = 0.0
                    goal.pose.orientation = quat
                    dist = math.dist((self.x, self.y), (wx, wy))
                    frontier_candidates.append((dist, goal))
            # Sort farthest first
            frontier_candidates.sort(reverse=True, key=lambda x: x[0])
            return [g for d, g in frontier_candidates]

        # Compute centroids and sort farthest first
        candidates = []
        for cluster in clusters:
            gx = sum(float(x) for x, _ in cluster) / len(cluster)
            gy = sum(float(y) for _, y in cluster) / len(cluster)
            wx, wy = self._grid_to_world(gx, gy)
            grid_pos = self._pose_to_grid(PoseStamped(pose=Pose(position=Point(x=float(wx), y=float(wy)))))
            if self._get_map_value(grid_pos, inflated=False) < 50:  # Check raw map
                # Orient toward unknown region
                unknown_neighbors = []
                for x, y in cluster:
                    for dy, dx in directions:
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < height and 0 <= nx < width and unknown[ny, nx]:
                            unknown_neighbors.append((nx, ny))
                if unknown_neighbors:
                    ux, uy = np.mean(unknown_neighbors, axis=0)
                    yaw = math.atan2(float(uy - gy), float(ux - gx))
                else:
                    yaw = random.uniform(-math.pi, math.pi)
                quat = self._euler_to_quat(yaw, 0.0, 0.0)
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = float(wx)
                goal.pose.position.y = float(wy)
                goal.pose.position.z = 0.0
                goal.pose.orientation = quat
                dist = math.dist((self.x, self.y), (wx, wy))
                candidates.append((dist, goal))
        # Sort farthest first
        candidates.sort(reverse=True, key=lambda x: x[0])
        return [g for d, g in candidates]

    def _generate_random_candidates(self):
        """Generate random free-space candidates, sorted farthest first, excluding obstacle cells."""
        if self.occupancy_grid is None:
            self.get_logger().warn('Cannot generate random candidates: No map available.')
            return []

        grid = self.occupancy_grid
        height, width = grid.shape
        free_cells = [(x, y) for y in range(height) for x in range(width) if grid[y, x] <= 0]  # Include unknown cells
        
        if not free_cells:
            self.get_logger().warn('No free or unknown cells available for random candidates.')
            return []

        candidates = []
        for x, y in free_cells:
            if grid[y, x] < 50:  # Exclude obstacle cells
                wx, wy = self._grid_to_world(float(x), float(y))
                yaw = random.uniform(-math.pi, math.pi)
                quat = self._euler_to_quat(yaw, 0.0, 0.0)
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = float(wx)
                goal.pose.position.y = float(wy)
                goal.pose.position.z = 0.0
                goal.pose.orientation = quat
                dist = math.dist((self.x, self.y), (wx, wy))
                candidates.append((dist, goal))
        # Sort farthest first
        candidates.sort(reverse=True, key=lambda x: x[0])
        return [g for d, g in candidates]

    def _inflate_map(self):
        if self.map is None or self.occupancy_grid is None:
            self.get_logger().warn('Cannot inflate: No map available.')
            return
        
        res = float(self.map_info.resolution)
        inflation_cells = int(self.inflation_radius / res)
        self.get_logger().info('Inflating obstacles by %.2f m (%d cells).' % (self.inflation_radius, inflation_cells))
        
        width = self.map_info.width
        height = self.map_info.height
        data = self.occupancy_grid
        
        # Inflate occupied cells (>=50)
        occupied = (data >= 50)
        inflated = binary_dilation(occupied, structure=np.ones((2 * inflation_cells + 1, 2 * inflation_cells + 1)))
        self.inflated_map_data = np.where(inflated, 100, np.where(data == -1, -1, 0)).flatten()
        self.get_logger().info('Map inflation complete. Occupied cells after inflation: %d' % np.sum(self.inflated_map_data >= 50))

    def _find_nearest_free(self, start):
        """Find the nearest free or unknown cell using BFS with a larger search radius."""
        width = self.map_info.width
        height = self.map_info.height
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

    def _a_star(self, start, goal):
        self.get_logger().debug('A* started from %s to %s' % (str(start), str(goal)))
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 8-way movement
        visited_count = 0
        while open_set:
            visited_count += 1
            if visited_count % 1000 == 0:
                self.get_logger().debug('A* processing: Visited %d nodes, open set size: %d' % (visited_count, len(open_set)))
            _, current = heapq.heappop(open_set)
            if current == goal:
                self.get_logger().info('A* found goal after visiting %d nodes.' % visited_count)
                return self._reconstruct_path(came_from, current)
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if (0 <= neighbor[0] < self.map_info.width and
                    0 <= neighbor[1] < self.map_info.height and
                    self._get_map_value(neighbor, inflated=True) <= 0):  # Free or unknown
                    tentative_g = g_score[current] + (1.4 if dx and dy else 1.0)  # Diagonal cost
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        self.get_logger().warn('A* exhausted open set after %d nodes. No path.' % visited_count)
        return None

    def _heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        self.get_logger().info('Reconstructed path with %d points.' % len(path))
        return path

    def _get_map_value(self, pos, inflated=True):
        x, y = pos
        idx = y * self.map_info.width + x
        map_data = self.inflated_map_data if inflated else self.occupancy_grid.flatten()
        if 0 <= idx < len(map_data):
            return map_data[idx]
        self.get_logger().warn('Grid position %s out of bounds. Treating as obstacle.' % str(pos))
        return 100

    def _pose_to_grid(self, pose):
        if self.map_info is None:
            self.get_logger().warn('Cannot convert pose to grid: No map info available.')
            return (0, 0)
        res = float(self.map_info.resolution)
        ox = float(self.map_info.origin.position.x)
        oy = float(self.map_info.origin.position.y)
        gx = int((pose.pose.position.x - ox) / res)
        gy = int((pose.pose.position.y - oy) / res)
        self.get_logger().debug('Converted pose (%.2f, %.2f) to grid (%d, %d)' % (pose.pose.position.x, pose.pose.position.y, gx, gy))
        return (gx, gy)

    def _grid_to_pose(self, grid):
        res = float(self.map_info.resolution)
        ox = float(self.map_info.origin.position.x)
        oy = float(self.map_info.origin.position.y)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(grid[0] * res + ox + res / 2.0)
        pose_stamped.pose.position.y = float(grid[1] * res + oy + res / 2.0)
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def _grid_to_world(self, gx, gy):
        res = float(self.map_info.resolution)
        ox = float(self.map_info.origin.position.x)
        oy = float(self.map_info.origin.position.y)
        return (ox + gx * res, oy + gy * res)

    def _euler_to_quat(self, yaw, pitch, roll):
        yaw, pitch, roll = float(yaw), float(pitch), float(roll)
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))

def main(args=None):
    rclpy.init(args=args)
    node = CombinedExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()