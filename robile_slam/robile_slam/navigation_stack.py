import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import heapq

class GlobalPlanner:
    def __init__(self, inflation_radius=0.2):
        self.map = None
        self.inflated_map_data = None
        self.start = None
        self.goal = None
        self.inflation_radius = inflation_radius
        self.logger = None  # Will hold the logger object

    def set_logger(self, logger):
        self.logger = logger  # Store the logger object (not the node)

    def map_callback(self, msg):
        self.logger.info('GlobalPlanner: Received map data. Height: %d, Width: %d, Resolution: %.2f' % (msg.info.height, msg.info.width, msg.info.resolution))
        self.map = msg
        self.inflate_map()
        return self.plan_if_possible()

    def start_callback(self, msg):
        self.logger.info('GlobalPlanner: Received initial pose: Position (x: %.2f, y: %.2f)' % (msg.pose.pose.position.x, msg.pose.pose.position.y))
        self.start = msg.pose.pose
        return self.plan_if_possible()

    def goal_callback(self, msg):
        self.logger.info('GlobalPlanner: Received goal pose: Position (x: %.2f, y: %.2f)' % (msg.pose.position.x, msg.pose.position.y))
        self.goal = msg.pose
        return self.plan_if_possible()

    def inflate_map(self):
        if self.map is None:
            self.logger.warn('GlobalPlanner: Cannot inflate: No map available.')
            return
        res = self.map.info.resolution
        inflation_cells = int(self.inflation_radius / res)
        self.logger.info('GlobalPlanner: Inflating obstacles by %.2f m (%d cells).' % (self.inflation_radius, inflation_cells))
        width = self.map.info.width
        height = self.map.info.height
        data = list(self.map.data)
        occupied_indices = [i for i, val in enumerate(self.map.data) if val >= 50]
        self.logger.info('GlobalPlanner: Found %d occupied cells to inflate.' % len(occupied_indices))
        for idx in occupied_indices:
            x = idx % width
            y = idx // width
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if abs(dx) + abs(dy) > inflation_cells * 1.5:
                        continue
                    nx = x + dx
                    ny = y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        nidx = ny * width + nx
                        if data[nidx] < 50:
                            data[nidx] = 100
        self.inflated_map_data = data
        self.logger.info('GlobalPlanner: Map inflation complete.')

    def plan_if_possible(self):
        if self.map is None or self.inflated_map_data is None:
            self.logger.warn('GlobalPlanner: Cannot plan: Map or inflated map not ready.')
            return None
        if self.start is None:
            self.logger.warn('GlobalPlanner: Cannot plan: Start pose not received yet.')
            return None
        if self.goal is None:
            self.logger.warn('GlobalPlanner: Cannot plan: Goal pose not received yet.')
            return None
        self.logger.info('GlobalPlanner: Starting path planning with A*...')
        start_grid = self.pose_to_grid(self.start)
        goal_grid = self.pose_to_grid(self.goal)
        self.logger.info('GlobalPlanner: Start grid: (%d, %d), Goal grid: (%d, %d)' % (start_grid[0], start_grid[1], goal_grid[0], goal_grid[1]))
        start_value = self.get_map_value(start_grid)
        goal_value = self.get_map_value(goal_grid)
        self.logger.info('GlobalPlanner: Map value at start: %d, at goal: %d' % (start_value, goal_value))
        if start_value >= 50 or goal_value >= 50:
            self.logger.warn('GlobalPlanner: Start or goal is in an obstacle (values >=50). Cannot plan.')
            return None
        path_grids = self.a_star(start_grid, goal_grid)
        if path_grids:
            self.logger.info('GlobalPlanner: Path found with %d grid points.' % len(path_grids))
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.logger.get_clock().now().to_msg()
            path_msg.poses = [self.grid_to_pose(grid) for grid in path_grids]
            return path_msg
        self.logger.warn('GlobalPlanner: No path found after A* search.')
        return None

    def a_star(self, start, goal):
        self.logger.debug('GlobalPlanner: A* started from %s to %s' % (str(start), str(goal)))
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        visited_count = 0
        while open_set:
            visited_count += 1
            if visited_count % 1000 == 0:
                self.logger.debug('GlobalPlanner: A* processing: Visited %d nodes, open set size: %d' % (visited_count, len(open_set)))
            _, current = heapq.heappop(open_set)
            if current == goal:
                self.logger.info('GlobalPlanner: A* found goal after visiting %d nodes.' % visited_count)
                return self.reconstruct_path(came_from, current)
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if (0 <= neighbor[0] < self.map.info.width and
                    0 <= neighbor[1] < self.map.info.height and
                    self.get_map_value(neighbor) < 50):
                    tentative_g = g_score[current] + 1
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        self.logger.warn('GlobalPlanner: A* exhausted open set after %d nodes. No path.')
        return None

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def get_map_value(self, pos):
        x, y = pos
        idx = y * self.map.info.width + x
        if self.inflated_map_data is None:
            self.logger.warn('GlobalPlanner: Inflated map not available. Falling back to original map.')
            if 0 <= idx < len(self.map.data):
                return self.map.data[idx]
            return 100
        if 0 <= idx < len(self.inflated_map_data):
            return self.inflated_map_data[idx]
        self.logger.warn('GlobalPlanner: Grid position %s out of bounds. Treating as obstacle.' % str(pos))
        return 100

    def pose_to_grid(self, pose):
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        gx = int((pose.position.x - ox) / res)
        gy = int((pose.position.y - oy) / res)
        self.logger.debug('GlobalPlanner: Converted pose (%.2f, %.2f) to grid (%d, %d)' % (pose.position.x, pose.position.y, gx, gy))
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
        self.logger.info('GlobalPlanner: Reconstructed path with %d points.' % len(path))
        return path

class WaypointFollower:
    def __init__(self, waypoint_spacing=1.0, waypoint_distance_threshold=0.2):
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.current_pose = None
        self.waypoint_spacing = waypoint_spacing
        self.waypoint_distance_threshold = waypoint_distance_threshold
        self.logger = None

    def set_logger(self, logger):
        self.logger = logger

    def path_callback(self, msg):
        self.logger.info('WaypointFollower: Received new global path with %d poses.' % len(msg.poses))
        self.waypoints = self.downsample_path(msg.poses)
        self.current_waypoint_idx = 0
        self.logger.info('WaypointFollower: Downsampled to %d waypoints.' % len(self.waypoints))
        return self.get_current_waypoint(), self.get_waypoints_marker()

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.logger.debug('WaypointFollower: Updated current pose: (x: %.2f, y: %.2f)' % (self.current_pose.position.x, self.current_pose.position.y))
        return self.check_waypoint_reached()

    def downsample_path(self, poses):
        if not poses:
            self.logger.warn('WaypointFollower: Empty path received. No waypoints generated.')
            return []
        waypoints = [poses[0]]
        last_wp_pos = poses[0].pose.position
        self.logger.debug('WaypointFollower: Added first waypoint: (x: %.2f, y: %.2f)' % (last_wp_pos.x, last_wp_pos.y))
        for pose in poses[1:]:
            pos = pose.pose.position
            dist = math.sqrt((pos.x - last_wp_pos.x)**2 + (pos.y - last_wp_pos.y)**2)
            self.logger.debug('WaypointFollower: Checking distance to next pose: %.2f' % dist)
            if dist >= self.waypoint_spacing:
                waypoints.append(pose)
                last_wp_pos = pos
                self.logger.debug('WaypointFollower: Added waypoint: (x: %.2f, y: %.2f)' % (last_wp_pos.x, last_wp_pos.y))
        last_pos = poses[-1].pose.position
        dist_to_last = math.sqrt((last_pos.x - last_wp_pos.x)**2 + (last_pos.y - last_wp_pos.y)**2)
        if dist_to_last > 0:
            waypoints.append(poses[-1])
            self.logger.debug('WaypointFollower: Added final waypoint: (x: %.2f, y: %.2f)' % (last_pos.x, last_pos.y))
        return waypoints

    def check_waypoint_reached(self):
        if not self.waypoints or self.current_pose is None:
            self.logger.debug('WaypointFollower: Waiting for waypoints (%d) or current pose.' % len(self.waypoints))
            return None, None
        if self.current_waypoint_idx >= len(self.waypoints):
            self.logger.info('WaypointFollower: All waypoints reached. No more local goals.')
            return None, None
        current_wp = self.waypoints[self.current_waypoint_idx]
        dist_to_wp = math.sqrt(
            (current_wp.pose.position.x - self.current_pose.position.x)**2 +
            (current_wp.pose.position.y - self.current_pose.position.y)**2
        )
        self.logger.debug('WaypointFollower: Distance to current waypoint %d: %.2f' % (self.current_waypoint_idx, dist_to_wp))
        if dist_to_wp < self.waypoint_distance_threshold:
            self.current_waypoint_idx += 1
            self.logger.info('WaypointFollower: Reached waypoint %d (dist: %.2f < %.2f). Advancing to next.' % (self.current_waypoint_idx - 1, dist_to_wp, self.waypoint_distance_threshold))
            return self.get_current_waypoint(), self.get_waypoints_marker()
        return None, None

    def get_current_waypoint(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.logger.warn('WaypointFollower: No more waypoints to publish.')
            return None
        local_goal = self.waypoints[self.current_waypoint_idx]
        local_goal.header.stamp = self.logger.get_clock().now().to_msg()
        pos = local_goal.pose.position
        self.logger.info('WaypointFollower: Prepared local goal %d: (x: %.2f, y: %.2f)' % (self.current_waypoint_idx, pos.x, pos.y))
        return local_goal

    def get_waypoints_marker(self):
        if not self.waypoints:
            self.logger.warn('WaypointFollower: No waypoints to visualize.')
            return None
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.logger.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [wp.pose.position for wp in self.waypoints]
        self.logger.info('WaypointFollower: Prepared %d waypoint points for visualization.' % len(self.waypoints))
        return marker

class PotentialFieldPlanner:
    def __init__(self, attractive_gain=1.5, repulsive_gain=2.0, repulsive_distance=0.5,
                 goal_distance_threshold=0.2, max_linear_speed=0.5, max_angular_speed=1.0,
                 stuck_velocity_threshold=0.05, stuck_time=5.0, deviation_threshold=0.5):
        self.current_pose = None
        self.local_goal = None
        self.scan_ranges = None
        self.stuck_timer = 0.0
        self.attractive_gain = attractive_gain
        self.repulsive_gain = repulsive_gain
        self.repulsive_distance = repulsive_distance
        self.goal_distance_threshold = goal_distance_threshold
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.stuck_velocity_threshold = stuck_velocity_threshold
        self.stuck_time = stuck_time
        self.deviation_threshold = deviation_threshold
        self.logger = None

    def set_logger(self, logger):
        self.logger = logger

    def local_goal_callback(self, msg):
        self.local_goal = msg.pose
        self.logger.info('PotentialFieldPlanner: Received local goal: (x: %.2f, y: %.2f)' % (msg.pose.position.x, msg.pose.position.y))

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.logger.debug('PotentialFieldPlanner: Updated current pose: (x: %.2f, y: %.2f)' % (self.current_pose.position.x, self.current_pose.position.y))

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.logger.debug('PotentialFieldPlanner: Received scan with %d ranges, min: %.2f, max: %.2f' % (
            len(msg.ranges), min([r for r in msg.ranges if r > 0]) if any(r > 0 for r in msg.ranges) else 0, max(msg.ranges)))

    def control_loop(self):
        if self.current_pose is None or self.local_goal is None or self.scan_ranges is None:
            self.logger.warn('PotentialFieldPlanner: Missing data: pose=%s, goal=%s, scan=%s' % (
                self.current_pose is not None, self.local_goal is not None, self.scan_ranges is not None))
            return None, None
        dist_to_goal = math.sqrt(
            (self.local_goal.position.x - self.current_pose.position.x)**2 +
            (self.local_goal.position.y - self.current_pose.position.y)**2
        )
        self.logger.debug('PotentialFieldPlanner: Distance to local goal: %.2f' % dist_to_goal)
        if dist_to_goal < self.goal_distance_threshold:
            self.logger.info('PotentialFieldPlanner: Local goal reached (dist: %.2f < %.2f). Stopping.' % (dist_to_goal, self.goal_distance_threshold))
            return self.stop_robot(), None
        attr_x, attr_y = self.attractive_potential()
        self.logger.debug('PotentialFieldPlanner: Attractive force: (x: %.2f, y: %.2f)' % (attr_x, attr_y))
        rep_x, rep_y = self.repulsive_potential()
        self.logger.debug('PotentialFieldPlanner: Repulsive force: (x: %.2f, y: %.2f)' % (rep_x, rep_y))
        total_force_x = attr_x + rep_x
        total_force_y = attr_y + rep_y
        self.logger.debug('PotentialFieldPlanner: Total force: (x: %.2f, y: %.2f)' % (total_force_x, total_force_y))
        desired_heading = math.atan2(total_force_y, total_force_x)
        current_heading = self.get_yaw(self.current_pose.orientation)
        angular_vel = self.angular_control(desired_heading - current_heading)
        linear_vel = self.max_linear_speed * (1 - abs(angular_vel) / self.max_angular_speed)
        self.logger.debug('PotentialFieldPlanner: Computed velocities: linear=%.2f, angular=%.2f' % (linear_vel, angular_vel))
        twist = Twist()
        twist.linear.x = min(linear_vel, self.max_linear_speed)
        twist.angular.z = angular_vel
        current_vel = math.sqrt(twist.linear.x**2 + twist.angular.z**2)
        if current_vel < self.stuck_velocity_threshold:
            self.stuck_timer += 0.1
            self.logger.debug('PotentialFieldPlanner: Low velocity (%.2f < %.2f), stuck timer: %.2f' % (
                current_vel, self.stuck_velocity_threshold, self.stuck_timer))
            if self.stuck_timer > self.stuck_time:
                self.logger.warn('PotentialFieldPlanner: Robot stuck for %.2f s. Triggering replan.' % self.stuck_timer)
                self.stuck_timer = 0.0
                return twist, self.trigger_replan()
        else:
            self.stuck_timer = 0.0
            self.logger.debug('PotentialFieldPlanner: Robot moving, resetting stuck timer.')
        if dist_to_goal > self.deviation_threshold:
            self.logger.warn('PotentialFieldPlanner: Deviation from goal too high (%.2f > %.2f). Triggering replan.' % (dist_to_goal, self.deviation_threshold))
            return twist, self.trigger_replan()
        self.logger.info('PotentialFieldPlanner: Computed cmd_vel: linear.x=%.2f, angular.z=%.2f' % (twist.linear.x, twist.angular.z))
        return twist, None

    def attractive_potential(self):
        dx = self.local_goal.position.x - self.current_pose.position.x
        dy = self.local_goal.position.y - self.current_pose.position.y
        dist = max(math.sqrt(dx**2 + dy**2), 0.001)
        force_x = self.attractive_gain * (dx / dist)
        force_y = self.attractive_gain * (dy / dist)
        return (force_x, force_y)

    def repulsive_potential(self):
        if self.scan_ranges is None:
            return (0.0, 0.0)
        force_x, force_y = 0.0, 0.0
        angle_min = -math.pi
        angle_increment = 2 * math.pi / len(self.scan_ranges)
        for i, r in enumerate(self.scan_ranges):
            if 0 < r < self.repulsive_distance:
                angle = angle_min + i * angle_increment
                magnitude = self.repulsive_gain * (1/r - 1/self.repulsive_distance) / (r**2)
                force_x -= magnitude * math.cos(angle)
                force_y -= magnitude * math.sin(angle)
        return (force_x, force_y)

    def angular_control(self, angle_error):
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
        return max(min(angle_error * 2.0, self.max_angular_speed), -self.max_angular_speed)

    def get_yaw(self, quat):
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y**2 + quat.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def trigger_replan(self):
        if self.current_pose is None:
            self.logger.warn('PotentialFieldPlanner: Cannot trigger replan: No current pose.')
            return None
        replan_msg = PoseWithCovarianceStamped()
        replan_msg.header.frame_id = 'map'
        replan_msg.header.stamp = self.logger.get_clock().now().to_msg()
        replan_msg.pose.pose = self.current_pose
        replan_msg.pose.covariance = [0.1] * 36
        self.logger.info('PotentialFieldPlanner: Prepared current pose for /initialpose replan.')
        return replan_msg

    def stop_robot(self):
        twist = Twist()
        self.logger.info('PotentialFieldPlanner: Prepared zero velocity.')
        return twist

class MasterNavigationNode(Node):
    def __init__(self):
        super().__init__('master_navigation')
        # Initialize classes
        self.global_planner = GlobalPlanner(inflation_radius=0.2)
        self.waypoint_follower = WaypointFollower(waypoint_spacing=1.0, waypoint_distance_threshold=0.2)
        self.potential_field_planner = PotentialFieldPlanner(
            attractive_gain=1.5, repulsive_gain=2.0, repulsive_distance=0.5,
            goal_distance_threshold=0.2, max_linear_speed=0.5, max_angular_speed=1.0,
            stuck_velocity_threshold=0.05, stuck_time=5.0, deviation_threshold=0.5
        )
        # Set loggers (pass self.get_logger() instead of self)
        self.global_planner.set_logger(self.get_logger())
        self.waypoint_follower.set_logger(self.get_logger())
        self.potential_field_planner.set_logger(self.get_logger())
        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Publishers
        self.global_path_pub = self.create_publisher(Path, '/global_path', 10)
        self.local_goal_pub = self.create_publisher(PoseStamped, '/local_goal', 10)
        self.waypoints_marker_pub = self.create_publisher(Marker, '/waypoints_marker', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.replan_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Master Navigation node started.')

    def map_callback(self, msg):
        path_msg = self.global_planner.map_callback(msg)
        if path_msg:
            self.global_path_pub.publish(path_msg)
            self.get_logger().info('MasterNavigation: Published global path.')
            local_goal, waypoints_marker = self.waypoint_follower.path_callback(path_msg)
            if local_goal:
                self.local_goal_pub.publish(local_goal)
                self.get_logger().info('MasterNavigation: Published initial local goal.')
            if waypoints_marker:
                self.waypoints_marker_pub.publish(waypoints_marker)
                self.get_logger().info('MasterNavigation: Published waypoints marker.')

    def initial_pose_callback(self, msg):
        if self.global_planner.map is None:
            self.get_logger().warn('MasterNavigation: Ignoring initial pose: Map not received yet.')
            return
        path_msg = self.global_planner.start_callback(msg)
        if path_msg:
            self.global_path_pub.publish(path_msg)
            self.get_logger().info('MasterNavigation: Published global path after initial pose.')
            local_goal, waypoints_marker = self.waypoint_follower.path_callback(path_msg)
            if local_goal:
                self.local_goal_pub.publish(local_goal)
                self.get_logger().info('MasterNavigation: Published initial local goal.')
            if waypoints_marker:
                self.waypoints_marker_pub.publish(waypoints_marker)
                self.get_logger().info('MasterNavigation: Published waypoints marker.')

    def goal_pose_callback(self, msg):
        if self.global_planner.map is None:
            self.get_logger().warn('MasterNavigation: Ignoring goal pose: Map not received yet.')
            return
        path_msg = self.global_planner.goal_callback(msg)
        if path_msg:
            self.global_path_pub.publish(path_msg)
            self.get_logger().info('MasterNavigation: Published global path after goal pose.')
            local_goal, waypoints_marker = self.waypoint_follower.path_callback(path_msg)
            if local_goal:
                self.local_goal_pub.publish(local_goal)
                self.get_logger().info('MasterNavigation: Published initial local goal.')
            if waypoints_marker:
                self.waypoints_marker_pub.publish(waypoints_marker)
                self.get_logger().info('MasterNavigation: Published waypoints marker.')

    def amcl_pose_callback(self, msg):
        local_goal, waypoints_marker = self.waypoint_follower.pose_callback(msg)
        if local_goal:
            self.local_goal_pub.publish(local_goal)
            self.get_logger().info('MasterNavigation: Published new local goal after waypoint reached.')
        if waypoints_marker:
            self.waypoints_marker_pub.publish(waypoints_marker)
            self.get_logger().info('MasterNavigation: Published updated waypoints marker.')
        self.potential_field_planner.pose_callback(msg)

    def scan_callback(self, msg):
        self.potential_field_planner.scan_callback(msg)

    def control_loop(self):
        twist, replan_msg = self.potential_field_planner.control_loop()
        if twist:
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('MasterNavigation: Published cmd_vel.')
        if replan_msg:
            self.replan_pub.publish(replan_msg)
            self.get_logger().info('MasterNavigation: Published replan request to /initialpose.')

def main(args=None):
    rclpy.init(args=args)
    node = MasterNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()