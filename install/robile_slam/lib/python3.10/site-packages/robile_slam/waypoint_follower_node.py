import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        # Subscriptions
        self.path_sub = self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # Use /odom for now; later /amcl_pose
        
        # Publishers
        self.local_goal_pub = self.create_publisher(PoseStamped, '/local_goal', 10)
        self.waypoints_pub = self.create_publisher(Path, '/waypoints', 10)  # New publisher for waypoints
        
        # Parameters
        self.declare_parameter('waypoint_spacing', 1.0)  # Meters between waypoints
        self.declare_parameter('waypoint_distance_threshold', 0.2)  # Distance to consider waypoint reached
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.waypoint_distance_threshold = self.get_parameter('waypoint_distance_threshold').value
        
        # Internal state
        self.waypoints = []  # List of PoseStamped
        self.current_waypoint_idx = 0
        self.current_pose = None
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Waypoint Follower node started. Waiting for global path and odom.')

    def path_callback(self, msg):
        self.get_logger().info('Received new global path with %d poses.' % len(msg.poses))
        self.waypoints = self.downsample_path(msg.poses)
        self.current_waypoint_idx = 0
        self.get_logger().info('Downsampled to %d waypoints.' % len(self.waypoints))
        self.publish_waypoints()
        self.publish_current_waypoint()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.get_logger().debug('Updated current pose: (x: %.2f, y: %.2f)' % (self.current_pose.position.x, self.current_pose.position.y))

    def downsample_path(self, poses):
        if not poses:
            self.get_logger().warn('Empty path received. No waypoints generated.')
            return []
        waypoints = [poses[0]]  # Start with first pose
        last_wp_pos = poses[0].pose.position
        self.get_logger().debug('Added first waypoint: (x: %.2f, y: %.2f)' % (last_wp_pos.x, last_wp_pos.y))
        for pose in poses[1:]:
            pos = pose.pose.position
            dist = math.sqrt((pos.x - last_wp_pos.x)**2 + (pos.y - last_wp_pos.y)**2)
            self.get_logger().debug('Checking distance to next pose: %.2f' % dist)
            if dist >= self.waypoint_spacing:
                waypoints.append(pose)
                last_wp_pos = pos
                self.get_logger().debug('Added waypoint: (x: %.2f, y: %.2f)' % (last_wp_pos.x, last_wp_pos.y))
        # Always add the last pose if not already added
        last_pos = poses[-1].pose.position
        dist_to_last = math.sqrt((last_pos.x - last_wp_pos.x)**2 + (last_pos.y - last_wp_pos.y)**2)
        if dist_to_last > 0:
            waypoints.append(poses[-1])
            self.get_logger().debug('Added final waypoint: (x: %.2f, y: %.2f)' % (last_pos.x, last_pos.y))
        return waypoints

    def control_loop(self):
        if not self.waypoints or self.current_pose is None:
            self.get_logger().debug('Control loop: Waiting for waypoints (%d) or current pose.' % len(self.waypoints))
            return
        
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached. No more local goals to publish.')
            return
        
        current_wp = self.waypoints[self.current_waypoint_idx]
        dist_to_wp = math.sqrt(
            (current_wp.pose.position.x - self.current_pose.position.x)**2 +
            (current_wp.pose.position.y - self.current_pose.position.y)**2
        )
        self.get_logger().debug('Distance to current waypoint %d: %.2f' % (self.current_waypoint_idx, dist_to_wp))
        
        if dist_to_wp < self.waypoint_distance_threshold:
            self.current_waypoint_idx += 1
            self.get_logger().info('Reached waypoint %d (dist: %.2f < %.2f). Advancing to next.' % (self.current_waypoint_idx - 1, dist_to_wp, self.waypoint_distance_threshold))
            self.publish_current_waypoint()
            self.publish_waypoints()  # Update waypoint visualization

    def publish_current_waypoint(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().warn('No more waypoints to publish.')
            return
        
        local_goal = self.waypoints[self.current_waypoint_idx]
        local_goal.header.stamp = self.get_clock().now().to_msg()
        self.local_goal_pub.publish(local_goal)
        pos = local_goal.pose.position
        self.get_logger().info('Published local goal %d: (x: %.2f, y: %.2f)' % (self.current_waypoint_idx, pos.x, pos.y))

    def publish_waypoints(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints to publish for visualization.')
            return
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = self.waypoints
        self.waypoints_pub.publish(path_msg)
        self.get_logger().info('Published %d waypoints to /waypoints for visualization.' % len(self.waypoints))

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()