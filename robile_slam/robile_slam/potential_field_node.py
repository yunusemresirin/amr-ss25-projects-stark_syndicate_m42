import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')
        # Subscriptions
        self.local_goal_sub = self.create_subscription(PoseStamped, '/local_goal', self.local_goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # Replace with /amcl_pose later
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.replan_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)  # To trigger global replan
        
        # Parameters
        self.declare_parameter('attractive_gain', 1.5)  # Gain for attractive potential
        self.declare_parameter('repulsive_gain', 2.5)  # Gain for repulsive potential
        self.declare_parameter('repulsive_distance', 0.5)  # Distance threshold for repulsion
        self.declare_parameter('goal_distance_threshold', 0.2)  # Distance to consider goal reached
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('stuck_velocity_threshold', 0.05)  # m/s
        self.declare_parameter('stuck_time', 5.0)  # seconds
        self.declare_parameter('deviation_threshold', 0.5)  # Max deviation from goal before replan
        
        self.attractive_gain = self.get_parameter('attractive_gain').value
        self.repulsive_gain = self.get_parameter('repulsive_gain').value
        self.repulsive_distance = self.get_parameter('repulsive_distance').value
        self.goal_distance_threshold = self.get_parameter('goal_distance_threshold').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.stuck_velocity_threshold = self.get_parameter('stuck_velocity_threshold').value
        self.stuck_time = self.get_parameter('stuck_time').value
        self.deviation_threshold = self.get_parameter('deviation_threshold').value
        
        # Internal state
        self.current_pose = None
        self.local_goal = None
        self.scan_ranges = None
        self.stuck_timer = 0.0
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Potential Field Planner node started. Waiting for local goal, odom, and scan.')

    def local_goal_callback(self, msg):
        self.local_goal = msg.pose
        self.get_logger().info('Received local goal: (x: %.2f anciennes y: %.2f)' % (msg.pose.position.x, msg.pose.position.y))

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.get_logger().debug('Updated current pose: (x: %.2f, y: %.2f)' % (self.current_pose.position.x, self.current_pose.position.y))

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.get_logger().debug('Received scan with %d ranges, min: %.2f, max: %.2f' % (len(msg.ranges), min([r for r in msg.ranges if r > 0]) if any(r > 0 for r in msg.ranges) else 0, max(msg.ranges)))

    def control_loop(self):
        if self.current_pose is None or self.local_goal is None or self.scan_ranges is None:
            self.get_logger().warn('Missing data: pose=%s, goal=%s, scan=%s' % (self.current_pose is not None, self.local_goal is not None, self.scan_ranges is not None))
            return
        
        # Check if goal reached
        dist_to_goal = math.sqrt(
            (self.local_goal.position.x - self.current_pose.position.x)**2 +
            (self.local_goal.position.y - self.current_pose.position.y)**2
        )
        self.get_logger().debug('Distance to local goal: %.2f' % dist_to_goal)
        
        if dist_to_goal < self.goal_distance_threshold:
            self.get_logger().info('Local goal reached (dist: %.2f < %.2f). Stopping.' % (dist_to_goal, self.goal_distance_threshold))
            self.stop_robot()
            return
        
        # Compute attractive force
        attr_x, attr_y = self.attractive_potential()
        self.get_logger().debug('Attractive force: (x: %.2f, y: %.2f)' % (attr_x, attr_y))
        
        # Compute repulsive force
        rep_x, rep_y = self.repulsive_potential()
        self.get_logger().debug('Repulsive force: (x: %.2f, y: %.2f)' % (rep_x, rep_y))
        
        # Combine forces
        total_force_x = attr_x + rep_x
        total_force_y = attr_y + rep_y
        self.get_logger().debug('Total force: (x: %.2f, y: %.2f)' % (total_force_x, total_force_y))
        
        # Convert to velocity
        desired_heading = math.atan2(total_force_y, total_force_x)
        current_heading = self.get_yaw(self.current_pose.orientation)
        angular_vel = self.angular_control(desired_heading - current_heading)
        linear_vel = self.max_linear_speed * (1 - abs(angular_vel) / self.max_angular_speed)
        self.get_logger().debug('Computed velocities: linear=%.2f, angular=%.2f' % (linear_vel, angular_vel))
        
        # Publish velocity
        twist = Twist()
        twist.linear.x = min(linear_vel, self.max_linear_speed)
        twist.angular.z = angular_vel
        self.vel_pub.publish(twist)
        self.get_logger().info('Published cmd_vel: linear.x=%.2f, angular.z=%.2f' % (twist.linear.x, twist.angular.z))
        
        # Check if stuck
        current_vel = math.sqrt(twist.linear.x**2 + twist.angular.z**2)
        if current_vel < self.stuck_velocity_threshold:
            self.stuck_timer += 0.1
            self.get_logger().debug('Low velocity (%.2f < %.2f), stuck timer: %.2f' % (current_vel, self.stuck_velocity_threshold, self.stuck_timer))
            if self.stuck_timer > self.stuck_time:
                self.get_logger().warn('Robot stuck for %.2f s. Triggering replan.' % self.stuck_timer)
                self.trigger_replan()
                self.stuck_timer = 0.0
        else:
            self.stuck_timer = 0.0
            self.get_logger().debug('Robot moving, resetting stuck timer.')
        
        # Check deviation from goal
        if dist_to_goal > self.deviation_threshold:
            self.get_logger().warn('Deviation from goal too high (%.2f > %.2f). Triggering replan.' % (dist_to_goal, self.deviation_threshold))
            self.trigger_replan()

    def attractive_potential(self):
        dx = self.local_goal.position.x - self.current_pose.position.x
        dy = self.local_goal.position.y - self.current_pose.position.y
        dist = max(math.sqrt(dx**2 + dy**2), 0.001)  # Avoid division by zero
        force_x = self.attractive_gain * (dx / dist)
        force_y = self.attractive_gain * (dy / dist)
        return (force_x, force_y)

    def repulsive_potential(self):
        if self.scan_ranges is None:
            return (0.0, 0.0)
        force_x, force_y = 0.0, 0.0
        angle_min = -math.pi  # Assume 360 deg scan
        angle_increment = 2 * math.pi / len(self.scan_ranges)
        for i, r in enumerate(self.scan_ranges):
            if 0 < r < self.repulsive_distance:  # Valid range
                angle = angle_min + i * angle_increment
                magnitude = self.repulsive_gain * (1/r - 1/self.repulsive_distance) / (r**2)
                force_x -= magnitude * math.cos(angle)  # Negative to push away
                force_y -= magnitude * math.sin(angle)
        return (force_x, force_y)

    def angular_control(self, angle_error):
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
        return max(min(angle_error * 2.0, self.max_angular_speed), -self.max_angular_speed)

    def get_yaw(self, quat):
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def trigger_replan(self):
        if self.current_pose is None:
            self.get_logger().warn('Cannot trigger replan: No current pose.')
            return
        replan_msg = PoseWithCovarianceStamped()
        replan_msg.header.frame_id = 'map'
        replan_msg.header.stamp = self.get_clock().now().to_msg()
        replan_msg.pose.pose = self.current_pose
        replan_msg.pose.covariance = [0.1] * 36  # Dummy covariance
        self.replan_pub.publish(replan_msg)
        self.get_logger().info('Published current pose to /initialpose for replan.')

    def stop_robot(self):
        twist = Twist()
        self.vel_pub.publish(twist)
        self.get_logger().info('Published zero velocity to /cmd_vel.')

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()