import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import tf2_ros
import tf_transformations
import math

class ParticleFilter(Node):
    def __init__(self):
        super().__init__('particle_filter_node')
        # Parameters
        self.num_particles = self.declare_parameter('num_particles', 500).value
        self.laser_max_range = self.declare_parameter('laser_max_range', 10.0).value
        # State
        self.particles = None  # [x, y, theta, weight]
        self.map = None
        self.odom = None
        self.scan = None
        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.particle_cloud_pub = self.create_publisher(PoseArray, '/particle_cloud', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        # Timer
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info('Particle Filter node started with %d particles.' % self.num_particles)

    def map_callback(self, msg):
        if not msg.data or msg.info.width == 0 or msg.info.height == 0:
            self.get_logger().warn('Received invalid map: Empty or zero dimensions. Waiting for valid map.')
            return
        self.get_logger().info('Received map. Initializing particles uniformly.')
        self.map = msg
        # Initialize particles if not already set
        if self.particles is None:
            self.particles = np.zeros((self.num_particles, 4))  # [x, y, theta, weight]
            # Uniformly distribute particles across free space
            width, height = msg.info.width, msg.info.height
            res = msg.info.resolution
            ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
            free_cells = [(i % width, i // width) for i, val in enumerate(msg.data) if val == 0]
            if not free_cells:
                self.get_logger().warn('No free cells in map. Cannot initialize particles.')
                self.particles = None
                return
            indices = np.random.choice(len(free_cells), self.num_particles)
            for i, idx in enumerate(indices):
                x, y = free_cells[idx]
                self.particles[i, 0] = x * res + ox + res / 2.0
                self.particles[i, 1] = y * res + oy + res / 2.0
                self.particles[i, 2] = np.random.uniform(-np.pi, np.pi)
                self.particles[i, 3] = 1.0 / self.num_particles
            self.get_logger().info('Particles initialized: %d particles.' % self.num_particles)

    def odom_callback(self, msg):
        self.odom = msg
        self.get_logger().debug('Received odometry: Position (x: %.2f, y: %.2f)' % (msg.pose.pose.position.x, msg.pose.pose.position.y))

    def scan_callback(self, msg):
        self.scan = msg
        self.get_logger().debug('Received scan with %d ranges.' % len(msg.ranges))

    def initial_pose_callback(self, msg):
        if self.map is None or self.particles is None:
            self.get_logger().warn('Ignoring initial pose: Map or particles not ready. Waiting for map...')
            # Schedule a retry
            self.create_timer(0.5, lambda: self.initial_pose_callback(msg))
            return
        self.get_logger().info('Received initial pose: Position (x: %.2f, y: %.2f). Reinitializing particles.' % (msg.pose.pose.position.x, msg.pose.pose.position.y))
        pose = msg.pose.pose
        samples = np.random.multivariate_normal(
            [pose.position.x, pose.position.y, self.get_yaw(pose.orientation)],
            np.diag([0.1, 0.1, 0.05]),
            self.num_particles
        )
        self.particles[:, :3] = samples
        self.particles[:, 3] = 1.0 / self.num_particles
    
    def update(self):
        if self.particles is None or self.odom is None or self.scan is None or self.map is None:
            self.get_logger().debug('Update skipped: Waiting for particles=%s, odom=%s, scan=%s, map=%s' % (
                self.particles is not None, self.odom is not None, self.scan is not None, self.map is not None))
            return
        # Motion update
        delta = self.compute_motion()
        self.particles[:, :3] += delta + np.random.normal(0, [0.05, 0.05, 0.02], (self.num_particles, 3))
        # Measurement update
        weights = self.compute_weights()
        self.particles[:, 3] = weights / np.sum(weights)
        # Publish pose and particle cloud
        self.publish_pose()
        # Resample if needed
        if 1.0 / np.sum(self.particles[:, 3]**2) < self.num_particles / 2:
            self.resample()

    def compute_motion(self):
        if self.odom is None:
            return np.zeros((self.num_particles, 3))
        # Simplified: Assume odometry delta since last update
        delta_x = self.odom.twist.twist.linear.x * 0.1  # Timer period
        delta_y = self.odom.twist.twist.linear.y * 0.1
        delta_theta = self.odom.twist.twist.angular.z * 0.1
        return np.array([delta_x, delta_y, delta_theta] * self.num_particles).reshape(self.num_particles, 3)

    def compute_weights(self):
        if self.scan is None or self.map is None:
            return self.particles[:, 3]
        weights = np.ones(self.num_particles)
        width = self.map.info.width
        res = self.map.info.resolution
        ox, oy = self.map.info.origin.position.x, self.map.info.origin.position.y
        for i in range(self.num_particles):
            x, y, theta = self.particles[i, :3]
            gx = int((x - ox) / res)
            gy = int((y - oy) / res)
            if 0 <= gx < width and 0 <= gy < self.map.info.height:
                idx = gy * width + gx
                if self.map.data[idx] >= 50:
                    weights[i] *= 0.01  # Penalize particles in obstacles
            else:
                weights[i] *= 0.01  # Out of bounds
            # Simplified laser likelihood
            for j, r in enumerate(self.scan.ranges):
                if r > 0 and r < self.laser_max_range:
                    angle = self.scan.angle_min + j * self.scan.angle_increment + theta
                    rx = x + r * math.cos(angle)
                    ry = y + r * math.sin(angle)
                    rgx = int((rx - ox) / res)
                    rgy = int((ry - oy) / res)
                    if 0 <= rgx < width and 0 <= rgy < self.map.info.height:
                        ridx = rgy * width + rgx
                        if self.map.data[ridx] >= 50:
                            weights[i] *= 0.9
                        else:
                            weights[i] *= 0.1
        return weights

    def resample(self):
        indices = np.random.choice(self.num_particles, size=self.num_particles, p=self.particles[:, 3])
        self.particles = self.particles[indices]
        self.particles[:, 3] = 1.0 / self.num_particles
        self.get_logger().info('Resampled particles.')

    def publish_pose(self):
        if self.particles is None or self.odom is None:
            self.get_logger().warn('Cannot publish pose: Particles or odom missing.')
            return
        # Weighted average pose
        weights = self.particles[:, 3]
        mean_x = np.sum(self.particles[:, 0] * weights) / np.sum(weights)
        mean_y = np.sum(self.particles[:, 1] * weights) / np.sum(weights)
        mean_theta = np.arctan2(
            np.sum(np.sin(self.particles[:, 2]) * weights),
            np.sum(np.cos(self.particles[:, 2]) * weights)
        )
        # Publish amcl_pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = mean_x
        pose_msg.pose.pose.position.y = mean_y
        pose_msg.pose.pose.orientation = self.yaw_to_quaternion(mean_theta)
        pose_msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
        self.pose_pub.publish(pose_msg)
        self.get_logger().info('Published amcl_pose: (x: %.2f, y: %.2f, yaw: %.2f)' % (mean_x, mean_y, mean_theta))
        # Publish particle cloud
        cloud_msg = PoseArray()
        cloud_msg.header.frame_id = 'map'
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.poses = [self.array_to_pose(p) for p in self.particles]
        self.particle_cloud_pub.publish(cloud_msg)
        # Publish TF
        t = TransformStamped()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.translation.x = mean_x
        t.transform.translation.y = mean_y
        t.transform.rotation = self.yaw_to_quaternion(mean_theta)
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug('Published TF: map -> odom (x: %.2f, y: %.2f, yaw: %.2f)' % (mean_x, mean_y, mean_theta))

    def get_yaw(self, quat):
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y**2 + quat.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        q = Pose().orientation
        q.w = math.cos(yaw / 2)
        q.z = math.sin(yaw / 2)
        return q

    def array_to_pose(self, particle):
        pose = Pose()
        pose.position.x = particle[0]
        pose.position.y = particle[1]
        pose.position.z = 0.0
        pose.orientation = self.yaw_to_quaternion(particle[2])
        return pose

def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()