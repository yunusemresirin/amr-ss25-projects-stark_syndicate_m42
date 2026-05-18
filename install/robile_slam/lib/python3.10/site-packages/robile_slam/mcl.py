import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

import numpy as np
from math import atan2


# =============================
# Utility functions for yaw <-> quaternion conversions
# =============================
def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw angle (rotation around Z) from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a quaternion from a yaw angle (2D orientation)."""
    half = 0.5 * yaw
    cy = np.cos(half)
    sy = np.sin(half)
    return Quaternion(x=0.0, y=0.0, z=float(sy), w=float(cy))


# =============================
# Particle representation
# =============================
class Particle:
    def __init__(self, pose: Pose, weight: float):
        self.pose = pose  # Robot pose (x, y, orientation)
        self.weight = weight  # Importance weight


# =============================
# Main Particle Filter Localizer
# =============================
class ParticleFilterLocalizer(Node):
    def __init__(self):
        super().__init__('particle_filter_localizer')  # ROS 2 node name

        # Parameters for particle filter
        self.particle_count = 500
        self.motion_noise_x = 0.05  # Motion noise in x
        self.motion_noise_y = 0.05  # Motion noise in y
        self.motion_noise_yaw = 0.02  # Motion noise in orientation
        self.laser_sigma = 0.2  # Sensor noise standard deviation

        # Particle storage
        self.particles = []
        self.initialized = False

        # Last odometry pose (to compute delta motion)
        self.last_odom_pose = None

        # Latest laser scan message
        self.latest_scan = None

        # Map storage
        self.map_data = None

        # QoS for map subscription (latched topic)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile=map_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.init_pose_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher for particle visualization
        self.particles_pub = self.create_publisher(PoseArray, '/particle_cloud', 10)

        # Timer to periodically publish particles
        self.create_timer(0.5, self.publish_particles)

    # =============================
    # Map callback
    # =============================
    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg  # Store map for simulated scans

    # =============================
    # Initial pose callback
    # =============================
    def init_pose_callback(self, msg: PoseWithCovarianceStamped):
        init_pose = msg.pose.pose
        self.initialize_particles(init_pose)
        self.initialized = True

    # =============================
    # Odometry callback
    # =============================
    def odom_callback(self, msg: Odometry):
        if not self.initialized:
            return

        current_pose = msg.pose.pose
        if self.last_odom_pose is not None:
            # Compute motion delta between two odometry readings
            dx, dy, dyaw = self.calculate_motion_delta(self.last_odom_pose, current_pose)
            # Apply motion update to all particles
            self.predict_particles(dx, dy, dyaw)
        self.last_odom_pose = current_pose

    # =============================
    # Laser scan callback
    # =============================
    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        if self.initialized:
            # Update particle weights using measurement likelihood
            self.update_weights()
            # Resample particles according to weights
            self.resample_particles()

    # =============================
    # Particle initialization
    # =============================
    def initialize_particles(self, init_pose: Pose):
        self.particles = []
        for _ in range(self.particle_count):
            p = Pose()
            p.position.x = init_pose.position.x + np.random.normal(0, 0.2)
            p.position.y = init_pose.position.y + np.random.normal(0, 0.2)
            yaw = yaw_from_quaternion(init_pose.orientation) + np.random.normal(0, 0.1)
            p.orientation = quaternion_from_yaw(yaw)
            self.particles.append(Particle(p, 1.0 / self.particle_count))

    # =============================
    # Predict motion of all particles (motion update)
    # =============================
    def predict_particles(self, dx, dy, dyaw):
        for particle in self.particles:
            particle.pose.position.x += dx + np.random.normal(0, self.motion_noise_x)
            particle.pose.position.y += dy + np.random.normal(0, self.motion_noise_y)
            yaw = yaw_from_quaternion(particle.pose.orientation)
            yaw += dyaw + np.random.normal(0, self.motion_noise_yaw)
            particle.pose.orientation = quaternion_from_yaw(yaw)

    # =============================
    # Update weights using laser scan likelihood
    # =============================
    def update_weights(self):
        if self.latest_scan is None or self.map_data is None:
            return

        weights = []
        for particle in self.particles:
            simulated_scan = self.simulate_scan(particle.pose, self.latest_scan)
            likelihood = self.compute_likelihood(self.latest_scan, simulated_scan)
            particle.weight = likelihood
            weights.append(likelihood)

        # Normalize weights
        total = sum(weights)
        if total > 0:
            for particle in self.particles:
                particle.weight /= total

    # =============================
    # Simulate a laser scan from particle pose
    # =============================
    def simulate_scan(self, pose: Pose, scan: LaserScan):
        # For simplicity, assume perfect expected range (stub function)
        return np.full(len(scan.ranges), 1.0)

    # =============================
    # Compute likelihood between real and simulated scan
    # =============================
    def compute_likelihood(self, real_scan: LaserScan, simulated_scan: np.ndarray):
        prob = 1.0
        for r, s in zip(real_scan.ranges, simulated_scan):
            if real_scan.range_min < r < real_scan.range_max:
                error = r - s
                prob *= np.exp(-(error ** 2) / (2 * self.laser_sigma ** 2))
        return prob

    # =============================
    # Resampling step
    # =============================
    def resample_particles(self):
        new_particles = []
        N = len(self.particles)
        weights = [p.weight for p in self.particles]
        cumulative = np.cumsum(weights)
        step = 1.0 / N
        start = np.random.uniform(0, step)
        i = 0
        for j in range(N):
            u = start + j * step
            while u > cumulative[i]:
                i += 1
            pose = self.particles[i].pose
            new_particles.append(Particle(pose, 1.0 / N))
        self.particles = new_particles

    # =============================
    # Publish particles as PoseArray for visualization
    # =============================
    def publish_particles(self):
        if not self.particles:
            return
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.poses = [p.pose for p in self.particles]
        self.particles_pub.publish(msg)

    # =============================
    # Compute motion delta from odometry
    # =============================
    def calculate_motion_delta(self, start: Pose, end: Pose):
        dx = end.position.x - start.position.x
        dy = end.position.y - start.position.y
        start_yaw = yaw_from_quaternion(start.orientation)
        end_yaw = yaw_from_quaternion(end.orientation)
        dyaw = end_yaw - start_yaw
        return dx, dy, dyaw


# =============================
# Main entry point
# =============================
def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
