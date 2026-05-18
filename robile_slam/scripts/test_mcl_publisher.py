#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math
import time

class TestMCLPublisher(Node):
    def __init__(self):
        super().__init__('test_mcl_publisher')
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Publish initial pose once after short delay
        self.create_timer(1.0, self._publish_initial_once)
        # Publish scan at 10 Hz
        self.create_timer(0.1, self._publish_scan)

        self._initial_sent = False
        self.get_logger().info('TestMCLPublisher started')

    def _publish_initial_once(self):
        if self._initial_sent:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        # 36 Elemente: setze kleine Unsicherheit
        msg.pose.covariance = [0.01 if i in (0,7,35) else 0.0 for i in range(36)]
        self.initial_pub.publish(msg)
        self.get_logger().info('Published initialpose (1.0, 0.0)')
        self._initial_sent = True

    def _publish_scan(self):
        # simple synthetic 360-beam scan: all ranges large (free space)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        num_readings = 360
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / float(num_readings)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.12
        scan.range_max = 10.0
        # set all ranges to 4.5 m (free)
        scan.ranges = [4.5] * num_readings
        # no intensities
        scan.intensities = []
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = TestMCLPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()