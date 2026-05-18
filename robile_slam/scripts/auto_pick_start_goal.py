import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import math
import random
import argparse
import time

class AutoPicker(Node):
    def __init__(self, min_distance=3.0, clearance=0.3, timeout=5.0, seed=None):
        super().__init__('auto_pick_start_goal')
        self.map = None
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.timeout = timeout
        self.min_distance = float(min_distance)
        self.clearance = float(clearance)
        if seed is not None:
            random.seed(seed)

    def map_cb(self, msg):
        self.map = msg

    def wait_map(self):
        start = time.time()
        while rclpy.ok() and self.map is None and (time.time() - start) < self.timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.map

    def grid_from_world(self, x, y):
        r = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        gx = int(math.floor((x - ox) / r))
        gy = int(math.floor((y - oy) / r))
        return gx, gy

    def world_from_grid(self, gx, gy):
        r = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        wx = ox + (gx + 0.5) * r
        wy = oy + (gy + 0.5) * r
        return wx, wy

    def index(self, gx, gy):
        w = self.map.info.width
        h = self.map.info.height
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return None
        return gy * w + gx

    def is_cell_free_with_clearance(self, gx, gy, clearance_cells):
        w = self.map.info.width
        h = self.map.info.height
        for dx in range(-clearance_cells, clearance_cells + 1):
            for dy in range(-clearance_cells, clearance_cells + 1):
                cx = gx + dx
                cy = gy + dy
                idx = self.index(cx, cy)
                if idx is None:
                    return False
                val = self.map.data[idx]
                # require strictly free (0); avoid occupied (100) and unknown (-1)
                if val != 0:
                    return False
        return True

    def pick_pair(self):
        # gather all free grid cells
        free_cells = []
        w = self.map.info.width
        h = self.map.info.height
        for gy in range(h):
            for gx in range(w):
                idx = self.index(gx, gy)
                if idx is None:
                    continue
                if self.map.data[idx] == 0:
                    free_cells.append((gx, gy))
        if not free_cells:
            return None

        # filter by clearance
        clearance_cells = max(0, int(math.ceil(self.clearance / self.map.info.resolution)))
        candidates = []
        for gx, gy in free_cells:
            if self.is_cell_free_with_clearance(gx, gy, clearance_cells):
                wx, wy = self.world_from_grid(gx, gy)
                candidates.append((gx, gy, wx, wy))

        if len(candidates) < 2:
            return None

        # choose two with at least min_distance
        random.shuffle(candidates)
        for i in range(len(candidates)):
            for j in range(i+1, len(candidates)):
                _, _, x1, y1 = candidates[i]
                _, _, x2, y2 = candidates[j]
                if math.hypot(x2 - x1, y2 - y1) >= self.min_distance:
                    return (x1, y1), (x2, y2)

        return None

    def publish_pair(self, start, goal):
        # initialpose (PoseWithCovarianceStamped)
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = 'map'
        ip.header.stamp = self.get_clock().now().to_msg()
        ip.pose.pose.position.x = float(start[0])
        ip.pose.pose.position.y = float(start[1])
        ip.pose.pose.orientation.w = 1.0
        # simple covariance: small uncertainty on x,y and yaw
        cov = [0.01]*36
        cov[0] = 0.01
        cov[7] = 0.01
        cov[35] = 0.01
        ip.pose.covariance = cov
        self.pub_init.publish(ip)

        # goal (PoseStamped)
        gp = PoseStamped()
        gp.header.frame_id = 'map'
        gp.header.stamp = self.get_clock().now().to_msg()
        gp.pose.position.x = float(goal[0])
        gp.pose.position.y = float(goal[1])
        gp.pose.orientation.w = 1.0
        self.pub_goal.publish(gp)

        self.get_logger().info(f'Published initialpose: ({start[0]:.3f}, {start[1]:.3f})')
        self.get_logger().info(f'Published goal_pose:   ({goal[0]:.3f}, {goal[1]:.3f})')
        # print ros2 topic pub snippets for reuse
        cov36 = '[' + ','.join(['0.01' if i in (0,7,35) else '0.0' for i in range(36)]) + ']'
        print("ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "
              f"\"{{header: {{frame_id: 'map'}}, pose: {{ pose: {{ position: {{ x: {start[0]:.3f}, y: {start[1]:.3f}, z: 0.0 }}, orientation: {{ w: 1.0 }} }}, covariance: {cov36} }} }}\" --once")
        print("ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
              f"\"{{header: {{frame_id: 'map'}}, pose: {{ position: {{ x: {goal[0]:.3f}, y: {goal[1]:.3f}, z: 0.0 }}, orientation: {{ w: 1.0 }} }} }}\" --once")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--min-distance', type=float, default=3.0, help='min distance between start and goal (m)')
    parser.add_argument('--clearance', type=float, default=0.3, help='required free clearance around points (m)')
    parser.add_argument('--timeout', type=float, default=5.0, help='map wait timeout (s)')
    parser.add_argument('--seed', type=int, default=None, help='random seed')
    args = parser.parse_args()

    rclpy.init()
    node = AutoPicker(min_distance=args.min_distance, clearance=args.clearance, timeout=args.timeout, seed=args.seed)
    try:
        m = node.wait_map()
        if m is None:
            node.get_logger().error('Keine /map empfangen (timeout).')
            return 1
        pair = node.pick_pair()
        if not pair:
            node.get_logger().error('Keine geeigneten Start/Goal-Paare gefunden. Reduziere clearance/min-distance oder überprüfe Karte.')
            return 2
        start, goal = pair
        node.publish_pair(start, goal)
        # give publishers time
        time.sleep(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0

if __name__ == '__main__':
    main()