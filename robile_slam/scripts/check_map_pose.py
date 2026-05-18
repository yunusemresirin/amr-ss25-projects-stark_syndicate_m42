#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import argparse
import math
import sys
import time

class MapChecker(Node):
    def __init__(self, timeout=5.0):
        super().__init__('map_checker')
        self.map_msg = None
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)
        self.start = time.time()
        self.timeout = timeout

    def map_cb(self, msg):
        self.map_msg = msg

    def wait_map(self):
        while rclpy.ok() and self.map_msg is None and (time.time() - self.start) < self.timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.map_msg

def world_to_grid(map_msg, x, y):
    r = map_msg.info.resolution
    ox = map_msg.info.origin.position.x
    oy = map_msg.info.origin.position.y
    gx = int(math.floor((x - ox) / r))
    gy = int(math.floor((y - oy) / r))
    return gx, gy

def grid_to_index(map_msg, gx, gy):
    w = map_msg.info.width
    h = map_msg.info.height
    if gx < 0 or gy < 0 or gx >= w or gy >= h:
        return None
    return gy * w + gx

def is_free(map_msg, x, y):
    gx, gy = world_to_grid(map_msg, x, y)
    idx = grid_to_index(map_msg, gx, gy)
    if idx is None:
        return None, (gx, gy)
    val = map_msg.data[idx]
    # Occupancy: 0 free, 100 occ, -1 unknown
    return val == 0, (gx, gy, val)

def find_nearest_free(map_msg, x, y, max_radius_m=2.0):
    r = map_msg.info.resolution
    max_cells = int(max_radius_m / r)
    ox = map_msg.info.origin.position.x
    oy = map_msg.info.origin.position.y
    gx0, gy0 = world_to_grid(map_msg, x, y)
    w = map_msg.info.width
    h = map_msg.info.height
    for radius in range(0, max_cells+1):
        for dx in range(-radius, radius+1):
            for dy in [-radius, radius] if abs(dx) != radius else range(-radius, radius+1):
                gx = gx0 + dx
                gy = gy0 + dy
                if 0 <= gx < w and 0 <= gy < h:
                    idx = grid_to_index(map_msg, gx, gy)
                    if idx is None:
                        continue
                    if map_msg.data[idx] == 0:
                        wx = ox + (gx + 0.5) * r
                        wy = oy + (gy + 0.5) * r
                        return wx, wy, gx, gy
    return None

def main():
    parser = argparse.ArgumentParser(description='Check pose against /map')
    parser.add_argument('--x', type=float, required=True, help='world x (m)')
    parser.add_argument('--y', type=float, required=True, help='world y (m)')
    parser.add_argument('--find-free', action='store_true', help='find nearest free cell if input is bad')
    parser.add_argument('--timeout', type=float, default=5.0, help='map wait timeout (s)')
    args = parser.parse_args()

    rclpy.init()
    checker = MapChecker(timeout=args.timeout)
    try:
        m = checker.wait_map()
        if m is None:
            print('Keine /map empfangen (timeout). Starte test_map_publisher oder map_server.')
            sys.exit(2)
        free, info = is_free(m, args.x, args.y)
        if free is None:
            print(f'Pose ({args.x},{args.y}) außerhalb der Karte. Grid-Koords: {info[0]},{info[1]}')
        elif free is True:
            print(f'Pose ({args.x},{args.y}) ist INNEN und FREI. Grid: {info[0]},{info[1]} (val={info[2]})')
            # provide ROS2 publish snippets
            print('\nPublish-Beispiel (initialpose):')
            cov36 = '[0.01,0,0,0,0,0,' + '0,0.01,0,0,0,0,' + '0,0,0.01' + ',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]'
            print(f"ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \"{{header: {{frame_id: 'map'}}, pose: {{ pose: {{ position: {{ x: {args.x}, y: {args.y}, z: 0.0 }}, orientation: {{ w: 1.0 }} }}, covariance: {cov36} }} }}\" --once")
            print('\nPublish-Beispiel (goal_pose):')
            print(f"ros2 topic pub /goal_pose geometry_msgs/PoseStamped \"{{header: {{frame_id: 'map'}}, pose: {{ position: {{ x: {args.x}, y: {args.y}, z: 0.0 }}, orientation: {{ w: 1.0 }} }} }}\" --once")
        else:
            print(f'Pose ({args.x},{args.y}) IST BELEGT/UNKOWN. value={info[2]} at grid {info[0]},{info[1]}')
            if args.find_free:
                res = find_nearest_free(m, args.x, args.y, max_radius_m=3.0)
                if res:
                    wx, wy, gx, gy = res
                    print(f'Nächste freie Zelle: world ({wx:.3f},{wy:.3f}), grid ({gx},{gy})')
                    print(f"ros2 topic pub /goal_pose geometry_msgs/PoseStamped \"{{header: {{frame_id: 'map'}}, pose: {{ position: {{ x: {wx:.3f}, y: {wy:.3f}, z: 0.0 }}, orientation: {{ w: 1.0 }} }} }}\" --once")
                else:
                    print('Kein freier Punkt im Suchradius gefunden.')
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()