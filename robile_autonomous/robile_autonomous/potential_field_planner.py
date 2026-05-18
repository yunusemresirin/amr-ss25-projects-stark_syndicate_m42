#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import math

class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/global_path', self.path_callback, 10) # Subscribes to the path from A*
        self.pose_sub = self.create_subscription(
            PoseStamped, '/current_pose', self.pose_callback, 10) # Subscribes to robot's current pose
        
        # Publisher for robot velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control timer to periodically execute the control loop
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz control loop
        
        # Parameters for potential field calculation
        self.attractive_gain = 1.0       # Strength of attraction to goal
        self.repulsive_gain = 2.0        # Strength of repulsion from obstacles
        self.obstacle_threshold = 1.0    # Distance within which obstacles exert repulsive force (meters)
        self.max_linear_vel = 0.5        # Maximum linear velocity (m/s)
        self.max_angular_vel = 1.0       # Maximum angular velocity (rad/s)
        self.goal_tolerance = 0.3        # Distance to waypoint to consider it reached (meters)
        
        # State variables
        self.current_pose = None
        self.global_path = None
        self.laser_ranges = None
        self.current_goal_index = 0
        self.get_logger().info("Potential Field Planner node started.")
    
    def scan_callback(self, msg):
        """
        Callback for laser scan data.
        Updates the laser ranges, replacing infinite values with max_range.
        """
        self.laser_ranges = np.array(msg.ranges)
        # Replace inf values with max_range to handle out-of-range readings
        self.laser_ranges[np.isinf(self.laser_ranges)] = msg.range_max
    
    def path_callback(self, msg):
        """
        Callback for the global path received from the A* planner.
        Resets the current goal index when a new path is received.
        """
        self.global_path = msg
        self.current_goal_index = 0
        self.get_logger().info(f"Received new global path with {len(msg.poses)} waypoints.")
    
    def pose_callback(self, msg):
        """
        Callback for the robot's current pose.
        """
        self.current_pose = msg
    
    def get_current_goal(self):
        """
        Determines the current waypoint from the global path the robot should aim for.
        Advances to the next waypoint if the current one is reached.
        """
        if not self.global_path or not self.global_path.poses:
            return None # No global path available
        
        # If all waypoints are processed, return None (goal reached)
        if self.current_goal_index >= len(self.global_path.poses):
            return None
        
        current_goal = self.global_path.poses[self.current_goal_index]
        
        # Check if we're close to the current goal waypoint, move to next
        if self.current_pose:
            dx = current_goal.pose.position.x - self.current_pose.pose.position.x
            dy = current_goal.pose.position.y - self.current_pose.pose.position.y
            distance_to_waypoint = math.sqrt(dx*dx + dy*dy)
            
            if distance_to_waypoint < self.goal_tolerance:  # Within tolerance of waypoint
                self.current_goal_index += 1
                self.get_logger().info(f"Reached waypoint {self.current_goal_index - 1}. Moving to next.")
                # Recursively call to get the *new* current goal if available
                return self.get_current_goal() 
        
        return current_goal
    
    def calculate_attractive_force(self, goal):
        """
        Calculates the attractive force towards the current goal.
        """
        if not self.current_pose:
            return np.array([0.0, 0.0])
        
        dx = goal.pose.position.x - self.current_pose.pose.position.x
        dy = goal.pose.position.y - self.current_pose.pose.position.y
        
        # Simple attractive force proportional to distance
        return self.attractive_gain * np.array([dx, dy])
    
    def calculate_repulsive_force(self):
        """
        Calculates the repulsive force from nearby obstacles based on laser scan data.
        """
        repulsive_force = np.array([0.0, 0.0])
        if self.laser_ranges is None or not self.current_pose:
            return repulsive_force
        
        # Assuming laser scan angles range from -pi/2 to pi/2 (typical for front-facing lidar)
        # Adjust if your sensor has a different field of view.
        angle_increment = math.pi / len(self.laser_ranges) # Assuming 180 degree FOV
        
        for i, range_val in enumerate(self.laser_ranges):
            if range_val < self.obstacle_threshold and range_val > 0.01: # Avoid division by zero
                # Calculate angle of obstacle relative to robot's front
                angle = -math.pi/2 + i * angle_increment
                
                # Repulsive force magnitude (inverse square law, common in potential fields)
                # The force increases significantly as range_val approaches 0.
                force_magnitude = self.repulsive_gain * (1.0/range_val - 1.0/self.obstacle_threshold) * (1.0/(range_val**2))
                
                # Force direction (away from obstacle, relative to robot's heading)
                force_x = -force_magnitude * math.cos(angle)
                force_y = -force_magnitude * math.sin(angle)
                
                # Transform force from robot frame to world frame (assuming current_pose has orientation)
                # This is important if you want forces to be applied consistently in the world frame
                # rather than just relative to the robot's current heading.
                # For simplicity, if only angular velocity is derived from heading error,
                # then keeping it in robot's local frame might be fine.
                # For now, let's assume forces are calculated in the robot's local frame
                # and then used to derive linear/angular velocities.
                
                repulsive_force += np.array([force_x, force_y])
        
        return repulsive_force
    
    def control_loop(self):
        """
        Main control loop that calculates forces and publishes Twist commands.
        """
        current_goal = self.get_current_goal()
        
        # If no current goal (path finished or no path received) or no current pose, stop the robot.
        if not current_goal or not self.current_pose:
            cmd = Twist()
            # Only log stopping if there was a path and it's now completed, or if pose is missing.
            if self.global_path and self.current_goal_index >= len(self.global_path.poses):
                self.get_logger().info("Robot reached final goal or path completed. Stopping.")
            elif not self.current_pose:
                self.get_logger().warn("No current pose available. Robot stopped.")
            elif not self.global_path:
                self.get_logger().warn("No global path available. Robot stopped.")
            
            self.cmd_pub.publish(cmd)
            return
        
        # Calculate forces
        attractive_force = self.calculate_attractive_force(current_goal)
        repulsive_force = self.calculate_repulsive_force()
        
        # Total force is the sum of attractive and repulsive forces
        total_force = attractive_force + repulsive_force
        
        # Convert total force vector into linear and angular velocities
        cmd = Twist()
        
        # Linear velocity: magnitude of the total force, capped at max_linear_vel
        cmd.linear.x = min(np.linalg.norm(total_force), self.max_linear_vel)
        
        # Angular velocity: derived from the desired heading towards the total force vector
        if np.linalg.norm(total_force) > 0: # Avoid division by zero if total_force is zero
            desired_heading_world = math.atan2(total_force[1], total_force[0])
            
            # Get current heading from robot's quaternion orientation
            # The quaternion is (x, y, z, w)
            # Yaw (z-axis rotation) from quaternion: atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            # For 2D navigation, typically only z and w components are non-zero for yaw.
            current_heading_world = 2 * math.atan2(
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w)
            
            heading_error = desired_heading_world - current_heading_world
            
            # Normalize angle to be within [-pi, pi]
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi
            
            # Angular velocity is proportional to heading error, capped at max_angular_vel
            cmd.angular.z = max(min(heading_error, self.max_angular_vel), -self.max_angular_vel)
        
        self.get_logger().info(
            f"Moving: Linear X={cmd.linear.x:.2f}, Angular Z={cmd.angular.z:.2f} "
            f"Target Goal Index: {self.current_goal_index}/{len(self.global_path.poses) if self.global_path else 0}"
        )
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    potential_field_planner = PotentialFieldPlanner()
    rclpy.spin(potential_field_planner)
    potential_field_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
