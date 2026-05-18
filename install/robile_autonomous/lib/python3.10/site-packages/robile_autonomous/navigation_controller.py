#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose # Correct import for Action
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup # Important for multi-threading
from rclpy.executors import MultiThreadedExecutor # Important for multi-threading
import threading # Not strictly used for node logic, but good to have if needed for other async tasks

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.get_logger().info("Navigation Controller node started.")

        # Using a ReentrantCallbackGroup allows callbacks (like goal_callback, pose_callback)
        # to be processed even if another callback (like waiting for action result) is blocking.
        # This is essential when using ActionClients with spin_until_future_complete or similar.
        self.callback_group = ReentrantCallbackGroup()
        
        # Action client for path planning
        # It's crucial to pass the callback_group to the ActionClient
        self.path_action_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose', # The name of the action server (from A_star_planner)
            callback_group=self.callback_group # Assign the callback group
        )
        self.get_logger().info("Created ComputePathToPose action client.")
        
        # Subscribers - assign callback_group to subscribers as well if they need to run concurrently
        # with action client operations. For simple cases, default group might be okay,
        # but for consistency and to avoid issues, it's good practice.
        self.pose_sub = self.create_subscription(
            PoseStamped, '/current_pose', self.pose_callback, 10,
            callback_group=self.callback_group # Assign callback group
        )
        self.get_logger().info("Subscribed to /current_pose topic.")
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10,
            callback_group=self.callback_group # Assign callback group
        )
        self.get_logger().info("Subscribed to /goal_pose topic.")
        
        self.current_pose = None
        self.goal_handle = None # To store the current goal handle
        self.get_logger().info("Navigation Controller initialized.")
    
    def pose_callback(self, msg):
        self.current_pose = msg
    
    def goal_callback(self, msg):
        if not self.current_pose:
            self.get_logger().warn("No current pose available to plan a path. Waiting for pose...")
            return
        
        self.get_logger().info(f"Received new goal position: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        
        self.get_logger().info("Waiting for ComputePathToPose action server...")
        # wait_for_server is blocking, but since we use a MultiThreadedExecutor,
        # other callbacks can still be processed.
        if not self.path_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ComputePathToPose action server not available. Is astar_planner node running?")
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = self.current_pose
        goal_msg.goal = msg
        
        self.get_logger().info("Sending goal to A* planner...")
        # send_goal_async returns a future. We add a done callback to process the result.
        self._send_goal_future = self.path_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server.')
            return

        self.get_logger().info('Goal accepted by action server.')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 2: # GoalStatus.SUCCEEDED
            self.get_logger().info(f"Path computed successfully! Path has {len(result.path.poses)} poses.")
        else:
            self.get_logger().error(f"Path computation failed with status: {status}")

    def feedback_callback(self, feedback_msg):
        # feedback = feedback_msg.feedback
        # self.get_logger().info(f"Received feedback: Path has {len(feedback.partial_path.poses)} poses so far.")
        pass

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor() # Create a multi-threaded executor
    navigation_controller = NavigationController()
    executor.add_node(navigation_controller) # Add the node to the executor
    
    try:
        executor.spin() # Spin the executor
    except KeyboardInterrupt:
        pass
    finally:
        navigation_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
