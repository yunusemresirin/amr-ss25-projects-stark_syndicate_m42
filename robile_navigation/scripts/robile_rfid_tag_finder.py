#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from robile_interfaces.msg import PositionLabelled, PositionLabelledArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time

class RFIDTagFinder(Node):

    def __init__(self):
        super().__init__('rfid_tag_finder')

        # declaring and getting parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_frame', 'map'),
                ('odom_frame', 'odom'),
                ('laser_link_frame', 'base_laser_front_link'),
                ('real_base_link_frame', 'real_base_link'),
                ('scan_topic', 'scan'),
                ('odom_topic', 'odom'),
                ('cmd_vel_topic', 'cmd_vel'),
                ('rfid_tag_poses_topic', 'rfid_tag_poses'),
                ('initial_pose_topic', 'initialpose'),
                ('real_base_link_pose_topic', 'real_base_link_pose'),
                ('estimated_base_link_pose_topic', 'estimated_base_link_pose'),                
                ('laserscan_noise_stddev', 0.01),
                ('odom_noise_linear_stddev', 0.01),
                ('odom_noise_angular_stddev', 0.01),
                ('rfid_tags.A', [0.,0.]),
                ('rfid_tags.B', [0.,0.]),
                ('rfid_tags.C', [0.,0.]),
                ('rfid_tags.D', [0.,0.]),
                ('rfid_tags.E', [0.,0.])
            ])
        
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.laser_link_frame = self.get_parameter('laser_link_frame').get_parameter_value().string_value
        self.real_base_link_frame = self.get_parameter('real_base_link_frame').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.rfid_tag_poses_topic = self.get_parameter('rfid_tag_poses_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value
        self.real_base_link_pose_topic = self.get_parameter('real_base_link_pose_topic').get_parameter_value().string_value
        self.estimated_base_link_pose_topic = self.get_parameter('estimated_base_link_pose_topic').get_parameter_value().string_value
        self.laserscan_noise_stddev = self.get_parameter('laserscan_noise_stddev').get_parameter_value().double_value
        self.odom_noise_linear_stddev = self.get_parameter('odom_noise_linear_stddev').get_parameter_value().double_value
        self.odom_noise_angular_stddev = self.get_parameter('odom_noise_angular_stddev').get_parameter_value().double_value
        self.rfid_tags_A = self.get_parameter('rfid_tags.A').get_parameter_value().double_array_value
        self.rfid_tags_B = self.get_parameter('rfid_tags.B').get_parameter_value().double_array_value
        self.rfid_tags_C = self.get_parameter('rfid_tags.C').get_parameter_value().double_array_value
        self.rfid_tags_D = self.get_parameter('rfid_tags.D').get_parameter_value().double_array_value
        self.rfid_tags_E = self.get_parameter('rfid_tags.E').get_parameter_value().double_array_value

        # setting up subscribers and publishers
        self.odom_subscriber = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)        
        self.scan_subscriber = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.cmd_vel_subscriber = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.initial_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, self.initial_pose_topic, self.initial_pose_callback, 10) # self.initial_pose_callback is also called at the first instance of laser scan message with default PoseWithCovarianceStamped() message
        self.rfid_tag_publisher = self.create_publisher(PositionLabelledArray, self.rfid_tag_poses_topic, 10)
        self.real_base_link_pose_publisher = self.create_publisher(PoseStamped, self.real_base_link_pose_topic, 10)
        self.estimated_base_link_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, self.estimated_base_link_pose_topic, 10)

        # create dictionary of rfid points wrt odom frame in simulation (meters)
        self.rfid_points_dict = {}
        self.rfid_points_dict['A'] = self.rfid_tags_A
        self.rfid_points_dict['B'] = self.rfid_tags_B
        self.rfid_points_dict['C'] = self.rfid_tags_C
        self.rfid_points_dict['D'] = self.rfid_tags_D
        self.rfid_points_dict['E'] = self.rfid_tags_E

        # other variables
        self.scan_params_saved = False
        self.real_and_estimated_base_link_pose_initialised = False
        self.laser_wrt_base_link_found = False
        self.previous_position = [0., 0., 0.] # robot pose wrt map frame in real world (meters). It is used to get the distance travelled in a given time period using odom data
        self.cmd_vel_msg = Twist()

        # setting up tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # setting up tf2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def initial_pose_callback(self, msg):
        """
        This callback function is called when initial pose is set in rviz and the first laser scan message is received.
        Every time the initial pose is set in rviz, both the real_base_link and estimated_base_link poses are initialised.
        """
        yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.real_base_link_wrt_map = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw] # robot pose wrt map frame in real world (meters)

        base_link_pose = PoseStamped()
        base_link_pose.header.stamp = self.get_clock().now().to_msg()
        base_link_pose.header.frame_id = self.map_frame
        base_link_pose.pose.position.x = msg.pose.pose.position.x
        base_link_pose.pose.position.y = msg.pose.pose.position.y
        base_link_pose.pose.position.z = 0.
        base_link_pose.pose.orientation.x = msg.pose.pose.orientation.x
        base_link_pose.pose.orientation.y = msg.pose.pose.orientation.y
        base_link_pose.pose.orientation.z = msg.pose.pose.orientation.z
        base_link_pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.real_base_link_pose_publisher.publish(base_link_pose)

        estimated_base_link_pose = PoseWithCovarianceStamped()
        estimated_base_link_pose.header.stamp = self.get_clock().now().to_msg()
        estimated_base_link_pose.header.frame_id = self.map_frame
        estimated_base_link_pose.pose.pose.position.x = msg.pose.pose.position.x
        estimated_base_link_pose.pose.pose.position.y = msg.pose.pose.position.y
        estimated_base_link_pose.pose.pose.position.z = 0.
        estimated_base_link_pose.pose.pose.orientation.x = msg.pose.pose.orientation.x
        estimated_base_link_pose.pose.pose.orientation.y = msg.pose.pose.orientation.y
        estimated_base_link_pose.pose.pose.orientation.z = msg.pose.pose.orientation.z
        estimated_base_link_pose.pose.pose.orientation.w = msg.pose.pose.orientation.w
        estimated_base_link_pose.pose.covariance[0] = 0.0
        estimated_base_link_pose.pose.covariance[7] = 0.0
        estimated_base_link_pose.pose.covariance[35] = 0.0
        self.estimated_base_link_pose_publisher.publish(estimated_base_link_pose)

    def scan_callback(self, msg):
        """
        The callback function for incoming laser scan message
        """
        # setting real base_link pose frame to X-axis of map_frame
        if not self.real_and_estimated_base_link_pose_initialised:
            self.initial_pose_callback(PoseWithCovarianceStamped())
            self.real_and_estimated_base_link_pose_initialised = True

        # saving laser scan parameters
        if not self.scan_params_saved:
            self.min_angle = msg.angle_min
            self.max_angle = msg.angle_max
            self.angle_increment = msg.angle_increment
            self.scan_max_range = msg.range_max
            self.scan_params_saved = True

        # the static transform from laser_link to base_link is stored once. It is used in check_rfid_detection function
        if not self.laser_wrt_base_link_found:
            self.laser_pose_wrt_base_link = self.get_transform(self.laser_link_frame, 'base_link')
            if isinstance(self.laser_pose_wrt_base_link, bool):
                pass
            else:
                self.laser_wrt_base_link_found = True

        laser_link_wrt_map = self.get_transform(self.laser_link_frame, self.map_frame)

        # checking if transform is available before converting laser scan points to cartesian coordinates in odom frame
        if isinstance(laser_link_wrt_map, bool):
            if not laser_link_wrt_map:
                return False
        cartesian_coordinates_wrt_map = self.get_cartesian_coordinates(msg, laser_link_wrt_map)
        detected_rfid_tag_wrt_real_base_link = self.check_rfid_detection(cartesian_coordinates_wrt_map)

        # publishing the detected rfid tags wrt real_base_link
        position_arr_msg = PositionLabelledArray()
        position_arr_msg.header.stamp = self.get_clock().now().to_msg()
        # the distances of the rfid tags to the robot are with respect to the real robot location
        position_arr_msg.header.frame_id = self.real_base_link_frame
        if len(detected_rfid_tag_wrt_real_base_link) > 0:
            for rfid_point in detected_rfid_tag_wrt_real_base_link:
                position_msg = PositionLabelled()
                position_msg.name = rfid_point[2]
                position_msg.position.x = rfid_point[0]
                position_msg.position.y = rfid_point[1]
                position_msg.position.z = 0.0
                position_arr_msg.positions.append(position_msg)
            self.rfid_tag_publisher.publish(position_arr_msg)

    def odom_callback(self, msg):
        self.current_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]]
        # updating the real_base_link pose only if the robot is moving
        if self.cmd_vel_msg.linear.x != 0.0 or self.cmd_vel_msg.linear.y != 0.0 or self.cmd_vel_msg.angular.z != 0.0:
            self.publish_real_base_link_pose()
            self.previous_position = self.current_position            

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    def publish_real_base_link_pose(self):
        """
        Publishing both PoseStamped and TransformStamped messages.
        PoseStamped message is published to the topic /real_base_link_pose to visualise the pose in rviz.
        TransformStamped message is published to the topic /tf to transform the laser scan points in real_base_link frame.
        """

        # sampling noise from gaussian distribution
        linear_noise = np.random.normal(0.0, self.odom_noise_linear_stddev, 2)
        angular_noise = np.random.normal(0.0, self.odom_noise_angular_stddev, 1)

        # updating the transform based on motion model
        # updated real odom frame: [prev_position + delta_position + noise]
        del_x_wrt_odom = self.current_position[0] - self.previous_position[0]
        del_y_wrt_odom = self.current_position[1] - self.previous_position[1]
        del_theta = self.current_position[2] - self.previous_position[2]

        del_x_wrt_odom_with_noise = del_x_wrt_odom + np.clip(linear_noise[0], -np.abs(del_x_wrt_odom), np.abs(del_x_wrt_odom))
        del_y_wrt_odom_with_noise = del_y_wrt_odom + np.clip(linear_noise[1], -np.abs(del_y_wrt_odom), np.abs(del_y_wrt_odom))
        del_theta_with_noise = del_theta + np.clip(angular_noise[0], -np.abs(del_theta), np.abs(del_theta))

        # transforming del_x_with_noise and del_y_with_noise to real_base_link frame
        del_pos_wrt_base_link = np.array([[np.cos(self.current_position[2]), np.sin(self.current_position[2])], [-np.sin(self.current_position[2]), np.cos(self.current_position[2])]]) @ np.array([[del_x_wrt_odom_with_noise], [del_y_wrt_odom_with_noise]])

        del_pos_real_base_link_wrt_map = np.array([[np.cos(self.real_base_link_wrt_map[2]), -np.sin(self.real_base_link_wrt_map[2])], [np.sin(self.real_base_link_wrt_map[2]), np.cos(self.real_base_link_wrt_map[2])]]) @ del_pos_wrt_base_link
        del_pos_real_base_link_wrt_map = del_pos_real_base_link_wrt_map.reshape(-1)

        # updating the base_link pose
        self.real_base_link_wrt_map[0] += del_pos_real_base_link_wrt_map[0]
        self.real_base_link_wrt_map[1] += del_pos_real_base_link_wrt_map[1]
        self.real_base_link_wrt_map[2] += del_theta_with_noise
        quat_x, quat_y, quat_z, quat_w = quaternion_from_euler(0.0, 0.0, self.real_base_link_wrt_map[2])

        # publishing the real_base_link pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(self.real_base_link_wrt_map[0])
        msg.pose.position.y = float(self.real_base_link_wrt_map[1])
        msg.pose.orientation.x = quat_x
        msg.pose.orientation.y = quat_y
        msg.pose.orientation.z = quat_z
        msg.pose.orientation.w = quat_w
        self.real_base_link_pose_publisher.publish(msg)

        # broadcasting the transform 
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.real_base_link_frame
        t.transform.translation.x = float(self.real_base_link_wrt_map[0])
        t.transform.translation.y = float(self.real_base_link_wrt_map[1])
        t.transform.rotation.x = quat_x
        t.transform.rotation.y = quat_y
        t.transform.rotation.z = quat_z
        t.transform.rotation.w = quat_w
        self.tf_broadcaster.sendTransform(t)

    def get_cartesian_coordinates(self, scan_msg, laser_link_wrt_map):
        """
        Get cartesian coordinates of laser scan points wrt map frame
        """

        # getting distances and angles from laser scan message
        distances = np.array(scan_msg.ranges)
        angles = np.arange(self.min_angle, self.min_angle + len(scan_msg.ranges) * self.angle_increment, self.angle_increment)

        # getting x, y and theta of laser_link_wrt_map
        x_laser_link_wrt_map, y_laser_link_wrt_map, theta_laser_wrt_map = laser_link_wrt_map

        # getting cartesian coordinates of laser scan points wrt map frame
        cos_angles = np.cos(angles + theta_laser_wrt_map)
        sin_angles = np.sin(angles + theta_laser_wrt_map)
        
        # laser scan points wrt map frame in Cartesian coordinates
        distances_x_wrt_map = x_laser_link_wrt_map + distances * cos_angles
        distances_y_wrt_map = y_laser_link_wrt_map + distances * sin_angles

        cartesian_coordinates  = np.array([distances_x_wrt_map, distances_y_wrt_map]).T
        return cartesian_coordinates
    
    def get_transform(self, child_frame, parent_frame, return_transform=False):
        """
        Get the transform from the parent_frame to the child_frame
        """

        try:
            while not self.tf_buffer.can_transform(child_frame, parent_frame, tf2_ros.Time()):
                self.get_logger().warn('Waiting for transform from ' + parent_frame + ' to ' + child_frame + ' frame...')
                time.sleep(1.0)
                return False
            transform = self.tf_buffer.lookup_transform(parent_frame, child_frame, tf2_ros.Time())

            if return_transform:
                return transform
            else:
                yaw_wrt_map = euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])[2]
                return np.array([transform.transform.translation.x, transform.transform.translation.y, yaw_wrt_map])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(str(e))
            return None
        
    def check_rfid_detection(self, cartesian_coordinates_wrt_map):
        """
        Check if rfid point is detected between in the ray traced by laser scan
        """

        yaw = self.real_base_link_wrt_map[2]
        real_laser_link_wrt_real_base_link = self.laser_pose_wrt_base_link
        real_laser_link_wrt_map = np.array([[np.cos(yaw), np.sin(yaw), 0.0], [-np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]]) @ real_laser_link_wrt_real_base_link + self.real_base_link_wrt_map
        detected_rfid_tag_wrt_real_base_link = []

        for rfid_tag_name, rfid_point in self.rfid_points_dict.items():
            
            rfid_coordinates = np.array(rfid_point)

            # getting angle and distance of rfid point wrt laser_link_wrt_map
            rfid_point_wrt_real_base_link = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]]) @ (np.array(rfid_coordinates) - self.real_base_link_wrt_map[:2])
            rfid_point_wrt_real_laser_link = rfid_point_wrt_real_base_link - real_laser_link_wrt_real_base_link[:2] # As real_laser_link_wrt_real_base_link it has only linear transformation
            angle_wrt_real_laser_link = np.arctan2(rfid_point_wrt_real_laser_link[1], rfid_point_wrt_real_laser_link[0])
            dist_from_real_laser_link = np.linalg.norm(rfid_point_wrt_real_laser_link)

            # checking if angle between rfid point and real_laser_link_wrt_map is within self.min_angle and self.max_angle 
            if self.min_angle < angle_wrt_real_laser_link < self.max_angle and dist_from_real_laser_link < self.scan_max_range:
                # getting closest index of laser scan in same direction as rfid point
                scan_index = int((angle_wrt_real_laser_link - self.min_angle) / self.angle_increment)
                # check if distance between rfid point and real_laser_link_wrt_map is within the distance of laser scan point
                if dist_from_real_laser_link <= np.linalg.norm(np.array(cartesian_coordinates_wrt_map[scan_index])-real_laser_link_wrt_map[:2]):
                    # add Gaussian noise to rfid_point_wrt_real_base_link
                    rfid_point_wrt_real_base_link += np.random.normal(0, self.laserscan_noise_stddev, 2)
                    rfid_point_wrt_real_base_link_list = list(rfid_point_wrt_real_base_link)
                    rfid_point_wrt_real_base_link_list.append(rfid_tag_name)
                    detected_rfid_tag_wrt_real_base_link.append(rfid_point_wrt_real_base_link_list)

        return detected_rfid_tag_wrt_real_base_link


def main(args=None):
    rclpy.init(args=args)

    try:
        rfid_tag_finder_node = RFIDTagFinder()
        rclpy.spin(rfid_tag_finder_node)

    finally:
        rfid_tag_finder_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
