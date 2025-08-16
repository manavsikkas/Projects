#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
import numpy as np


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        self.gesture_sub = self.create_subscription(
            String, 'hand_gesture', self.gesture_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.gnss_sub = self.create_subscription(
            NavSatFix, 'gnss/fix', self.gnss_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'robot_pose', 10)
        
        self.current_gesture = "STOP"
        self.imu_data = None
        self.gnss_data = None
        self.odom_data = None
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Robot Controller initialized')
    
    def gesture_callback(self, msg):
        self.current_gesture = msg.data
        self.get_logger().debug(f'Received gesture: {self.current_gesture}')
    
    def imu_callback(self, msg):
        self.imu_data = msg
        accel = msg.linear_acceleration
        gyro = msg.angular_velocity
        self.get_logger().debug(
            f'IMU - Accel: [{accel.x:.2f}, {accel.y:.2f}, {accel.z:.2f}], '
            f'Gyro: [{gyro.x:.2f}, {gyro.y:.2f}, {gyro.z:.2f}]'
        )
    
    def gnss_callback(self, msg):
        self.gnss_data = msg
        self.get_logger().debug(
            f'GNSS - Lat: {msg.latitude:.6f}, Lon: {msg.longitude:.6f}, Alt: {msg.altitude:.2f}'
        )
    
    def odom_callback(self, msg):
        self.odom_data = msg
    
    def control_loop(self):
        twist_msg = Twist()
        
        speed_factor = 1.0
        if self.imu_data:
            accel_magnitude = np.sqrt(
                self.imu_data.linear_acceleration.x**2 +
                self.imu_data.linear_acceleration.y**2
            )
            if accel_magnitude > 2.0:
                speed_factor = 0.5
        
        if self.current_gesture == "FORWARD":
            twist_msg.linear.x = 0.5 * speed_factor
            twist_msg.angular.z = 0.0
        elif self.current_gesture == "BACKWARD":
            twist_msg.linear.x = -0.5 * speed_factor
            twist_msg.angular.z = 0.0
        elif self.current_gesture == "LEFT":
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5 * speed_factor
        elif self.current_gesture == "RIGHT":
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -0.5 * speed_factor
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist_msg)
        
        if self.odom_data and self.gnss_data:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.pose = self.odom_data.pose.pose
            
            covariance = np.zeros(36)
            covariance[0] = 0.1
            covariance[7] = 0.1
            covariance[14] = 0.1
            covariance[21] = 0.05
            covariance[28] = 0.05
            covariance[35] = 0.05
            pose_msg.pose.covariance = covariance.tolist()
            
            self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()