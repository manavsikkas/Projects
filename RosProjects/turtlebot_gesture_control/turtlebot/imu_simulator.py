#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
import math


class IMUSimulator(Node):
    def __init__(self):
        super().__init__('imu_simulator')
        
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'turtlebot/cmd_vel', self.cmd_vel_callback, 10)
        
        self.linear_velocity = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        self.noise_level = 0.01
        self.bias = np.array([0.001, 0.001, 0.001])
        
        self.timer = self.create_timer(0.02, self.publish_imu_data)
        
        self.get_logger().info('IMU Simulator initialized')
    
    def cmd_vel_callback(self, msg):
        self.linear_velocity[0] = msg.linear.x
        self.angular_velocity[2] = msg.angular.z
    
    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        accel_x = (self.linear_velocity[0] - 
                  np.random.normal(0, self.noise_level)) + self.bias[0]
        accel_y = np.random.normal(0, self.noise_level) + self.bias[1]
        accel_z = 9.81 + np.random.normal(0, self.noise_level) + self.bias[2]
        
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        
        gyro_x = np.random.normal(0, self.noise_level * 0.1)
        gyro_y = np.random.normal(0, self.noise_level * 0.1)
        gyro_z = self.angular_velocity[2] + np.random.normal(0, self.noise_level * 0.1)
        
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        
        yaw = math.atan2(2.0 * (self.orientation[3] * self.orientation[2]), 
                        1.0 - 2.0 * (self.orientation[2] * self.orientation[2]))
        yaw += gyro_z * 0.02
        
        self.orientation[2] = math.sin(yaw / 2.0)
        self.orientation[3] = math.cos(yaw / 2.0)
        
        imu_msg.orientation.x = self.orientation[0]
        imu_msg.orientation.y = self.orientation[1]
        imu_msg.orientation.z = self.orientation[2]
        imu_msg.orientation.w = self.orientation[3]
        
        covariance = np.zeros(9)
        covariance[0] = 0.001
        covariance[4] = 0.001
        covariance[8] = 0.001
        
        imu_msg.orientation_covariance = covariance.tolist()
        imu_msg.angular_velocity_covariance = covariance.tolist()
        imu_msg.linear_acceleration_covariance = covariance.tolist()
        
        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()