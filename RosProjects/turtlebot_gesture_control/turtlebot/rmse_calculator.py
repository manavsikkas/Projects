#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
import numpy as np
import math


class RMSECalculator(Node):
    def __init__(self):
        super().__init__('rmse_calculator')
        
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.gnss_sub = self.create_subscription(
            NavSatFix, 'gnss/fix', self.gnss_callback, 10)
        
        self.rmse_pub = self.create_publisher(Float64, 'localization/rmse', 10)
        self.error_pub = self.create_publisher(PointStamped, 'localization/error', 10)
        
        self.ground_truth_position = None
        self.gnss_position = None
        
        self.base_latitude = 37.7749
        self.base_longitude = -122.4194
        self.meters_per_degree_lat = 111320.0
        self.meters_per_degree_lon = 111320.0 * math.cos(math.radians(self.base_latitude))
        
        self.error_history = []
        self.max_history_size = 100
        
        self.timer = self.create_timer(0.5, self.calculate_rmse)
        
        self.get_logger().info('RMSE Calculator initialized')
    
    def odom_callback(self, msg):
        self.ground_truth_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
    
    def gnss_callback(self, msg):
        delta_lat = msg.latitude - self.base_latitude
        delta_lon = msg.longitude - self.base_longitude
        
        x = delta_lon * self.meters_per_degree_lon
        y = delta_lat * self.meters_per_degree_lat
        z = msg.altitude - 10.0
        
        self.gnss_position = [x, y, z]
    
    def calculate_rmse(self):
        if self.ground_truth_position is None or self.gnss_position is None:
            return
        
        error_x = self.gnss_position[0] - self.ground_truth_position[0]
        error_y = self.gnss_position[1] - self.ground_truth_position[1]
        error_z = self.gnss_position[2] - self.ground_truth_position[2]
        
        squared_error = error_x**2 + error_y**2 + error_z**2
        
        self.error_history.append(squared_error)
        if len(self.error_history) > self.max_history_size:
            self.error_history.pop(0)
        
        rmse = math.sqrt(np.mean(self.error_history))
        
        rmse_msg = Float64()
        rmse_msg.data = rmse
        self.rmse_pub.publish(rmse_msg)
        
        error_msg = PointStamped()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.header.frame_id = 'map'
        error_msg.point.x = error_x
        error_msg.point.y = error_y
        error_msg.point.z = error_z
        self.error_pub.publish(error_msg)
        
        self.get_logger().info(
            f'RMSE: {rmse:.3f}m | Current Error: X={error_x:.2f}m, Y={error_y:.2f}m, Z={error_z:.2f}m'
        )
        
        if rmse > 5.0:
            self.get_logger().warn(f'High RMSE detected: {rmse:.3f}m')


def main(args=None):
    rclpy.init(args=args)
    node = RMSECalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()