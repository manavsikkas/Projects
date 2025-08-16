#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
import numpy as np
import math


class GNSSMock(Node):
    def __init__(self):
        super().__init__('gnss_mock')
        
        self.gnss_pub = self.create_publisher(NavSatFix, 'gnss/fix', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        self.base_latitude = 37.7749
        self.base_longitude = -122.4194
        self.base_altitude = 10.0
        
        self.meters_per_degree_lat = 111320.0
        self.meters_per_degree_lon = 111320.0 * math.cos(math.radians(self.base_latitude))
        
        self.position = [0.0, 0.0]
        
        self.noise_std_dev = 2.0
        self.drift_rate = 0.001
        self.accumulated_drift = [0.0, 0.0]
        
        self.timer = self.create_timer(1.0, self.publish_gnss_data)
        
        self.get_logger().info(f'GNSS Mock initialized at Lat: {self.base_latitude}, Lon: {self.base_longitude}')
    
    def odom_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
    
    def publish_gnss_data(self):
        gnss_msg = NavSatFix()
        gnss_msg.header.stamp = self.get_clock().now().to_msg()
        gnss_msg.header.frame_id = 'gnss_link'
        
        self.accumulated_drift[0] += np.random.normal(0, self.drift_rate)
        self.accumulated_drift[1] += np.random.normal(0, self.drift_rate)
        
        noise_x = np.random.normal(0, self.noise_std_dev)
        noise_y = np.random.normal(0, self.noise_std_dev)
        
        noisy_position_x = self.position[0] + noise_x + self.accumulated_drift[0]
        noisy_position_y = self.position[1] + noise_y + self.accumulated_drift[1]
        
        delta_lat = noisy_position_y / self.meters_per_degree_lat
        delta_lon = noisy_position_x / self.meters_per_degree_lon
        
        gnss_msg.latitude = self.base_latitude + delta_lat
        gnss_msg.longitude = self.base_longitude + delta_lon
        gnss_msg.altitude = self.base_altitude + np.random.normal(0, 0.5)
        
        gnss_msg.status.status = NavSatStatus.STATUS_FIX
        gnss_msg.status.service = NavSatStatus.SERVICE_GPS
        
        covariance = np.zeros(9)
        covariance[0] = self.noise_std_dev ** 2
        covariance[4] = self.noise_std_dev ** 2
        covariance[8] = 0.25
        gnss_msg.position_covariance = covariance.tolist()
        gnss_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gnss_pub.publish(gnss_msg)
        
        self.get_logger().debug(
            f'Published GNSS: Lat={gnss_msg.latitude:.6f}, '
            f'Lon={gnss_msg.longitude:.6f}, Alt={gnss_msg.altitude:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GNSSMock()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()