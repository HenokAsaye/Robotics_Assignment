#!/usr/bin/env python3
"""
Status Monitor Node
Module Owner: Team Collaboration

Monitors robot health, sensor status, and system performance.
Publishes aggregated status for dashboard display.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan, Imu, BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import json
from datetime import datetime


class StatusMonitor(Node):
    """
    Monitors and aggregates robot status information.
    Provides health checks and diagnostic data.
    """

    def __init__(self):
        super().__init__('status_monitor')
        
        # Status data
        self.status = {
            'timestamp': '',
            'system_health': 'OK',
            'sensors': {
                'lidar': {'status': 'unknown', 'last_update': None},
                'imu': {'status': 'unknown', 'last_update': None},
                'camera': {'status': 'unknown', 'last_update': None},
            },
            'navigation': {
                'odometry': False,
                'velocity': {'linear': 0.0, 'angular': 0.0},
            },
            'battery': {
                'percentage': 100.0,
                'voltage': 12.0,
                'charging': False,
            },
            'warnings': [],
            'errors': [],
        }
        
        # Timeouts (seconds)
        self.sensor_timeout = 5.0
        
        # Last update times
        self.last_lidar_time = None
        self.last_imu_time = None
        self.last_odom_time = None
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan',
            self.scan_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data',
            self.imu_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom',
            self.odom_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel',
            self.cmd_vel_callback, 10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, 'robot/status', 10
        )
        self.health_pub = self.create_publisher(
            String, 'robot/health', 10
        )
        self.alert_pub = self.create_publisher(
            String, 'robot/alerts', 10
        )
        
        # Status update timer
        self.status_timer = self.create_timer(1.0, self.update_status)
        
        self.get_logger().info('Status Monitor initialized')

    def scan_callback(self, msg: LaserScan):
        """Handle LiDAR scan data"""
        self.last_lidar_time = self.get_clock().now()
        self.status['sensors']['lidar']['status'] = 'OK'
        self.status['sensors']['lidar']['last_update'] = str(datetime.now())
        
        # Check for obstacles too close
        min_range = min(msg.ranges)
        if min_range < 0.3:
            self.add_warning('Obstacle very close: {:.2f}m'.format(min_range))

    def imu_callback(self, msg: Imu):
        """Handle IMU data"""
        self.last_imu_time = self.get_clock().now()
        self.status['sensors']['imu']['status'] = 'OK'
        self.status['sensors']['imu']['last_update'] = str(datetime.now())

    def odom_callback(self, msg: Odometry):
        """Handle odometry data"""
        self.last_odom_time = self.get_clock().now()
        self.status['navigation']['odometry'] = True
        
        # Extract velocity
        self.status['navigation']['velocity']['linear'] = round(
            msg.twist.twist.linear.x, 3
        )
        self.status['navigation']['velocity']['angular'] = round(
            msg.twist.twist.angular.z, 3
        )

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands"""
        pass  # Just monitoring

    def add_warning(self, message: str):
        """Add a warning message"""
        if message not in self.status['warnings']:
            self.status['warnings'].append(message)
            if len(self.status['warnings']) > 10:
                self.status['warnings'].pop(0)

    def add_error(self, message: str):
        """Add an error message"""
        if message not in self.status['errors']:
            self.status['errors'].append(message)
            self.get_logger().error(message)

    def update_status(self):
        """Update and publish status"""
        now = self.get_clock().now()
        self.status['timestamp'] = str(datetime.now())
        
        # Clear old warnings
        self.status['warnings'] = []
        
        # Check sensor timeouts
        if self.last_lidar_time:
            lidar_age = (now - self.last_lidar_time).nanoseconds / 1e9
            if lidar_age > self.sensor_timeout:
                self.status['sensors']['lidar']['status'] = 'TIMEOUT'
                self.add_warning('LiDAR data timeout')
        
        if self.last_imu_time:
            imu_age = (now - self.last_imu_time).nanoseconds / 1e9
            if imu_age > self.sensor_timeout:
                self.status['sensors']['imu']['status'] = 'TIMEOUT'
                self.add_warning('IMU data timeout')
        
        if self.last_odom_time:
            odom_age = (now - self.last_odom_time).nanoseconds / 1e9
            if odom_age > self.sensor_timeout:
                self.status['navigation']['odometry'] = False
                self.add_warning('Odometry timeout')
        
        # Determine overall health
        if self.status['errors']:
            self.status['system_health'] = 'ERROR'
        elif self.status['warnings']:
            self.status['system_health'] = 'WARNING'
        else:
            self.status['system_health'] = 'OK'
        
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps(self.status)
        self.status_pub.publish(status_msg)
        
        # Publish health summary
        health_msg = String()
        health_msg.data = self.status['system_health']
        self.health_pub.publish(health_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
