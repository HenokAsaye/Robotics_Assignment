#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        
        self.declare_parameter('min_obstacle_distance', 0.30)  # Increased from 0.25m to reduce LiDAR noise false positives
        self.declare_parameter('emergency_enabled', True)
        
        self.min_distance = self.get_parameter('min_obstacle_distance').value
        self.enabled = self.get_parameter('emergency_enabled').value
        
        self.emergency_active = False
        self.last_obstacle_distance = float('inf')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan',
            self.scan_callback, 10
        )
        self.trigger_sub = self.create_subscription(
            Bool, 'emergency_stop_trigger',
            self.trigger_callback, 10
        )
        
        self.stop_timer = self.create_timer(0.1, self.publish_stop)
        
        self.get_logger().info('Emergency Stop node initialized')
        self.get_logger().info(f'Minimum obstacle distance: {self.min_distance}m')

    def scan_callback(self, msg: LaserScan):
        if not self.enabled:
            return
        
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid_ranges:
            return
        
        min_range = min(valid_ranges)
        self.last_obstacle_distance = min_range
        
        if min_range < self.min_distance and not self.emergency_active:
            self.get_logger().error(
                f'EMERGENCY: Obstacle at {min_range:.2f}m! Stopping robot.'
            )
            self.activate_emergency()

    def trigger_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('Manual emergency stop triggered!')
            self.activate_emergency()
        else:
            self.get_logger().info('Emergency stop released.')
            self.deactivate_emergency()

    def activate_emergency(self):
        self.emergency_active = True
        self.emergency_pub.publish(Bool(data=True))
        
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def deactivate_emergency(self):
        if self.last_obstacle_distance > self.min_distance * 1.5:
            self.emergency_active = False
            self.emergency_pub.publish(Bool(data=False))
            self.get_logger().info('Emergency stop deactivated. Robot can resume.')
        else:
            self.get_logger().warn(
                'Cannot deactivate: obstacle still too close '
                f'({self.last_obstacle_distance:.2f}m)'
            )

    def publish_stop(self):
        if self.emergency_active:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
