#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.delivery_pub = self.create_publisher(String, 'delivery/request', 10)

    def move(self, linear_x=0.3, angular_z=0.0, duration=3.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def send_delivery(self):
        request = {
            "pickup": "pharmacy",
            "delivery": "hospital",
            "priority": 1,
            "package_type": "medicine"
        }
        msg = String()
        msg.data = json.dumps(request)
        
        for _ in range(5):
            self.delivery_pub.publish(msg)
            time.sleep(0.1)

def main():
    rclpy.init()
    mover = RobotMover()
    
    print("Moving robot forward (3 seconds)...")
    mover.move(linear_x=0.4, duration=3.0)
    
    print("Rotating robot (2 seconds)...")
    mover.move(angular_z=0.5, duration=2.0)
    
    print("Sending delivery request...")
    mover.send_delivery()
    
    print("✅ Done")
    
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
