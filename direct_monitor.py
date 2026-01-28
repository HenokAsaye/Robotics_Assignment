#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime

class DirectMonitor(Node):
    def __init__(self):
        super().__init__('direct_monitor')
        
        try:
            self.status_sub = self.create_subscription(
                String, 'delivery/status', self.on_status, 10
            )
            print("✅ Subscribed to /delivery/status")
        except Exception as e:
            print(f"⚠️  Could not subscribe to /delivery/status: {e}")
        
        try:
            self.state_sub = self.create_subscription(
                String, 'delivery/state', self.on_state, 10
            )
            print("✅ Subscribed to /delivery/state")
        except Exception as e:
            print(f"⚠️  Could not subscribe to /delivery/state: {e}")        self.print_header()

    def print_header(self):
        print("\n" + "="*70)
        print("MEDBOT DELIVERY MONITOR")
        print("="*70)
        print("Listening for delivery status updates...\n")

    def on_status(self, msg):
        try:
            data = json.loads(msg.data)
            ts = datetime.now().strftime("%H:%M:%S")
            print(f"\n[{ts}] 📊 DELIVERY STATUS:")
            print(f"  State: {data.get('state', 'unknown')}")
            print(f"  Queue: {data.get('queue_length', 0)} items")
            print(f"  Completed: {data.get('completed_count', 0)}")
            if 'position' in data:
                print(f"  Position: ({data['position']['x']:.2f}, {data['position']['y']:.2f})")
        except json.JSONDecodeError:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Received status: {msg.data}")

    def on_state(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        state = msg.data.upper()
        
        # Emoji mapping
        emojis = {
            'IDLE': '🔴',
            'EN_ROUTE_PICKUP': '🚀',
            'AT_PICKUP': '📍',
            'LOADING': '📦',
            'EN_ROUTE_DELIVERY': '🚀',
            'AT_DELIVERY': '🏥',
            'UNLOADING': '📦',
            'RETURNING': '🏠',
            'COMPLETED': '✅',
            'FAILED': '❌',
        }
        
        emoji = emojis.get(state, '⚪')
        print(f"\n[{ts}] {emoji} STATE: {state}")

def main():
    rclpy.init()
    monitor = DirectMonitor()
    
    print("Spinning... (Press Ctrl+C to stop)\n")
    
    try:
        while True:
            rclpy.spin_once(monitor, timeout_sec=0.5)
    except KeyboardInterrupt:
        print("\n\n[STOPPED] Monitor closed by user")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
