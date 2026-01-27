#!/usr/bin/env python3
"""
Waypoint Publisher Node
Module Owner: Team Collaboration

Publishes predefined waypoints for delivery routes.
Provides waypoint visualization and route management.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

import json
import math
from typing import List, Tuple


class WaypointPublisher(Node):
    """
    Publishes and manages delivery waypoints.
    Provides predefined routes for common delivery scenarios.
    """
    
    # Predefined routes in the Addis Ababa simulation
    ROUTES = {
        'pharmacy_to_hospital': [
            (-8.0, -8.0, 0.0),      # Pharmacy
            (-8.0, 0.0, 1.57),      # Turn onto main road
            (0.0, 0.0, 0.0),        # Intersection
            (18.0, 0.0, 0.0),       # Near hospital
            (18.0, 8.0, 1.57),      # Hospital entrance
        ],
        'hospital_to_clinic': [
            (18.0, 8.0, 3.14),      # Hospital
            (0.0, 0.0, 3.14),       # Intersection
            (-16.0, 0.0, 1.57),     # Near clinic
            (-16.0, 8.0, 1.57),     # Clinic
        ],
        'clinic_to_pharmacy': [
            (-16.0, 8.0, -1.57),    # Clinic
            (-16.0, 0.0, 0.0),      # Main road
            (-8.0, 0.0, -1.57),     # Turn to pharmacy
            (-8.0, -8.0, -1.57),    # Pharmacy
        ],
        'patrol_route': [
            (0.0, 0.0, 0.0),        # Home base
            (10.0, 0.0, 0.0),       # East patrol
            (10.0, 5.0, 1.57),      # North-east
            (-10.0, 5.0, 3.14),     # North-west
            (-10.0, 0.0, -1.57),    # West
            (0.0, 0.0, 0.0),        # Return home
        ],
    }

    def __init__(self):
        super().__init__('waypoint_publisher')
        
        # Action client for waypoint following
        self.waypoint_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints'
        )
        
        # Publishers
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, 'waypoints_marker', 10
        )
        self.route_status_pub = self.create_publisher(
            String, 'route/status', 10
        )
        
        # Subscribers
        self.route_request_sub = self.create_subscription(
            String, 'route/request',
            self.route_request_callback, 10
        )
        
        # State
        self.current_route = None
        self.is_following = False
        
        # Visualization timer
        self.vis_timer = self.create_timer(1.0, self.publish_route_markers)
        
        self.get_logger().info('Waypoint Publisher initialized')
        self.get_logger().info(f'Available routes: {list(self.ROUTES.keys())}')

    def route_request_callback(self, msg: String):
        """Handle route execution requests"""
        try:
            request = json.loads(msg.data)
            route_name = request.get('route')
            
            if route_name not in self.ROUTES:
                self.get_logger().error(f'Unknown route: {route_name}')
                return
            
            self.execute_route(route_name)
            
        except json.JSONDecodeError:
            # Assume it's just the route name
            if msg.data in self.ROUTES:
                self.execute_route(msg.data)
            else:
                self.get_logger().error(f'Unknown route: {msg.data}')

    def execute_route(self, route_name: str):
        """Execute a predefined route"""
        if self.is_following:
            self.get_logger().warn('Already following a route. Cancel first.')
            return
        
        if not self.waypoint_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Waypoint follower action server not available!')
            return
        
        waypoints = self.ROUTES[route_name]
        self.current_route = route_name
        
        self.get_logger().info(f'Executing route: {route_name} ({len(waypoints)} waypoints)')
        
        # Create waypoint poses
        poses = []
        for x, y, theta in waypoints:
            poses.append(self.create_pose_stamped(x, y, theta))
        
        # Send goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
        
        self.is_following = True
        send_goal_future = self.waypoint_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Route goal rejected!')
            self.is_following = False
            return
        
        self.get_logger().info('Route goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle route following feedback"""
        feedback = feedback_msg.feedback
        current_wp = feedback.current_waypoint
        
        status = {
            'route': self.current_route,
            'current_waypoint': current_wp,
            'status': 'in_progress'
        }
        
        self.route_status_pub.publish(String(data=json.dumps(status)))
        self.get_logger().debug(f'At waypoint {current_wp}')

    def result_callback(self, future):
        """Handle route completion"""
        result = future.result().result
        missed = result.missed_waypoints
        
        self.is_following = False
        
        if len(missed) == 0:
            self.get_logger().info(f'Route {self.current_route} completed successfully!')
            status = 'completed'
        else:
            self.get_logger().warn(f'Route completed with {len(missed)} missed waypoints')
            status = 'partial'
        
        status_msg = {
            'route': self.current_route,
            'status': status,
            'missed_waypoints': list(missed)
        }
        self.route_status_pub.publish(String(data=json.dumps(status_msg)))
        self.current_route = None

    def publish_route_markers(self):
        """Publish waypoint markers for visualization"""
        markers = MarkerArray()
        
        for route_idx, (route_name, waypoints) in enumerate(self.ROUTES.items()):
            # Only show current route or all routes if none active
            if self.current_route and route_name != self.current_route:
                continue
            
            for wp_idx, (x, y, theta) in enumerate(waypoints):
                # Waypoint marker
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f'route_{route_name}'
                marker.id = wp_idx
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.1
                marker.pose.orientation.z = math.sin(theta / 2)
                marker.pose.orientation.w = math.cos(theta / 2)
                marker.scale.x = 0.5
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                
                # Color based on route
                if route_name == self.current_route:
                    marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
                else:
                    marker.color.r, marker.color.g, marker.color.b = 0.5, 0.5, 0.5
                marker.color.a = 0.8
                
                markers.markers.append(marker)
        
        self.waypoint_marker_pub.publish(markers)

    def create_pose_stamped(self, x: float, y: float, theta: float) -> PoseStamped:
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(theta / 2)
        pose.pose.orientation.w = math.cos(theta / 2)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
