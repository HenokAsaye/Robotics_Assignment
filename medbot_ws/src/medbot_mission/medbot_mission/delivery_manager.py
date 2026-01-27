#!/usr/bin/env python3
"""
Delivery Manager Node
Module Owner: Netsanet (Obstacle Avoidance) / Team Collaboration

This node manages medical delivery missions for the autonomous robot.
It handles:
- Receiving delivery requests
- Planning delivery routes with multiple waypoints
- Monitoring delivery status
- Handling delivery completion and failures
- Coordinating with Nav2 for navigation

Designed for Ethiopian healthcare delivery scenarios.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes
from visualization_msgs.msg import Marker, MarkerArray

from enum import Enum
from dataclasses import dataclass
from typing import List, Optional
import json
import math


class DeliveryState(Enum):
    """Enumeration of delivery states"""
    IDLE = "idle"
    ACCEPTING = "accepting"
    EN_ROUTE_PICKUP = "en_route_pickup"
    AT_PICKUP = "at_pickup"
    LOADING = "loading"
    EN_ROUTE_DELIVERY = "en_route_delivery"
    AT_DELIVERY = "at_delivery"
    UNLOADING = "unloading"
    RETURNING = "returning"
    COMPLETED = "completed"
    FAILED = "failed"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class DeliveryRequest:
    """Data class for delivery request"""
    request_id: str
    pickup_location: PoseStamped
    delivery_location: PoseStamped
    priority: int  # 1 = highest (emergency), 5 = lowest
    package_type: str  # "medicine", "blood", "vaccine", "equipment"
    requester: str  # Healthcare facility name
    notes: str


@dataclass
class DeliveryLocation:
    """Predefined delivery locations in Addis Ababa simulation"""
    name: str
    x: float
    y: float
    theta: float


class DeliveryManager(Node):
    """
    Main delivery management node for the medical delivery robot.
    
    Coordinates navigation, monitors status, and manages delivery lifecycle.
    """
    
    # Predefined locations in the simulation world
    LOCATIONS = {
        'hospital': DeliveryLocation('Tikur Anbessa Hospital', 18.0, 8.0, 1.57),
        'clinic': DeliveryLocation('Health Clinic', -16.0, 8.0, 1.57),
        'pharmacy': DeliveryLocation('Pharmacy', -8.0, -8.0, -1.57),
        'home_base': DeliveryLocation('Home Base', 0.0, 0.0, 0.0),
        'shop_area': DeliveryLocation('Shop Area', 12.0, -8.0, -1.57),
    }

    def __init__(self):
        super().__init__('delivery_manager')
        
        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize state
        self.current_state = DeliveryState.IDLE
        self.current_delivery: Optional[DeliveryRequest] = None
        self.delivery_queue: List[DeliveryRequest] = []
        self.completed_deliveries: List[str] = []
        self.current_pose: Optional[PoseStamped] = None
        self.is_navigation_active = False
        
        # Parameters
        self.declare_parameter('delivery_timeout', 300.0)  # 5 minutes
        self.declare_parameter('loading_time', 5.0)  # seconds
        self.declare_parameter('unloading_time', 5.0)  # seconds
        self.declare_parameter('auto_return_home', True)
        
        self.delivery_timeout = self.get_parameter('delivery_timeout').value
        self.loading_time = self.get_parameter('loading_time').value
        self.unloading_time = self.get_parameter('unloading_time').value
        self.auto_return_home = self.get_parameter('auto_return_home').value
        
        # Action clients for Nav2
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        self.follow_waypoints_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints',
            callback_group=self.callback_group
        )
        
        # Publishers
        self.state_pub = self.create_publisher(
            String, 'delivery/state', 10
        )
        self.status_pub = self.create_publisher(
            String, 'delivery/status', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, 'delivery/markers', 10
        )
        self.emergency_pub = self.create_publisher(
            Bool, 'emergency_stop', 10
        )
        
        # Subscribers
        self.request_sub = self.create_subscription(
            String, 'delivery/request', 
            self.delivery_request_callback, 10,
            callback_group=self.callback_group
        )
        self.cancel_sub = self.create_subscription(
            String, 'delivery/cancel',
            self.cancel_callback, 10,
            callback_group=self.callback_group
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom',
            self.odom_callback, 10
        )
        self.emergency_sub = self.create_subscription(
            Bool, 'emergency_stop_trigger',
            self.emergency_callback, 10,
            callback_group=self.callback_group
        )
        
        # Timers
        self.status_timer = self.create_timer(
            1.0, self.publish_status,
            callback_group=self.callback_group
        )
        self.marker_timer = self.create_timer(
            2.0, self.publish_markers,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('Delivery Manager Node Initialized')
        self.get_logger().info('Ethiopian Medical Delivery Robot - Ready for Service')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Available locations: {list(self.LOCATIONS.keys())}')
        self.publish_state()

    def delivery_request_callback(self, msg: String):
        """Handle incoming delivery requests"""
        try:
            request_data = json.loads(msg.data)
            
            # Parse pickup location
            pickup_loc = self.LOCATIONS.get(request_data.get('pickup', 'pharmacy'))
            delivery_loc = self.LOCATIONS.get(request_data.get('delivery', 'hospital'))
            
            pickup_pose = self.create_pose_stamped(
                pickup_loc.x, pickup_loc.y, pickup_loc.theta
            )
            delivery_pose = self.create_pose_stamped(
                delivery_loc.x, delivery_loc.y, delivery_loc.theta
            )
            
            request = DeliveryRequest(
                request_id=request_data.get('id', f'DEL_{len(self.completed_deliveries)+1:04d}'),
                pickup_location=pickup_pose,
                delivery_location=delivery_pose,
                priority=request_data.get('priority', 3),
                package_type=request_data.get('package_type', 'medicine'),
                requester=request_data.get('requester', 'Unknown'),
                notes=request_data.get('notes', '')
            )
            
            self.delivery_queue.append(request)
            self.delivery_queue.sort(key=lambda x: x.priority)
            
            self.get_logger().info(
                f'New delivery request received: {request.request_id} '
                f'({request.package_type}) - Priority: {request.priority}'
            )
            
            # Start processing if idle
            if self.current_state == DeliveryState.IDLE:
                self.process_next_delivery()
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid delivery request format: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing delivery request: {e}')

    def process_next_delivery(self):
        """Process the next delivery in the queue"""
        if not self.delivery_queue:
            self.get_logger().info('No pending deliveries. Robot is idle.')
            self.set_state(DeliveryState.IDLE)
            if self.auto_return_home:
                self.return_to_home()
            return
        
        self.current_delivery = self.delivery_queue.pop(0)
        self.get_logger().info(
            f'Starting delivery: {self.current_delivery.request_id}'
        )
        
        # Navigate to pickup location
        self.set_state(DeliveryState.EN_ROUTE_PICKUP)
        self.navigate_to_pose(self.current_delivery.pickup_location)

    def navigate_to_pose(self, pose: PoseStamped):
        """Navigate to a specific pose using Nav2"""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available!')
            self.set_state(DeliveryState.FAILED)
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(
            f'Navigating to: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )
        
        self.is_navigation_active = True
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self.set_state(DeliveryState.FAILED)
            return
        
        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        
        if distance < 1.0:
            self.get_logger().debug(f'Distance to goal: {distance:.2f}m')

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        self.is_navigation_active = False
        result = future.result().result
        
        if self.current_state == DeliveryState.EN_ROUTE_PICKUP:
            self.get_logger().info('Arrived at pickup location!')
            self.set_state(DeliveryState.AT_PICKUP)
            # Simulate loading
            self.create_timer(
                self.loading_time, self.loading_complete,
                callback_group=self.callback_group
            )
            
        elif self.current_state == DeliveryState.EN_ROUTE_DELIVERY:
            self.get_logger().info('Arrived at delivery location!')
            self.set_state(DeliveryState.AT_DELIVERY)
            # Simulate unloading
            self.create_timer(
                self.unloading_time, self.unloading_complete,
                callback_group=self.callback_group
            )
            
        elif self.current_state == DeliveryState.RETURNING:
            self.get_logger().info('Returned to home base.')
            self.set_state(DeliveryState.IDLE)
            self.process_next_delivery()

    def loading_complete(self):
        """Called when loading is complete"""
        if self.current_state != DeliveryState.AT_PICKUP:
            return
            
        self.get_logger().info('Loading complete. Proceeding to delivery location.')
        self.set_state(DeliveryState.EN_ROUTE_DELIVERY)
        self.navigate_to_pose(self.current_delivery.delivery_location)

    def unloading_complete(self):
        """Called when unloading is complete"""
        if self.current_state != DeliveryState.AT_DELIVERY:
            return
            
        self.get_logger().info(
            f'Delivery {self.current_delivery.request_id} completed successfully!'
        )
        self.completed_deliveries.append(self.current_delivery.request_id)
        self.set_state(DeliveryState.COMPLETED)
        self.current_delivery = None
        
        # Process next delivery or return home
        self.process_next_delivery()

    def return_to_home(self):
        """Return to home base"""
        if self.current_state not in [DeliveryState.IDLE, DeliveryState.COMPLETED]:
            return
            
        home = self.LOCATIONS['home_base']
        home_pose = self.create_pose_stamped(home.x, home.y, home.theta)
        
        # Check if already at home
        if self.current_pose:
            dist = math.sqrt(
                (self.current_pose.pose.position.x - home.x)**2 +
                (self.current_pose.pose.position.y - home.y)**2
            )
            if dist < 0.5:
                self.get_logger().info('Already at home base.')
                return
        
        self.get_logger().info('Returning to home base...')
        self.set_state(DeliveryState.RETURNING)
        self.navigate_to_pose(home_pose)

    def cancel_callback(self, msg: String):
        """Handle delivery cancellation"""
        request_id = msg.data
        
        # Check if it's the current delivery
        if self.current_delivery and self.current_delivery.request_id == request_id:
            self.get_logger().warn(f'Cancelling current delivery: {request_id}')
            # Cancel navigation if active
            self.set_state(DeliveryState.IDLE)
            self.current_delivery = None
            self.process_next_delivery()
        else:
            # Remove from queue
            self.delivery_queue = [
                d for d in self.delivery_queue if d.request_id != request_id
            ]
            self.get_logger().info(f'Removed delivery {request_id} from queue')

    def emergency_callback(self, msg: Bool):
        """Handle emergency stop"""
        if msg.data:
            self.get_logger().error('EMERGENCY STOP ACTIVATED!')
            self.set_state(DeliveryState.EMERGENCY_STOP)
            # Publish emergency stop to robot
            self.emergency_pub.publish(Bool(data=True))
        else:
            self.get_logger().info('Emergency stop released.')
            self.set_state(DeliveryState.IDLE)

    def odom_callback(self, msg: Odometry):
        """Update current pose from odometry"""
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

    def set_state(self, new_state: DeliveryState):
        """Update delivery state"""
        old_state = self.current_state
        self.current_state = new_state
        self.get_logger().info(f'State: {old_state.value} -> {new_state.value}')
        self.publish_state()

    def publish_state(self):
        """Publish current state"""
        state_msg = String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)

    def publish_status(self):
        """Publish detailed status"""
        status = {
            'state': self.current_state.value,
            'queue_length': len(self.delivery_queue),
            'completed_count': len(self.completed_deliveries),
            'current_delivery': self.current_delivery.request_id if self.current_delivery else None,
            'navigation_active': self.is_navigation_active
        }
        
        if self.current_pose:
            status['position'] = {
                'x': round(self.current_pose.pose.position.x, 2),
                'y': round(self.current_pose.pose.position.y, 2)
            }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)

    def publish_markers(self):
        """Publish visualization markers for RViz"""
        markers = MarkerArray()
        
        # Publish location markers
        for i, (name, loc) in enumerate(self.LOCATIONS.items()):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'delivery_locations'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = loc.x
            marker.pose.position.y = loc.y
            marker.pose.position.z = 0.5
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 1.0
            
            # Color based on location type
            if 'hospital' in name:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            elif 'clinic' in name:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
            elif 'pharmacy' in name:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
            marker.color.a = 0.8
            
            markers.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'delivery_labels'
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = loc.x
            text_marker.pose.position.y = loc.y
            text_marker.pose.position.z = 1.5
            text_marker.scale.z = 0.5
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = loc.name
            
            markers.markers.append(text_marker)
        
        self.marker_pub.publish(markers)

    def create_pose_stamped(self, x: float, y: float, theta: float) -> PoseStamped:
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(theta / 2)
        pose.pose.orientation.w = math.cos(theta / 2)
        
        return pose


def main(args=None):
    rclpy.init(args=args)
    
    delivery_manager = DeliveryManager()
    
    executor = MultiThreadedExecutor()
    executor.add_node(delivery_manager)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        delivery_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
