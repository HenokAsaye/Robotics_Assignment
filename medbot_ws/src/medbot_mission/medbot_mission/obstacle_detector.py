#!/usr/bin/env python3
"""
Obstacle Detector Node
Module Owner: Netsanet (Obstacle Avoidance)

Advanced obstacle detection for Ethiopian urban environments.
Detects and classifies obstacles for safe navigation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import json
import math
from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class Obstacle:
    """Represents a detected obstacle"""
    id: int
    distance: float
    angle: float
    size: float
    x: float
    y: float
    category: str  # 'static', 'dynamic', 'pedestrian', 'vehicle'


class ObstacleDetector(Node):
    """
    Detects and classifies obstacles from LiDAR data.
    Optimized for Ethiopian urban environment challenges.
    """

    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Parameters
        self.declare_parameter('detection_range', 5.0)
        self.declare_parameter('cluster_threshold', 0.3)
        self.declare_parameter('min_cluster_size', 3)
        
        self.detection_range = self.get_parameter('detection_range').value
        self.cluster_threshold = self.get_parameter('cluster_threshold').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        
        # State
        self.obstacles: List[Obstacle] = []
        self.obstacle_id_counter = 0
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan',
            self.scan_callback, 10
        )
        
        # Publishers
        self.obstacle_pub = self.create_publisher(
            String, 'obstacles/detected', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, 'obstacles/markers', 10
        )
        self.warning_pub = self.create_publisher(
            String, 'obstacles/warnings', 10
        )
        
        self.get_logger().info('Obstacle Detector initialized')

    def scan_callback(self, msg: LaserScan):
        """Process LiDAR scan for obstacle detection"""
        # Convert scan to points
        points = self.scan_to_points(msg)
        
        # Cluster points into obstacles
        clusters = self.cluster_points(points)
        
        # Convert clusters to obstacles
        self.obstacles = []
        for cluster in clusters:
            if len(cluster) >= self.min_cluster_size:
                obstacle = self.cluster_to_obstacle(cluster)
                if obstacle:
                    self.obstacles.append(obstacle)
        
        # Publish results
        self.publish_obstacles()
        self.publish_markers()
        self.check_warnings()

    def scan_to_points(self, msg: LaserScan) -> List[Tuple[float, float]]:
        """Convert LaserScan to list of (x, y) points"""
        points = []
        angle = msg.angle_min
        
        for r in msg.ranges:
            if msg.range_min < r < min(msg.range_max, self.detection_range):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y, r, angle))
            angle += msg.angle_increment
        
        return points

    def cluster_points(self, points: List[Tuple]) -> List[List[Tuple]]:
        """Cluster points using simple distance-based clustering"""
        if not points:
            return []
        
        clusters = []
        used = [False] * len(points)
        
        for i, point in enumerate(points):
            if used[i]:
                continue
            
            cluster = [point]
            used[i] = True
            
            # Find nearby points
            for j, other_point in enumerate(points):
                if used[j]:
                    continue
                
                dist = math.sqrt(
                    (point[0] - other_point[0])**2 + 
                    (point[1] - other_point[1])**2
                )
                
                if dist < self.cluster_threshold:
                    cluster.append(other_point)
                    used[j] = True
            
            clusters.append(cluster)
        
        return clusters

    def cluster_to_obstacle(self, cluster: List[Tuple]) -> Obstacle:
        """Convert a point cluster to an Obstacle object"""
        if not cluster:
            return None
        
        # Calculate centroid
        x_sum = sum(p[0] for p in cluster)
        y_sum = sum(p[1] for p in cluster)
        n = len(cluster)
        
        center_x = x_sum / n
        center_y = y_sum / n
        
        # Calculate distance and angle to centroid
        distance = math.sqrt(center_x**2 + center_y**2)
        angle = math.atan2(center_y, center_x)
        
        # Estimate size from cluster spread
        max_x = max(p[0] for p in cluster)
        min_x = min(p[0] for p in cluster)
        max_y = max(p[1] for p in cluster)
        min_y = min(p[1] for p in cluster)
        size = max(max_x - min_x, max_y - min_y)
        
        # Classify obstacle
        category = self.classify_obstacle(size, distance)
        
        self.obstacle_id_counter += 1
        
        return Obstacle(
            id=self.obstacle_id_counter,
            distance=distance,
            angle=angle,
            size=size,
            x=center_x,
            y=center_y,
            category=category
        )

    def classify_obstacle(self, size: float, distance: float) -> str:
        """Classify obstacle based on size and characteristics"""
        # Simple classification based on size
        if size < 0.3:
            return 'small_object'
        elif size < 0.8:
            return 'pedestrian'
        elif size < 2.0:
            return 'vehicle'
        else:
            return 'structure'

    def publish_obstacles(self):
        """Publish detected obstacles"""
        obstacle_data = [{
            'id': obs.id,
            'distance': round(obs.distance, 2),
            'angle': round(math.degrees(obs.angle), 1),
            'size': round(obs.size, 2),
            'x': round(obs.x, 2),
            'y': round(obs.y, 2),
            'category': obs.category
        } for obs in self.obstacles]
        
        msg = String()
        msg.data = json.dumps({
            'count': len(self.obstacles),
            'obstacles': obstacle_data
        })
        self.obstacle_pub.publish(msg)

    def publish_markers(self):
        """Publish visualization markers"""
        markers = MarkerArray()
        
        # Clear old markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)
        
        for obs in self.obstacles:
            marker = Marker()
            marker.header.frame_id = 'lidar_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = obs.id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obs.x
            marker.pose.position.y = obs.y
            marker.pose.position.z = 0.5
            marker.scale.x = max(0.2, obs.size)
            marker.scale.y = max(0.2, obs.size)
            marker.scale.z = 1.0
            
            # Color based on category
            if obs.category == 'pedestrian':
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
            elif obs.category == 'vehicle':
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.5, 0.5, 0.5
            marker.color.a = 0.6
            
            marker.lifetime.sec = 1
            markers.markers.append(marker)
        
        self.marker_pub.publish(markers)

    def check_warnings(self):
        """Check for dangerous obstacles and publish warnings"""
        for obs in self.obstacles:
            if obs.distance < 1.0:
                warning = {
                    'type': 'close_obstacle',
                    'obstacle_id': obs.id,
                    'category': obs.category,
                    'distance': obs.distance
                }
                self.warning_pub.publish(String(data=json.dumps(warning)))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
