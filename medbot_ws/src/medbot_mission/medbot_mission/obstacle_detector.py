#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import json
import math
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, field
from collections import deque

@dataclass
class TrackedObstacle:
    id: int
    x: float
    y: float
    distance: float
    angle: float
    size: float
    category: str
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    age: int = 0  # Frames since first detection
    last_seen: int = 0  # Frames since last update
    confidence: float = 1.0
    history: deque = field(default_factory=lambda: deque(maxlen=10))

    def update_position(self, x: float, y: float, size: float):
        if len(self.history) > 0:
            prev_x, prev_y, _ = self.history[-1]
            self.velocity_x = x - prev_x
            self.velocity_y = y - prev_y
        self.history.append((x, y, size))
        self.x = x
        self.y = y
        self.size = size
        self.distance = math.sqrt(x*x + y*y)
        self.angle = math.atan2(y, x)
        self.age += 1
        self.last_seen = 0
        self.confidence = min(1.0, self.confidence + 0.1)

class SpatialIndex:
    def __init__(self, cell_size: float = 0.5):
        self.cell_size = cell_size
        self.grid: Dict[Tuple[int, int], List[int]] = {}

    def clear(self):
        self.grid.clear()

    def _get_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(x / self.cell_size), int(y / self.cell_size))

    def insert(self, idx: int, x: float, y: float):
        cell = self._get_cell(x, y)
        if cell not in self.grid:
            self.grid[cell] = []
        self.grid[cell].append(idx)

    def get_neighbors(self, x: float, y: float) -> List[int]:
        cx, cy = self._get_cell(x, y)
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                cell = (cx + dx, cy + dy)
                if cell in self.grid:
                    neighbors.extend(self.grid[cell])
        return neighbors

class ObstacleDetector(Node):
    CATEGORY_SIZES = {
        'small_object': (0.0, 0.3),     # Stones, debris, small items
        'pedestrian': (0.3, 0.8),        # Walking people
        'bajaj': (0.8, 1.5),             # Three-wheeler taxis (common in Ethiopia)
        'vehicle': (1.5, 3.0),           # Cars, minibuses
        'structure': (3.0, float('inf')) # Buildings, walls, large objects
    }

    def __init__(self):
        super().__init__('obstacle_detector')

        self.declare_parameter('detection_range', 5.0)
        self.declare_parameter('cluster_threshold', 0.3)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('tracking_timeout', 5)  # Frames before losing track
        self.declare_parameter('association_threshold', 0.5)  # Max distance for track association
        self.declare_parameter('danger_zone_radius', 1.0)  # Critical proximity warning

        self.detection_range = self.get_parameter('detection_range').value
        self.cluster_threshold = self.get_parameter('cluster_threshold').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.tracking_timeout = self.get_parameter('tracking_timeout').value
        self.association_threshold = self.get_parameter('association_threshold').value
        self.danger_zone_radius = self.get_parameter('danger_zone_radius').value

        self.tracked_obstacles: Dict[int, TrackedObstacle] = {}
        self.next_obstacle_id = 1
        self.frame_count = 0
        self.spatial_index = SpatialIndex(cell_size=self.cluster_threshold * 2)

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan',
            self.scan_callback, 10
        )

        self.obstacle_pub = self.create_publisher(
            String, 'obstacles/detected', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, 'obstacles/markers', 10
        )
        self.warning_pub = self.create_publisher(
            String, 'obstacles/warnings', 10
        )
        self.danger_pub = self.create_publisher(
            String, 'obstacles/danger', 10
        )

        self.get_logger().info('Enhanced Obstacle Detector initialized')
        self.get_logger().info(f'Detection range: {self.detection_range}m')
        self.get_logger().info(f'Tracking enabled with {self.tracking_timeout} frame timeout')

    def scan_callback(self, msg: LaserScan):
        self.frame_count += 1

        points = self.scan_to_points(msg)

        clusters = self.cluster_points_spatial(points)

        detections = []
        for cluster in clusters:
            if len(cluster) >= self.min_cluster_size:
                detection = self.cluster_to_detection(cluster)
                if detection:
                    detections.append(detection)

        self.update_tracking(detections)

        self.publish_obstacles()
        self.publish_markers()
        self.check_warnings()

    def scan_to_points(self, msg: LaserScan) -> List[Tuple[float, float, float, float]]:
        points = []
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        for i, r in enumerate(msg.ranges):
            if i < len(angles) and msg.range_min < r < min(msg.range_max, self.detection_range):
                angle = angles[i]
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y, r, angle))

        return points

    def cluster_points_spatial(self, points: List[Tuple]) -> List[List[Tuple]]:
        if not points:
            return []

        self.spatial_index.clear()
        for i, point in enumerate(points):
            self.spatial_index.insert(i, point[0], point[1])

        clusters = []
        visited = set()

        for i, point in enumerate(points):
            if i in visited:
                continue

            cluster = []
            stack = [i]

            while stack:
                idx = stack.pop()
                if idx in visited:
                    continue
                visited.add(idx)
                cluster.append(points[idx])

                neighbors = self.spatial_index.get_neighbors(points[idx][0], points[idx][1])
                for n_idx in neighbors:
                    if n_idx not in visited:
                        dx = points[idx][0] - points[n_idx][0]
                        dy = points[idx][1] - points[n_idx][1]
                        if dx*dx + dy*dy < self.cluster_threshold * self.cluster_threshold:
                            stack.append(n_idx)

            if len(cluster) >= self.min_cluster_size:
                clusters.append(cluster)

        return clusters

    def cluster_to_detection(self, cluster: List[Tuple]) -> Optional[Dict]:
        if not cluster:
            return None

        coords = np.array([(p[0], p[1]) for p in cluster])
        center_x, center_y = np.mean(coords, axis=0)

        distance = math.sqrt(center_x**2 + center_y**2)
        angle = math.atan2(center_y, center_x)

        max_spread = np.max(np.ptp(coords, axis=0))
        size = max_spread

        category = self.classify_obstacle(size)

        return {
            'x': center_x,
            'y': center_y,
            'distance': distance,
            'angle': angle,
            'size': size,
            'category': category,
            'points': len(cluster)
        }

    def classify_obstacle(self, size: float) -> str:
        for category, (min_size, max_size) in self.CATEGORY_SIZES.items():
            if min_size <= size < max_size:
                return category
        return 'structure'

    def update_tracking(self, detections: List[Dict]):
        for track in self.tracked_obstacles.values():
            track.last_seen += 1

        used_tracks = set()
        for det in detections:
            best_track_id = None
            best_distance = self.association_threshold

            for track_id, track in self.tracked_obstacles.items():
                if track_id in used_tracks:
                    continue

                pred_x = track.x + track.velocity_x
                pred_y = track.y + track.velocity_y

                dx = det['x'] - pred_x
                dy = det['y'] - pred_y
                dist = math.sqrt(dx*dx + dy*dy)

                if dist < best_distance:
                    best_distance = dist
                    best_track_id = track_id

            if best_track_id is not None:
                self.tracked_obstacles[best_track_id].update_position(
                    det['x'], det['y'], det['size']
                )
                self.tracked_obstacles[best_track_id].category = det['category']
                used_tracks.add(best_track_id)
            else:
                new_track = TrackedObstacle(
                    id=self.next_obstacle_id,
                    x=det['x'],
                    y=det['y'],
                    distance=det['distance'],
                    angle=det['angle'],
                    size=det['size'],
                    category=det['category']
                )
                self.tracked_obstacles[self.next_obstacle_id] = new_track
                self.next_obstacle_id += 1

        stale_ids = [
            track_id for track_id, track in self.tracked_obstacles.items()
            if track.last_seen > self.tracking_timeout
        ]
        for track_id in stale_ids:
            del self.tracked_obstacles[track_id]

    def publish_obstacles(self):
        obstacle_data = []
        for track in self.tracked_obstacles.values():
            if track.last_seen == 0:  # Only publish currently visible obstacles
                obstacle_data.append({
                    'id': track.id,
                    'x': round(track.x, 3),
                    'y': round(track.y, 3),
                    'distance': round(track.distance, 2),
                    'angle_deg': round(math.degrees(track.angle), 1),
                    'size': round(track.size, 2),
                    'category': track.category,
                    'velocity': {
                        'x': round(track.velocity_x, 3),
                        'y': round(track.velocity_y, 3)
                    },
                    'age': track.age,
                    'confidence': round(track.confidence, 2)
                })

        msg = String()
        msg.data = json.dumps({
            'frame': self.frame_count,
            'count': len(obstacle_data),
            'tracked_total': len(self.tracked_obstacles),
            'obstacles': obstacle_data
        })
        self.obstacle_pub.publish(msg)

    def publish_markers(self):
        markers = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = 'lidar_link'
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        colors = {
            'small_object': (0.5, 0.5, 0.5),   # Grey
            'pedestrian': (0.0, 0.0, 1.0),      # Blue
            'bajaj': (0.0, 0.8, 0.0),           # Green (Ethiopian green)
            'vehicle': (1.0, 0.8, 0.0),         # Yellow (Ethiopian yellow)
            'structure': (0.8, 0.2, 0.2),       # Red (Ethiopian red)
        }

        for track in self.tracked_obstacles.values():
            if track.last_seen > 2:  # Don't show stale tracks
                continue

            marker = Marker()
            marker.header.frame_id = 'lidar_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = track.id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = track.x
            marker.pose.position.y = track.y
            marker.pose.position.z = 0.5
            marker.scale.x = max(0.2, track.size)
            marker.scale.y = max(0.2, track.size)
            marker.scale.z = 1.0

            r, g, b = colors.get(track.category, (0.5, 0.5, 0.5))
            marker.color.r, marker.color.g, marker.color.b = r, g, b
            marker.color.a = 0.7 * track.confidence

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000  # 200ms
            markers.markers.append(marker)

            if abs(track.velocity_x) > 0.01 or abs(track.velocity_y) > 0.01:
                arrow = Marker()
                arrow.header = marker.header
                arrow.ns = 'velocities'
                arrow.id = track.id + 10000
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD

                start = Point()
                start.x, start.y, start.z = track.x, track.y, 0.5
                end = Point()
                end.x = track.x + track.velocity_x * 5  # Scale for visibility
                end.y = track.y + track.velocity_y * 5
                end.z = 0.5
                arrow.points = [start, end]

                arrow.scale.x = 0.05  # Shaft diameter
                arrow.scale.y = 0.1   # Head diameter
                arrow.scale.z = 0.1   # Head length
                arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 0.0, 1.0
                arrow.color.a = 0.8

                arrow.lifetime.sec = 0
                arrow.lifetime.nanosec = 200000000
                markers.markers.append(arrow)

            text = Marker()
            text.header = marker.header
            text.ns = 'labels'
            text.id = track.id + 20000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = track.x
            text.pose.position.y = track.y
            text.pose.position.z = 1.2
            text.scale.z = 0.2
            text.color.r, text.color.g, text.color.b = 1.0, 1.0, 1.0
            text.color.a = 0.9
            text.text = f'{track.category}\n{track.distance:.1f}m'
            text.lifetime.sec = 0
            text.lifetime.nanosec = 200000000
            markers.markers.append(text)

        self.marker_pub.publish(markers)

    def check_warnings(self):
        for track in self.tracked_obstacles.values():
            if track.last_seen > 0:
                continue

            if track.distance < self.danger_zone_radius:
                danger = {
                    'type': 'danger_zone',
                    'obstacle_id': track.id,
                    'category': track.category,
                    'distance': round(track.distance, 2),
                    'angle_deg': round(math.degrees(track.angle), 1),
                    'severity': 'critical' if track.distance < 0.5 else 'warning'
                }
                self.danger_pub.publish(String(data=json.dumps(danger)))

                if track.distance < 0.5:
                    self.get_logger().warn(
                        f'CRITICAL: {track.category} at {track.distance:.2f}m!'
                    )

            elif track.distance < 1.5:
                warning = {
                    'type': 'close_obstacle',
                    'obstacle_id': track.id,
                    'category': track.category,
                    'distance': round(track.distance, 2)
                }
                self.warning_pub.publish(String(data=json.dumps(warning)))

            speed = math.sqrt(track.velocity_x**2 + track.velocity_y**2)
            if speed > 0.05:  # Moving obstacle
                approach_rate = -(track.x * track.velocity_x + track.y * track.velocity_y) / track.distance
                if approach_rate > 0.02:  # Approaching
                    warning = {
                        'type': 'approaching_obstacle',
                        'obstacle_id': track.id,
                        'category': track.category,
                        'distance': round(track.distance, 2),
                        'approach_rate': round(approach_rate, 3)
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
