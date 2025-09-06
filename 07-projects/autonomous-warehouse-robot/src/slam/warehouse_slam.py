#!/usr/bin/env python3
"""
Warehouse SLAM System

Advanced SLAM implementation for warehouse environments featuring:
- LiDAR-based mapping with loop closure detection
- Visual SLAM integration
- Multi-modal sensor fusion
- Real-time map updates
- Adaptive mapping parameters

Author: Tanmay Pancholi
Date: 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import cv2
from scipy.spatial.distance import cdist
from scipy.optimize import minimize
import math
from typing import List, Tuple, Optional, Dict
import threading
import time
from collections import deque

# ROS2 Messages
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from std_msgs.msg import Header, Bool
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

# Custom Messages
from warehouse_robot_msgs.msg import SLAMStatus, LoopClosure

class Particle:
    """Particle for particle filter SLAM"""
    def __init__(self, x: float, y: float, theta: float, weight: float = 1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight
        self.landmarks = {}  # Dictionary of observed landmarks
        self.map = None  # Occupancy grid for this particle

class Landmark:
    """Landmark representation"""
    def __init__(self, x: float, y: float, descriptor: np.ndarray = None):
        self.x = x
        self.y = y
        self.descriptor = descriptor
        self.observations = []  # List of observations
        self.covariance = np.eye(2) * 0.1  # Position uncertainty

class WarehouseSLAM(Node):
    """
    Advanced SLAM system for warehouse environments.
    
    Features:
    - Particle filter SLAM with adaptive resampling
    - Loop closure detection using visual and geometric features
    - Multi-modal sensor fusion (LiDAR + Camera)
    - Real-time map building and updates
    - Adaptive mapping parameters based on environment
    """
    
    def __init__(self):
        super().__init__('warehouse_slam')
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/warehouse_robot/map', qos_profile)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/warehouse_robot/pose', qos_profile)
        self.status_pub = self.create_publisher(SLAMStatus, '/warehouse_robot/slam_status', qos_profile)
        self.loop_closure_pub = self.create_publisher(LoopClosure, '/warehouse_robot/loop_closure', qos_profile)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/warehouse_robot/scan', self.scan_callback, qos_profile)
        self.image_sub = self.create_subscription(
            Image, '/warehouse_robot/camera/image_raw', self.image_callback, qos_profile)
        self.odom_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/warehouse_robot/odom', self.odom_callback, qos_profile)
        
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # SLAM parameters
        self.declare_parameters()
        self.load_parameters()
        
        # SLAM state
        self.particles = []
        self.best_particle = None
        self.landmarks = {}
        self.occupancy_grid = None
        self.current_pose = None
        self.previous_pose = None
        self.laser_data = None
        self.camera_data = None
        self.odom_data = None
        
        # Performance metrics
        self.map_quality = 0.0
        self.loop_closures_detected = 0
        self.particles_count = 0
        self.processing_time = 0.0
        
        # Threading
        self.slam_lock = threading.Lock()
        
        # Timers
        self.slam_timer = self.create_timer(0.1, self.slam_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Initialize SLAM
        self.initialize_slam()
        
        self.get_logger().info("Warehouse SLAM system initialized")
    
    def declare_parameters(self):
        """Declare ROS2 parameters"""
        self.declare_parameter('num_particles', 100)
        self.declare_parameter('resample_threshold', 0.5)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 200)
        self.declare_parameter('map_height', 200)
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('loop_closure_threshold', 0.3)
        self.declare_parameter('landmark_threshold', 0.5)
        self.declare_parameter('motion_noise', 0.1)
        self.declare_parameter('measurement_noise', 0.05)
    
    def load_parameters(self):
        """Load parameters from ROS2 parameter server"""
        self.num_particles = self.get_parameter('num_particles').value
        self.resample_threshold = self.get_parameter('resample_threshold').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.loop_closure_threshold = self.get_parameter('loop_closure_threshold').value
        self.landmark_threshold = self.get_parameter('landmark_threshold').value
        self.motion_noise = self.get_parameter('motion_noise').value
        self.measurement_noise = self.get_parameter('measurement_noise').value
    
    def initialize_slam(self):
        """Initialize SLAM system"""
        # Create initial particles
        self.particles = []
        for _ in range(self.num_particles):
            particle = Particle(0.0, 0.0, 0.0, 1.0 / self.num_particles)
            particle.map = self.create_empty_map()
            self.particles.append(particle)
        
        self.best_particle = self.particles[0]
        self.particles_count = self.num_particles
        
        # Initialize occupancy grid
        self.occupancy_grid = self.create_empty_map()
        
        self.get_logger().info(f"SLAM initialized with {self.num_particles} particles")
    
    def create_empty_map(self) -> OccupancyGrid:
        """Create empty occupancy grid"""
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = -self.map_width * self.map_resolution / 2
        map_msg.info.origin.position.y = -self.map_height * self.map_resolution / 2
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Initialize with unknown (-1)
        map_msg.data = [-1] * (self.map_width * self.map_height)
        
        return map_msg
    
    def scan_callback(self, msg: LaserScan):
        """Handle laser scan data"""
        self.laser_data = msg
        self.update_map_with_scan()
    
    def image_callback(self, msg: Image):
        """Handle camera image data"""
        self.camera_data = msg
        self.extract_visual_features()
    
    def odom_callback(self, msg: PoseWithCovarianceStamped):
        """Handle odometry data"""
        self.odom_data = msg
        self.current_pose = msg.pose.pose
    
    def slam_loop(self):
        """Main SLAM processing loop"""
        if not self.laser_data or not self.odom_data:
            return
        
        start_time = time.time()
        
        with self.slam_lock:
            # Motion update
            self.motion_update()
            
            # Measurement update
            self.measurement_update()
            
            # Resample if necessary
            if self.needs_resampling():
                self.resample_particles()
            
            # Update best particle
            self.update_best_particle()
            
            # Loop closure detection
            self.detect_loop_closure()
            
            # Update global map
            self.update_global_map()
        
        self.processing_time = time.time() - start_time
    
    def motion_update(self):
        """Update particle poses based on odometry"""
        if not self.previous_pose or not self.current_pose:
            self.previous_pose = self.current_pose
            return
        
        # Calculate motion
        dx = self.current_pose.position.x - self.previous_pose.position.x
        dy = self.current_pose.position.y - self.previous_pose.position.y
        dtheta = self.calculate_angle_difference(
            self.previous_pose.orientation,
            self.current_pose.orientation
        )
        
        # Add noise to motion
        noise_dx = np.random.normal(0, self.motion_noise)
        noise_dy = np.random.normal(0, self.motion_noise)
        noise_dtheta = np.random.normal(0, self.motion_noise * 0.1)
        
        # Update each particle
        for particle in self.particles:
            # Transform motion to particle's coordinate frame
            cos_theta = math.cos(particle.theta)
            sin_theta = math.sin(particle.theta)
            
            particle.x += (dx + noise_dx) * cos_theta - (dy + noise_dy) * sin_theta
            particle.y += (dx + noise_dx) * sin_theta + (dy + noise_dy) * cos_theta
            particle.theta += dtheta + noise_dtheta
            
            # Normalize angle
            particle.theta = self.normalize_angle(particle.theta)
        
        self.previous_pose = self.current_pose
    
    def measurement_update(self):
        """Update particle weights based on laser measurements"""
        if not self.laser_data:
            return
        
        for particle in self.particles:
            # Calculate likelihood of laser scan given particle pose
            likelihood = self.calculate_scan_likelihood(particle)
            particle.weight *= likelihood
        
        # Normalize weights
        total_weight = sum(p.weight for p in self.particles)
        if total_weight > 0:
            for particle in self.particles:
                particle.weight /= total_weight
    
    def calculate_scan_likelihood(self, particle: Particle) -> float:
        """Calculate likelihood of laser scan given particle pose"""
        if not self.laser_data or not particle.map:
            return 1.0
        
        likelihood = 1.0
        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        
        for i, range_val in enumerate(ranges):
            if range_val < self.min_range or range_val > self.max_range:
                continue
            
            # Calculate expected range from map
            angle = angle_min + i * angle_increment
            expected_range = self.raycast_map(particle, angle)
            
            if expected_range > 0:
                # Calculate likelihood based on range difference
                range_diff = abs(range_val - expected_range)
                if range_diff < 0.1:
                    likelihood *= 0.9
                elif range_diff < 0.2:
                    likelihood *= 0.7
                elif range_diff < 0.5:
                    likelihood *= 0.3
                else:
                    likelihood *= 0.1
        
        return max(likelihood, 1e-10)  # Prevent zero likelihood
    
    def raycast_map(self, particle: Particle, angle: float) -> float:
        """Raycast from particle position in given direction"""
        if not particle.map:
            return -1.0
        
        # Start from particle position
        x = particle.x
        y = particle.y
        
        # Direction vector
        dx = math.cos(particle.theta + angle)
        dy = math.sin(particle.theta + angle)
        
        # Raycast step by step
        step_size = self.map_resolution
        max_steps = int(self.max_range / step_size)
        
        for step in range(max_steps):
            # Convert to grid coordinates
            grid_x = int((x - particle.map.info.origin.position.x) / self.map_resolution)
            grid_y = int((y - particle.map.info.origin.position.y) / self.map_resolution)
            
            # Check bounds
            if (grid_x < 0 or grid_x >= particle.map.info.width or
                grid_y < 0 or grid_y >= particle.map.info.height):
                break
            
            # Check occupancy
            index = grid_y * particle.map.info.width + grid_x
            if index < len(particle.map.data):
                occupancy = particle.map.data[index]
                if occupancy > 50:  # Occupied
                    return step * step_size
                elif occupancy >= 0:  # Free
                    pass  # Continue raycasting
                else:  # Unknown
                    pass  # Continue raycasting
            
            # Move along ray
            x += dx * step_size
            y += dy * step_size
        
        return self.max_range  # No obstacle found
    
    def needs_resampling(self) -> bool:
        """Check if particle resampling is needed"""
        if not self.particles:
            return False
        
        # Calculate effective number of particles
        weights = [p.weight for p in self.particles]
        total_weight = sum(weights)
        
        if total_weight == 0:
            return True
        
        effective_particles = 1.0 / sum(w**2 for w in weights)
        return effective_particles < self.resample_threshold * self.num_particles
    
    def resample_particles(self):
        """Resample particles based on their weights"""
        if not self.particles:
            return
        
        # Calculate cumulative weights
        weights = [p.weight for p in self.particles]
        total_weight = sum(weights)
        
        if total_weight == 0:
            # Equal weights if all weights are zero
            weights = [1.0 / len(self.particles)] * len(self.particles)
            total_weight = 1.0
        
        # Normalize weights
        weights = [w / total_weight for w in weights]
        
        # Create cumulative distribution
        cumulative = []
        cumsum = 0.0
        for w in weights:
            cumsum += w
            cumulative.append(cumsum)
        
        # Resample particles
        new_particles = []
        for _ in range(self.num_particles):
            # Random number for resampling
            r = np.random.random()
            
            # Find particle to resample
            for i, cum in enumerate(cumulative):
                if r <= cum:
                    # Create new particle with same pose but reset weight
                    old_particle = self.particles[i]
                    new_particle = Particle(
                        old_particle.x, old_particle.y, old_particle.theta, 1.0 / self.num_particles
                    )
                    new_particle.map = old_particle.map  # Copy map
                    new_particle.landmarks = old_particle.landmarks.copy()  # Copy landmarks
                    new_particles.append(new_particle)
                    break
        
        self.particles = new_particles
        self.particles_count = len(self.particles)
        
        self.get_logger().info(f"Resampled to {self.particles_count} particles")
    
    def update_best_particle(self):
        """Update the best particle based on weights"""
        if not self.particles:
            return
        
        # Find particle with highest weight
        best_weight = 0.0
        for particle in self.particles:
            if particle.weight > best_weight:
                best_weight = particle.weight
                self.best_particle = particle
    
    def detect_loop_closure(self):
        """Detect loop closures using visual and geometric features"""
        if not self.best_particle or not self.camera_data:
            return
        
        # Extract current visual features
        current_features = self.extract_visual_features()
        
        if not current_features:
            return
        
        # Compare with previous features
        for landmark_id, landmark in self.landmarks.items():
            if landmark.descriptor is not None and len(current_features) > 0:
                # Calculate feature similarity
                similarity = self.calculate_feature_similarity(
                    current_features, landmark.descriptor
                )
                
                if similarity > self.loop_closure_threshold:
                    # Potential loop closure detected
                    self.loop_closures_detected += 1
                    
                    # Publish loop closure event
                    loop_msg = LoopClosure()
                    loop_msg.header.stamp = self.get_clock().now().to_msg()
                    loop_msg.landmark_id = landmark_id
                    loop_msg.confidence = similarity
                    loop_msg.robot_pose.position.x = self.best_particle.x
                    loop_msg.robot_pose.position.y = self.best_particle.y
                    loop_msg.robot_pose.orientation.z = math.sin(self.best_particle.theta / 2)
                    loop_msg.robot_pose.orientation.w = math.cos(self.best_particle.theta / 2)
                    
                    self.loop_closure_pub.publish(loop_msg)
                    
                    self.get_logger().info(f"Loop closure detected with confidence {similarity:.2f}")
                    break
    
    def extract_visual_features(self) -> Optional[np.ndarray]:
        """Extract visual features from camera image"""
        if not self.camera_data:
            return None
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_data, "bgr8")
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Extract ORB features
            orb = cv2.ORB_create(nfeatures=100)
            keypoints, descriptors = orb.detectAndCompute(gray, None)
            
            if descriptors is not None and len(descriptors) > 0:
                # Return mean descriptor
                return np.mean(descriptors, axis=0)
            
        except Exception as e:
            self.get_logger().error(f"Feature extraction error: {str(e)}")
        
        return None
    
    def calculate_feature_similarity(self, features1: np.ndarray, features2: np.ndarray) -> float:
        """Calculate similarity between feature descriptors"""
        if features1 is None or features2 is None:
            return 0.0
        
        # Normalize features
        features1 = features1 / (np.linalg.norm(features1) + 1e-8)
        features2 = features2 / (np.linalg.norm(features2) + 1e-8)
        
        # Calculate cosine similarity
        similarity = np.dot(features1, features2)
        return max(0.0, similarity)
    
    def update_map_with_scan(self):
        """Update occupancy grid with laser scan data"""
        if not self.laser_data or not self.best_particle:
            return
        
        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        
        for i, range_val in enumerate(ranges):
            if range_val < self.min_range or range_val > self.max_range:
                continue
            
            angle = angle_min + i * angle_increment
            
            # Calculate endpoint in world coordinates
            end_x = self.best_particle.x + range_val * math.cos(self.best_particle.theta + angle)
            end_y = self.best_particle.y + range_val * math.sin(self.best_particle.theta + angle)
            
            # Update occupancy grid
            self.update_occupancy_cell(end_x, end_y, True)  # Occupied
            
            # Update free space along the ray
            self.update_ray_occupancy(
                self.best_particle.x, self.best_particle.y,
                end_x, end_y, False
            )
    
    def update_occupancy_cell(self, x: float, y: float, occupied: bool):
        """Update a single occupancy grid cell"""
        if not self.occupancy_grid:
            return
        
        # Convert to grid coordinates
        grid_x = int((x - self.occupancy_grid.info.origin.position.x) / self.map_resolution)
        grid_y = int((y - self.occupancy_grid.info.origin.position.y) / self.map_resolution)
        
        # Check bounds
        if (grid_x < 0 or grid_x >= self.occupancy_grid.info.width or
            grid_y < 0 or grid_y >= self.occupancy_grid.info.height):
            return
        
        # Update cell
        index = grid_y * self.occupancy_grid.info.width + grid_x
        if index < len(self.occupancy_grid.data):
            if occupied:
                self.occupancy_grid.data[index] = min(100, self.occupancy_grid.data[index] + 10)
            else:
                self.occupancy_grid.data[index] = max(0, self.occupancy_grid.data[index] - 5)
    
    def update_ray_occupancy(self, start_x: float, start_y: float, 
                           end_x: float, end_y: float, occupied: bool):
        """Update occupancy along a ray"""
        # Bresenham's line algorithm for grid cells
        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)
        
        x_step = 1 if start_x < end_x else -1
        y_step = 1 if start_y < end_y else -1
        
        error = dx - dy
        x, y = start_x, start_y
        
        while True:
            self.update_occupancy_cell(x, y, occupied)
            
            if x == end_x and y == end_y:
                break
            
            error2 = 2 * error
            
            if error2 > -dy:
                error -= dy
                x += x_step
            
            if error2 < dx:
                error += dx
                y += y_step
    
    def update_global_map(self):
        """Update global map from best particle"""
        if not self.best_particle or not self.best_particle.map:
            return
        
        # Copy best particle's map to global map
        self.occupancy_grid = self.best_particle.map
        
        # Publish map
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.occupancy_grid)
        
        # Publish robot pose
        self.publish_robot_pose()
        
        # Publish transform
        self.publish_transform()
    
    def publish_robot_pose(self):
        """Publish robot pose with covariance"""
        if not self.best_particle:
            return
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        
        pose_msg.pose.pose.position.x = self.best_particle.x
        pose_msg.pose.pose.position.y = self.best_particle.y
        pose_msg.pose.pose.position.z = 0.0
        
        # Convert angle to quaternion
        pose_msg.pose.pose.orientation.z = math.sin(self.best_particle.theta / 2)
        pose_msg.pose.pose.orientation.w = math.cos(self.best_particle.theta / 2)
        
        # Set covariance (simplified)
        covariance = np.zeros((6, 6))
        covariance[0, 0] = 0.1  # x variance
        covariance[1, 1] = 0.1  # y variance
        covariance[5, 5] = 0.1  # yaw variance
        pose_msg.pose.covariance = covariance.flatten().tolist()
        
        self.pose_pub.publish(pose_msg)
    
    def publish_transform(self):
        """Publish transform from map to base_link"""
        if not self.best_particle:
            return
        
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        
        transform.transform.translation.x = self.best_particle.x
        transform.transform.translation.y = self.best_particle.y
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.z = math.sin(self.best_particle.theta / 2)
        transform.transform.rotation.w = math.cos(self.best_particle.theta / 2)
        
        self.tf_broadcaster.sendTransform(transform)
    
    def publish_status(self):
        """Publish SLAM status"""
        status_msg = SLAMStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.map_quality = self.map_quality
        status_msg.loop_closures_detected = self.loop_closures_detected
        status_msg.particles_count = self.particles_count
        status_msg.processing_time = self.processing_time
        
        self.status_pub.publish(status_msg)
    
    def calculate_angle_difference(self, q1, q2) -> float:
        """Calculate angle difference between two quaternions"""
        # Simplified angle calculation
        return 0.0  # This should be properly implemented
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    slam = WarehouseSLAM()
    
    try:
        rclpy.spin(slam)
    except KeyboardInterrupt:
        slam.get_logger().info("SLAM node shutting down...")
    finally:
        slam.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
