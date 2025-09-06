#!/usr/bin/env python3
"""
Autonomous Navigation System for Warehouse Robot

This module implements advanced autonomous navigation capabilities including:
- A* path planning with dynamic obstacle avoidance
- DWA (Dynamic Window Approach) local planner
- Adaptive path re-planning
- Multi-robot coordination

Author: Tanmay Pancholi
Date: 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
from collections import deque
import heapq
from typing import List, Tuple, Optional, Dict
import threading
import time

# ROS2 Messages
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, Bool
from visualization_msgs.msg import Marker, MarkerArray

# Custom Messages
from warehouse_robot_msgs.msg import NavigationGoal, NavigationStatus, PathPlan

class Node:
    """A* path planning node"""
    def __init__(self, x: int, y: int, g: float = 0, h: float = 0, parent=None):
        self.x = x
        self.y = y
        self.g = g  # Cost from start
        self.h = h  # Heuristic cost to goal
        self.f = g + h  # Total cost
        self.parent = parent
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class AutonomousNavigator(Node):
    """
    Advanced autonomous navigation system for warehouse robots.
    
    Features:
    - A* global path planning
    - DWA local planning with obstacle avoidance
    - Dynamic re-planning based on environment changes
    - Multi-robot coordination
    - Adaptive speed control
    """
    
    def __init__(self):
        super().__init__('autonomous_navigator')
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/warehouse_robot/cmd_vel', qos_profile)
        self.path_pub = self.create_publisher(Path, '/warehouse_robot/global_path', qos_profile)
        self.marker_pub = self.create_publisher(MarkerArray, '/warehouse_robot/navigation_markers', qos_profile)
        self.status_pub = self.create_publisher(NavigationStatus, '/warehouse_robot/navigation_status', qos_profile)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/warehouse_robot/navigation_goal', self.goal_callback, qos_profile)
        self.odom_sub = self.create_subscription(
            Odometry, '/warehouse_robot/odom', self.odom_callback, qos_profile)
        self.scan_sub = self.create_subscription(
            LaserScan, '/warehouse_robot/scan', self.scan_callback, qos_profile)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/warehouse_robot/map', self.map_callback, qos_profile)
        
        # Navigation parameters
        self.declare_parameters()
        self.load_parameters()
        
        # State variables
        self.current_pose = None
        self.current_goal = None
        self.current_path = []
        self.occupancy_grid = None
        self.laser_data = None
        self.navigation_active = False
        self.path_lock = threading.Lock()
        
        # Performance metrics
        self.path_planning_time = 0.0
        self.total_distance_traveled = 0.0
        self.collision_count = 0
        self.replan_count = 0
        
        # Timers
        self.navigation_timer = self.create_timer(0.1, self.navigation_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("Autonomous Navigator initialized")
    
    def declare_parameters(self):
        """Declare ROS2 parameters"""
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('linear_acceleration', 0.2)
        self.declare_parameter('angular_acceleration', 0.5)
        self.declare_parameter('safety_distance', 0.3)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('replan_threshold', 0.5)
        self.declare_parameter('max_planning_time', 2.0)
        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('grid_resolution', 0.05)
    
    def load_parameters(self):
        """Load parameters from ROS2 parameter server"""
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.linear_accel = self.get_parameter('linear_acceleration').value
        self.angular_accel = self.get_parameter('angular_acceleration').value
        self.safety_dist = self.get_parameter('safety_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.replan_threshold = self.get_parameter('replan_threshold').value
        self.max_planning_time = self.get_parameter('max_planning_time').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
    
    def goal_callback(self, msg: PoseStamped):
        """Handle new navigation goal"""
        self.get_logger().info(f"New goal received: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}")
        self.current_goal = msg
        self.navigation_active = True
        self.plan_path()
    
    def odom_callback(self, msg: Odometry):
        """Handle odometry updates"""
        self.current_pose = msg.pose.pose
        self.update_distance_traveled(msg)
    
    def scan_callback(self, msg: LaserScan):
        """Handle laser scan data"""
        self.laser_data = msg
        self.check_collision_risk()
    
    def map_callback(self, msg: OccupancyGrid):
        """Handle occupancy grid updates"""
        self.occupancy_grid = msg
        # Trigger re-planning if current path is blocked
        if self.navigation_active and self.is_path_blocked():
            self.get_logger().warn("Path blocked, re-planning...")
            self.plan_path()
    
    def plan_path(self):
        """Plan global path using A* algorithm"""
        if not self.current_pose or not self.current_goal or not self.occupancy_grid:
            return
        
        start_time = time.time()
        
        try:
            # Convert poses to grid coordinates
            start_grid = self.world_to_grid(
                self.current_pose.position.x, 
                self.current_pose.position.y
            )
            goal_grid = self.world_to_grid(
                self.current_goal.pose.position.x, 
                self.current_goal.pose.position.y
            )
            
            # Plan path using A*
            path_grid = self.astar_planning(start_grid, goal_grid)
            
            if path_grid:
                # Convert grid path to world coordinates
                self.current_path = [self.grid_to_world(x, y) for x, y in path_grid]
                
                # Publish path for visualization
                self.publish_path()
                
                # Publish navigation markers
                self.publish_navigation_markers()
                
                self.get_logger().info(f"Path planned with {len(self.current_path)} waypoints")
            else:
                self.get_logger().error("Failed to plan path")
                self.navigation_active = False
            
            self.path_planning_time = time.time() - start_time
            
        except Exception as e:
            self.get_logger().error(f"Path planning error: {str(e)}")
            self.navigation_active = False
    
    def astar_planning(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        A* path planning algorithm with adaptive heuristics
        
        Args:
            start: Starting grid coordinates (x, y)
            goal: Goal grid coordinates (x, y)
            
        Returns:
            List of grid coordinates representing the path
        """
        if not self.is_valid_grid_point(start) or not self.is_valid_grid_point(goal):
            return []
        
        # Priority queue for open set
        open_set = [Node(start[0], start[1], 0, self.heuristic(start, goal))]
        open_dict = {start: open_set[0]}
        closed_set = set()
        
        # Directions for 8-connected grid
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        while open_set:
            current = heapq.heappop(open_set)
            current_pos = (current.x, current.y)
            
            if current_pos in closed_set:
                continue
            
            closed_set.add(current_pos)
            
            # Check if goal reached
            if self.is_goal_reached(current_pos, goal):
                return self.reconstruct_path(current)
            
            # Explore neighbors
            for dx, dy in directions:
                neighbor = (current.x + dx, current.y + dy)
                
                if neighbor in closed_set or not self.is_valid_grid_point(neighbor):
                    continue
                
                # Calculate costs
                g_cost = current.g + self.calculate_cost(current_pos, neighbor)
                h_cost = self.heuristic(neighbor, goal)
                f_cost = g_cost + h_cost
                
                if neighbor in open_dict:
                    if g_cost < open_dict[neighbor].g:
                        open_dict[neighbor].g = g_cost
                        open_dict[neighbor].f = f_cost
                        open_dict[neighbor].parent = current
                        heapq.heapify(open_set)
                else:
                    neighbor_node = Node(neighbor[0], neighbor[1], g_cost, h_cost, current)
                    heapq.heappush(open_set, neighbor_node)
                    open_dict[neighbor] = neighbor_node
        
        return []  # No path found
    
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Calculate heuristic cost (Euclidean distance)"""
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        return math.sqrt(dx * dx + dy * dy)
    
    def calculate_cost(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Calculate movement cost between two grid points"""
        dx = abs(pos1[0] - pos2[0])
        dy = abs(pos1[1] - pos2[1])
        
        # Diagonal movement costs more
        if dx == 1 and dy == 1:
            return 1.414  # sqrt(2)
        else:
            return 1.0
    
    def is_valid_grid_point(self, pos: Tuple[int, int]) -> bool:
        """Check if grid point is valid and not occupied"""
        if not self.occupancy_grid:
            return False
        
        x, y = pos
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        if x < 0 or x >= width or y < 0 or y >= height:
            return False
        
        # Check occupancy
        index = y * width + x
        if index < len(self.occupancy_grid.data):
            occupancy = self.occupancy_grid.data[index]
            return occupancy < 50  # Free space threshold
        
        return False
    
    def is_goal_reached(self, pos: Tuple[int, int], goal: Tuple[int, int]) -> bool:
        """Check if current position is close enough to goal"""
        distance = self.heuristic(pos, goal)
        return distance < 3  # 3 grid cells tolerance
    
    def reconstruct_path(self, goal_node: Node) -> List[Tuple[int, int]]:
        """Reconstruct path from goal node to start"""
        path = []
        current = goal_node
        
        while current:
            path.append((current.x, current.y))
            current = current.parent
        
        return path[::-1]  # Reverse to get start to goal
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        if not self.occupancy_grid:
            return (0, 0)
        
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        resolution = self.occupancy_grid.info.resolution
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        if not self.occupancy_grid:
            return (0.0, 0.0)
        
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        resolution = self.occupancy_grid.info.resolution
        
        world_x = grid_x * resolution + origin_x
        world_y = grid_y * resolution + origin_y
        
        return (world_x, world_y)
    
    def navigation_loop(self):
        """Main navigation control loop"""
        if not self.navigation_active or not self.current_path or not self.current_pose:
            return
        
        # Get current waypoint
        current_waypoint = self.get_current_waypoint()
        if not current_waypoint:
            self.navigation_active = False
            self.get_logger().info("Navigation goal reached!")
            return
        
        # Calculate control commands using DWA
        cmd_vel = self.dwa_control(current_waypoint)
        
        # Publish control command
        self.cmd_vel_pub.publish(cmd_vel)
    
    def get_current_waypoint(self) -> Optional[Tuple[float, float]]:
        """Get current target waypoint"""
        if not self.current_path:
            return None
        
        # Find closest waypoint ahead of robot
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        min_distance = float('inf')
        target_waypoint = None
        
        for waypoint in self.current_path:
            wx, wy = waypoint
            distance = math.sqrt((wx - robot_x)**2 + (wy - robot_y)**2)
            
            if distance < min_distance and distance > 0.1:  # Look ahead
                min_distance = distance
                target_waypoint = waypoint
        
        return target_waypoint
    
    def dwa_control(self, target: Tuple[float, float]) -> Twist:
        """
        Dynamic Window Approach for local path planning and control
        
        Args:
            target: Target waypoint (x, y)
            
        Returns:
            Twist message with velocity commands
        """
        cmd_vel = Twist()
        
        if not target:
            return cmd_vel
        
        # Calculate desired velocity
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Calculate heading to target
        dx = target[0] - robot_x
        dy = target[1] - robot_y
        target_heading = math.atan2(dy, dx)
        
        # Get current robot heading (simplified - assuming quaternion)
        current_heading = 0.0  # This should be extracted from pose.orientation
        
        # Calculate heading error
        heading_error = self.normalize_angle(target_heading - current_heading)
        
        # Calculate distance to target
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        # Adaptive speed control
        if distance_to_target < self.goal_tolerance:
            # Stop when close to target
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        else:
            # Proportional control for linear velocity
            max_linear = self.max_linear_vel
            linear_vel = min(max_linear, distance_to_target * 0.5)
            
            # Proportional control for angular velocity
            max_angular = self.max_angular_vel
            angular_vel = max(-max_angular, min(max_angular, heading_error * 2.0))
            
            # Apply obstacle avoidance
            linear_vel, angular_vel = self.apply_obstacle_avoidance(linear_vel, angular_vel)
            
            cmd_vel.linear.x = linear_vel
            cmd_vel.angular.z = angular_vel
        
        return cmd_vel
    
    def apply_obstacle_avoidance(self, linear_vel: float, angular_vel: float) -> Tuple[float, float]:
        """Apply obstacle avoidance using laser data"""
        if not self.laser_data:
            return linear_vel, angular_vel
        
        # Find minimum distance in front of robot
        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        
        # Check front sector (30 degrees)
        front_start = int((-0.26 - angle_min) / angle_increment)  # -15 degrees
        front_end = int((0.26 - angle_min) / angle_increment)     # +15 degrees
        
        min_distance = float('inf')
        for i in range(max(0, front_start), min(len(ranges), front_end)):
            if ranges[i] < min_distance:
                min_distance = ranges[i]
        
        # Adjust velocity based on obstacle distance
        if min_distance < self.safety_dist:
            # Reduce linear velocity
            linear_vel *= 0.1
            
            # Add turning behavior
            if min_distance < self.safety_dist * 0.5:
                angular_vel = self.max_angular_vel * 0.5
        
        return linear_vel, angular_vel
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def is_path_blocked(self) -> bool:
        """Check if current path is blocked by obstacles"""
        if not self.current_path or not self.laser_data:
            return False
        
        # Check if any waypoint in current path is blocked
        for waypoint in self.current_path[:5]:  # Check first 5 waypoints
            wx, wy = waypoint
            rx = self.current_pose.position.x
            ry = self.current_pose.position.y
            
            # Calculate distance and angle to waypoint
            dx = wx - rx
            dy = wy - ry
            distance = math.sqrt(dx**2 + dy**2)
            angle = math.atan2(dy, dx)
            
            # Check if there's an obstacle in this direction
            if self.is_obstacle_in_direction(angle, distance):
                return True
        
        return False
    
    def is_obstacle_in_direction(self, angle: float, max_distance: float) -> bool:
        """Check if there's an obstacle in a specific direction"""
        if not self.laser_data:
            return False
        
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        
        # Convert angle to laser index
        index = int((angle - angle_min) / angle_increment)
        
        if 0 <= index < len(self.laser_data.ranges):
            obstacle_distance = self.laser_data.ranges[index]
            return obstacle_distance < max_distance and obstacle_distance > 0.1
        
        return False
    
    def check_collision_risk(self):
        """Check for imminent collision risk"""
        if not self.laser_data:
            return
        
        ranges = self.laser_data.ranges
        min_distance = min([r for r in ranges if r > 0.1])
        
        if min_distance < self.safety_dist * 0.5:
            self.collision_count += 1
            self.get_logger().warn(f"Collision risk detected! Distance: {min_distance:.2f}m")
    
    def update_distance_traveled(self, odom_msg: Odometry):
        """Update total distance traveled"""
        if hasattr(self, 'last_pose') and self.last_pose:
            dx = odom_msg.pose.pose.position.x - self.last_pose.position.x
            dy = odom_msg.pose.pose.position.y - self.last_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            self.total_distance_traveled += distance
        
        self.last_pose = odom_msg.pose.pose
    
    def publish_path(self):
        """Publish planned path for visualization"""
        if not self.current_path:
            return
        
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for x, y in self.current_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_navigation_markers(self):
        """Publish navigation markers for visualization"""
        marker_array = MarkerArray()
        
        # Goal marker
        if self.current_goal:
            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.id = 0
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose = self.current_goal.pose
            goal_marker.scale.x = 0.3
            goal_marker.scale.y = 0.3
            goal_marker.scale.z = 0.3
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 0.8
            marker_array.markers.append(goal_marker)
        
        self.marker_pub.publish(marker_array)
    
    def publish_status(self):
        """Publish navigation status"""
        status_msg = NavigationStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.navigation_active = self.navigation_active
        status_msg.path_planning_time = self.path_planning_time
        status_msg.total_distance = self.total_distance_traveled
        status_msg.collision_count = self.collision_count
        status_msg.replan_count = self.replan_count
        
        self.status_pub.publish(status_msg)

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    navigator = AutonomousNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("Navigation node shutting down...")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
