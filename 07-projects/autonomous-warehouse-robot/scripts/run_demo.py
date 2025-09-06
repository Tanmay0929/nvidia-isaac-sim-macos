#!/usr/bin/env python3
"""
Warehouse Robot Demo Runner

This script runs various demonstration scenarios for the autonomous warehouse robot.
It showcases different capabilities including navigation, manipulation, and coordination.


Author: Tanmay Pancholi
Date: 2024
"""

import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
import time
import threading
from typing import List, Dict, Optional

# ROS2 Messages
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from warehouse_robot_msgs.msg import ManipulationGoal, NavigationGoal

class DemoRunner(Node):
    """Demo runner for warehouse robot scenarios"""
    
    def __init__(self):
        super().__init__('demo_runner')
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Publishers
        self.nav_goal_pub = self.create_publisher(
            PoseStamped, '/warehouse_robot/navigation_goal', qos_profile)
        self.manip_goal_pub = self.create_publisher(
            ManipulationGoal, '/warehouse_robot/manipulation_goal', qos_profile)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/warehouse_robot/cmd_vel', qos_profile)
        
        # Demo scenarios
        self.scenarios = {
            'navigation': self.run_navigation_demo,
            'warehouse': self.run_warehouse_demo,
            'multi_robot': self.run_multi_robot_demo,
            'manipulation': self.run_manipulation_demo,
            'full_system': self.run_full_system_demo
        }
        
        self.get_logger().info("Demo Runner initialized")
    
    def run_navigation_demo(self):
        """Run basic navigation demonstration"""
        self.get_logger().info("Starting Navigation Demo...")
        
        # Define waypoints for navigation demo
        waypoints = [
            (0.0, 0.0, 0.0),      # Start
            (2.0, 0.0, 0.0),      # Forward
            (2.0, 2.0, 0.0),      # Right
            (0.0, 2.0, 0.0),      # Back
            (0.0, 0.0, 0.0),      # Return to start
        ]
        
        for i, (x, y, theta) in enumerate(waypoints):
            self.get_logger().info(f"Navigating to waypoint {i+1}: ({x}, {y}, {theta})")
            
            # Create navigation goal
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            
            # Set orientation
            goal.pose.orientation.z = np.sin(theta / 2)
            goal.pose.orientation.w = np.cos(theta / 2)
            
            # Publish goal
            self.nav_goal_pub.publish(goal)
            
            # Wait for navigation to complete
            time.sleep(5)
        
        self.get_logger().info("Navigation Demo completed!")
    
    def run_warehouse_demo(self):
        """Run warehouse operations demonstration"""
        self.get_logger().info("Starting Warehouse Operations Demo...")
        
        # Define warehouse tasks
        tasks = [
            {
                'type': 'pick',
                'location': (1.0, 1.0, 0.0),
                'object': 'package_small'
            },
            {
                'type': 'transport',
                'destination': (3.0, 3.0, 0.0)
            },
            {
                'type': 'place',
                'location': (3.0, 3.0, 0.0)
            },
            {
                'type': 'return',
                'location': (0.0, 0.0, 0.0)
            }
        ]
        
        for i, task in enumerate(tasks):
            self.get_logger().info(f"Executing task {i+1}: {task['type']}")
            
            if task['type'] == 'pick':
                # Navigate to pick location
                self.navigate_to_location(task['location'])
                
                # Execute pick operation
                self.execute_pick_operation(task['object'])
                
            elif task['type'] == 'transport':
                # Navigate to destination
                self.navigate_to_location(task['destination'])
                
            elif task['type'] == 'place':
                # Execute place operation
                self.execute_place_operation(task['location'])
                
            elif task['type'] == 'return':
                # Return to start position
                self.navigate_to_location(task['location'])
            
            time.sleep(3)
        
        self.get_logger().info("Warehouse Operations Demo completed!")
    
    def run_multi_robot_demo(self):
        """Run multi-robot coordination demonstration"""
        self.get_logger().info("Starting Multi-Robot Demo...")
        
        # Define coordinated tasks for multiple robots
        robot_tasks = {
            'robot_1': [
                {'type': 'navigate', 'location': (2.0, 0.0, 0.0)},
                {'type': 'pick', 'object': 'package_medium'},
                {'type': 'navigate', 'location': (4.0, 2.0, 0.0)},
                {'type': 'place'}
            ],
            'robot_2': [
                {'type': 'navigate', 'location': (0.0, 2.0, 0.0)},
                {'type': 'pick', 'object': 'package_small'},
                {'type': 'navigate', 'location': (2.0, 4.0, 0.0)},
                {'type': 'place'}
            ]
        }
        
        # Execute tasks in parallel (simplified)
        for robot_id, tasks in robot_tasks.items():
            self.get_logger().info(f"Executing tasks for {robot_id}")
            
            for task in tasks:
                if task['type'] == 'navigate':
                    self.navigate_to_location(task['location'])
                elif task['type'] == 'pick':
                    self.execute_pick_operation(task['object'])
                elif task['type'] == 'place':
                    self.execute_place_operation()
                
                time.sleep(2)
        
        self.get_logger().info("Multi-Robot Demo completed!")
    
    def run_manipulation_demo(self):
        """Run manipulation demonstration"""
        self.get_logger().info("Starting Manipulation Demo...")
        
        # Define manipulation tasks
        manipulation_tasks = [
            {
                'operation': 'move_to_pose',
                'pose': {
                    'position': {'x': 0.5, 'y': 0.0, 'z': 0.3},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            },
            {
                'operation': 'pick',
                'target_pose': {
                    'position': {'x': 0.3, 'y': 0.0, 'z': 0.1},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            },
            {
                'operation': 'place',
                'target_pose': {
                    'position': {'x': 0.7, 'y': 0.0, 'z': 0.1},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            }
        ]
        
        for i, task in enumerate(manipulation_tasks):
            self.get_logger().info(f"Executing manipulation task {i+1}: {task['operation']}")
            
            # Create manipulation goal
            goal = ManipulationGoal()
            goal.operation = task['operation']
            goal.target_pose.position.x = task['target_pose']['position']['x']
            goal.target_pose.position.y = task['target_pose']['position']['y']
            goal.target_pose.position.z = task['target_pose']['position']['z']
            goal.target_pose.orientation.x = task['target_pose']['orientation']['x']
            goal.target_pose.orientation.y = task['target_pose']['orientation']['y']
            goal.target_pose.orientation.z = task['target_pose']['orientation']['z']
            goal.target_pose.orientation.w = task['target_pose']['orientation']['w']
            
            # Publish goal
            self.manip_goal_pub.publish(goal)
            
            # Wait for completion
            time.sleep(5)
        
        self.get_logger().info("Manipulation Demo completed!")
    
    def run_full_system_demo(self):
        """Run complete system demonstration"""
        self.get_logger().info("Starting Full System Demo...")
        
        # Phase 1: SLAM and Mapping
        self.get_logger().info("Phase 1: SLAM and Mapping")
        self.explore_environment()
        
        # Phase 2: Navigation
        self.get_logger().info("Phase 2: Navigation")
        self.run_navigation_demo()
        
        # Phase 3: Manipulation
        self.get_logger().info("Phase 3: Manipulation")
        self.run_manipulation_demo()
        
        # Phase 4: Warehouse Operations
        self.get_logger().info("Phase 4: Warehouse Operations")
        self.run_warehouse_demo()
        
        self.get_logger().info("Full System Demo completed!")
    
    def navigate_to_location(self, location: tuple):
        """Navigate to a specific location"""
        x, y, theta = location
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = np.sin(theta / 2)
        goal.pose.orientation.w = np.cos(theta / 2)
        
        self.nav_goal_pub.publish(goal)
        time.sleep(3)
    
    def execute_pick_operation(self, object_type: str):
        """Execute pick operation"""
        goal = ManipulationGoal()
        goal.operation = "pick"
        goal.target_pose.position.x = 0.3
        goal.target_pose.position.y = 0.0
        goal.target_pose.position.z = 0.1
        goal.target_pose.orientation.w = 1.0
        
        self.manip_goal_pub.publish(goal)
        time.sleep(3)
    
    def execute_place_operation(self, location: tuple = None):
        """Execute place operation"""
        goal = ManipulationGoal()
        goal.operation = "place"
        
        if location:
            x, y, z = location
            goal.target_pose.position.x = x
            goal.target_pose.position.y = y
            goal.target_pose.position.z = z
        else:
            goal.target_pose.position.x = 0.7
            goal.target_pose.position.y = 0.0
            goal.target_pose.position.z = 0.1
        
        goal.target_pose.orientation.w = 1.0
        
        self.manip_goal_pub.publish(goal)
        time.sleep(3)
    
    def explore_environment(self):
        """Explore environment for SLAM"""
        self.get_logger().info("Exploring environment for mapping...")
        
        # Define exploration waypoints
        exploration_points = [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (2.0, 2.0, 0.0),
            (0.0, 2.0, 0.0),
            (0.0, 0.0, 0.0)
        ]
        
        for point in exploration_points:
            self.navigate_to_location(point)
            time.sleep(2)
        
        self.get_logger().info("Environment exploration completed!")

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Run warehouse robot demos')
    parser.add_argument('--scenario', 
                       choices=['navigation', 'warehouse', 'multi_robot', 'manipulation', 'full_system'],
                       default='navigation',
                       help='Demo scenario to run')
    parser.add_argument('--duration', type=int, default=60,
                       help='Demo duration in seconds')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    demo_runner = DemoRunner()
    
    try:
        # Run selected scenario
        if args.scenario in demo_runner.scenarios:
            demo_runner.get_logger().info(f"Running {args.scenario} scenario...")
            demo_runner.scenarios[args.scenario]()
        else:
            demo_runner.get_logger().error(f"Unknown scenario: {args.scenario}")
        
        # Keep node alive for specified duration
        demo_runner.get_logger().info(f"Demo completed. Keeping node alive for {args.duration} seconds...")
        time.sleep(args.duration)
        
    except KeyboardInterrupt:
        demo_runner.get_logger().info("Demo interrupted by user")
    finally:
        demo_runner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
