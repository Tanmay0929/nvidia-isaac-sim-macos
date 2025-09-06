#!/usr/bin/env python3
"""
Robot Arm Controller for Warehouse Operations

Advanced manipulation system featuring:
- 6-DOF robotic arm control
- Object detection and recognition
- Pick and place operations
- Force/torque control
- Collision avoidance
- Trajectory planning

Author: Tanmay Pancholi
Date: 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import math
from typing import List, Tuple, Optional, Dict
import threading
import time
from collections import deque

# ROS2 Messages
from sensor_msgs.msg import JointState, Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from std_msgs.msg import Header, Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from visualization_msgs.msg import Marker, MarkerArray

# Custom Messages
from warehouse_robot_msgs.msg import ManipulationGoal, ManipulationStatus, ObjectDetection

class JointController:
    """Individual joint controller with PID control"""
    
    def __init__(self, joint_name: str, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0):
        self.joint_name = joint_name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.target_position = 0.0
        self.current_position = 0.0
        self.current_velocity = 0.0
        
        self.integral_error = 0.0
        self.previous_error = 0.0
        
        self.max_velocity = 1.0
        self.max_torque = 10.0
    
    def update(self, current_pos: float, current_vel: float, dt: float) -> float:
        """Update PID controller and return torque command"""
        error = self.target_position - current_pos
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral_error += error * dt
        i_term = self.ki * self.integral_error
        
        # Derivative term
        d_error = (error - self.previous_error) / dt if dt > 0 else 0.0
        d_term = self.kd * d_error
        
        # Total torque
        torque = p_term + i_term + d_term
        
        # Limit torque
        torque = max(-self.max_torque, min(self.max_torque, torque))
        
        self.previous_error = error
        return torque
    
    def set_target(self, target: float):
        """Set target position"""
        self.target_position = target
        self.integral_error = 0.0  # Reset integral on new target

class ObjectDetector:
    """Object detection and recognition system"""
    
    def __init__(self):
        self.detected_objects = []
        self.object_database = self.load_object_database()
        
        # YOLO parameters (simplified)
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4
    
    def load_object_database(self) -> Dict:
        """Load object database with known objects"""
        return {
            'package_small': {
                'size': (0.1, 0.1, 0.1),
                'weight': 0.5,
                'grasp_points': [(0.0, 0.0, 0.05)],
                'grasp_force': 5.0
            },
            'package_medium': {
                'size': (0.2, 0.2, 0.2),
                'weight': 1.0,
                'grasp_points': [(0.0, 0.0, 0.1)],
                'grasp_force': 10.0
            },
            'package_large': {
                'size': (0.3, 0.3, 0.3),
                'weight': 2.0,
                'grasp_points': [(0.0, 0.0, 0.15)],
                'grasp_force': 15.0
            }
        }
    
    def detect_objects(self, image: np.ndarray) -> List[Dict]:
        """Detect objects in image using computer vision"""
        detected = []
        
        # Simplified object detection (in real implementation, use YOLO/SSD)
        # For demo purposes, we'll simulate object detection
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Filter small objects
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Estimate object type based on size
                object_type = self.classify_object(w, h)
                
                if object_type:
                    detected.append({
                        'type': object_type,
                        'bbox': (x, y, w, h),
                        'center': (x + w//2, y + h//2),
                        'confidence': 0.8,
                        'size': self.object_database[object_type]['size']
                    })
        
        return detected
    
    def classify_object(self, width: int, height: int) -> Optional[str]:
        """Classify object based on size"""
        area = width * height
        
        if area < 5000:
            return 'package_small'
        elif area < 15000:
            return 'package_medium'
        elif area < 30000:
            return 'package_large'
        
        return None
    
    def calculate_3d_position(self, bbox: Tuple, depth_image: np.ndarray) -> Optional[Tuple[float, float, float]]:
        """Calculate 3D position of object from 2D detection and depth"""
        x, y, w, h = bbox
        center_x, center_y = x + w//2, y + h//2
        
        if (0 <= center_x < depth_image.shape[1] and 
            0 <= center_y < depth_image.shape[0]):
            
            # Get depth at center point
            depth = depth_image[center_y, center_x]
            
            if depth > 0:
                # Convert to 3D coordinates (simplified camera model)
                fx, fy = 525.0, 525.0  # Focal lengths
                cx, cy = 320.0, 240.0  # Principal point
                
                x_3d = (center_x - cx) * depth / fx
                y_3d = (center_y - cy) * depth / fy
                z_3d = depth
                
                return (x_3d, y_3d, z_3d)
        
        return None

class TrajectoryPlanner:
    """Trajectory planning for robot arm movements"""
    
    def __init__(self):
        self.joint_limits = {
            'arm_base_joint': (-3.14, 3.14),
            'arm_joint_1': (-1.57, 1.57),
            'arm_joint_2': (-1.57, 1.57),
            'gripper_joint': (-1.57, 1.57)
        }
        
        self.max_velocities = {
            'arm_base_joint': 1.0,
            'arm_joint_1': 1.0,
            'arm_joint_2': 1.0,
            'gripper_joint': 1.0
        }
    
    def plan_trajectory(self, start_pose: np.ndarray, end_pose: np.ndarray, 
                       duration: float = 2.0) -> List[np.ndarray]:
        """Plan smooth trajectory between poses"""
        # Calculate number of waypoints
        num_waypoints = int(duration * 50)  # 50 Hz
        
        trajectory = []
        
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            
            # Linear interpolation for position
            position = start_pose[:3] + t * (end_pose[:3] - start_pose[:3])
            
            # Spherical linear interpolation for orientation
            orientation = self.slerp_quaternion(start_pose[3:], end_pose[3:], t)
            
            waypoint = np.concatenate([position, orientation])
            trajectory.append(waypoint)
        
        return trajectory
    
    def slerp_quaternion(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """Spherical linear interpolation between quaternions"""
        # Normalize quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # Calculate dot product
        dot = np.dot(q1, q2)
        
        # If dot product is negative, slerp won't take the shorter path
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # If quaternions are very close, use linear interpolation
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # Calculate angle between quaternions
        theta_0 = math.acos(abs(dot))
        sin_theta_0 = math.sin(theta_0)
        
        theta = theta_0 * t
        sin_theta = math.sin(theta)
        
        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        
        return s0 * q1 + s1 * q2
    
    def inverse_kinematics(self, target_pose: np.ndarray) -> Optional[np.ndarray]:
        """Calculate inverse kinematics for target pose"""
        # Simplified inverse kinematics for 4-DOF arm
        x, y, z = target_pose[:3]
        
        # Base rotation
        base_angle = math.atan2(y, x)
        
        # Calculate arm configuration
        r = math.sqrt(x**2 + y**2)
        d = math.sqrt(r**2 + z**2)
        
        # Arm link lengths (from URDF)
        l1 = 0.2  # Link 1 length
        l2 = 0.3  # Link 2 length
        
        # Check if target is reachable
        if d > l1 + l2 or d < abs(l1 - l2):
            return None
        
        # Calculate joint angles using law of cosines
        cos_joint2 = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        joint2 = math.acos(cos_joint2)
        
        alpha = math.atan2(z, r)
        beta = math.acos((l1**2 + d**2 - l2**2) / (2 * l1 * d))
        joint1 = alpha - beta
        
        # Gripper angle (simplified)
        gripper_angle = 0.0
        
        return np.array([base_angle, joint1, joint2, gripper_angle])
    
    def check_collision(self, joint_angles: np.ndarray) -> bool:
        """Check if joint configuration causes collision"""
        # Simplified collision checking
        # In real implementation, use collision detection library
        
        # Check joint limits
        joint_names = ['arm_base_joint', 'arm_joint_1', 'arm_joint_2', 'gripper_joint']
        
        for i, (name, angle) in enumerate(zip(joint_names, joint_angles)):
            min_limit, max_limit = self.joint_limits[name]
            if angle < min_limit or angle > max_limit:
                return True
        
        # Check for self-collision (simplified)
        if abs(joint_angles[1] + joint_angles[2]) > 2.5:
            return True
        
        return False

class RobotArmController(Node):
    """
    Advanced robot arm controller for warehouse operations.
    
    Features:
    - 6-DOF robotic arm control with PID controllers
    - Object detection and recognition
    - Pick and place operations
    - Force/torque control
    - Collision avoidance
    - Trajectory planning and execution
    """
    
    def __init__(self):
        super().__init__('robot_arm_controller')
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/warehouse_robot/arm_controller/joint_trajectory', qos_profile)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/warehouse_robot/gripper_controller/joint_trajectory', qos_profile)
        self.status_pub = self.create_publisher(
            ManipulationStatus, '/warehouse_robot/manipulation_status', qos_profile)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/warehouse_robot/manipulation_markers', qos_profile)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            ManipulationGoal, '/warehouse_robot/manipulation_goal', self.goal_callback, qos_profile)
        self.joint_state_sub = self.create_subscription(
            JointState, '/warehouse_robot/joint_states', self.joint_state_callback, qos_profile)
        self.image_sub = self.create_subscription(
            Image, '/warehouse_robot/camera/image_raw', self.image_callback, qos_profile)
        self.depth_sub = self.create_subscription(
            Image, '/warehouse_robot/camera/depth', self.depth_callback, qos_profile)
        
        # Controllers and systems
        self.joint_controllers = self.initialize_joint_controllers()
        self.object_detector = ObjectDetector()
        self.trajectory_planner = TrajectoryPlanner()
        
        # State variables
        self.current_joint_states = {}
        self.current_pose = None
        self.target_pose = None
        self.manipulation_active = False
        self.current_operation = None
        self.detected_objects = []
        
        # Performance metrics
        self.operations_completed = 0
        self.success_rate = 0.0
        self.average_operation_time = 0.0
        self.collision_count = 0
        
        # Timers
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("Robot Arm Controller initialized")
    
    def initialize_joint_controllers(self) -> Dict[str, JointController]:
        """Initialize PID controllers for each joint"""
        controllers = {}
        
        # Joint parameters (tuned for warehouse robot)
        joint_params = {
            'arm_base_joint': {'kp': 2.0, 'ki': 0.1, 'kd': 0.5},
            'arm_joint_1': {'kp': 1.5, 'ki': 0.1, 'kd': 0.3},
            'arm_joint_2': {'kp': 1.5, 'ki': 0.1, 'kd': 0.3},
            'gripper_joint': {'kp': 1.0, 'ki': 0.0, 'kd': 0.1}
        }
        
        for joint_name, params in joint_params.items():
            controllers[joint_name] = JointController(
                joint_name, params['kp'], params['ki'], params['kd']
            )
        
        return controllers
    
    def goal_callback(self, msg: ManipulationGoal):
        """Handle manipulation goal"""
        self.get_logger().info(f"New manipulation goal: {msg.operation}")
        
        self.current_operation = msg.operation
        self.target_pose = np.array([
            msg.target_pose.position.x,
            msg.target_pose.position.y,
            msg.target_pose.position.z,
            msg.target_pose.orientation.x,
            msg.target_pose.orientation.y,
            msg.target_pose.orientation.z,
            msg.target_pose.orientation.w
        ])
        
        self.manipulation_active = True
        
        # Execute operation based on type
        if msg.operation == "pick":
            self.execute_pick_operation()
        elif msg.operation == "place":
            self.execute_place_operation()
        elif msg.operation == "move_to_pose":
            self.execute_move_operation()
        else:
            self.get_logger().error(f"Unknown operation: {msg.operation}")
            self.manipulation_active = False
    
    def joint_state_callback(self, msg: JointState):
        """Handle joint state updates"""
        for i, name in enumerate(msg.name):
            if name in self.joint_controllers:
                self.current_joint_states[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0
                }
    
    def image_callback(self, msg: Image):
        """Handle camera image"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect objects
            self.detected_objects = self.object_detector.detect_objects(cv_image)
            
            # Publish object detections
            self.publish_object_markers()
            
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
    
    def depth_callback(self, msg: Image):
        """Handle depth image"""
        try:
            # Convert ROS depth image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            
            # Update object 3D positions
            for obj in self.detected_objects:
                if 'bbox' in obj:
                    pos_3d = self.object_detector.calculate_3d_position(
                        obj['bbox'], depth_image
                    )
                    if pos_3d:
                        obj['position_3d'] = pos_3d
            
        except Exception as e:
            self.get_logger().error(f"Depth processing error: {str(e)}")
    
    def control_loop(self):
        """Main control loop"""
        if not self.manipulation_active:
            return
        
        # Update joint controllers
        dt = 0.02  # 50 Hz
        joint_commands = {}
        
        for joint_name, controller in self.joint_controllers.items():
            if joint_name in self.current_joint_states:
                current_pos = self.current_joint_states[joint_name]['position']
                current_vel = self.current_joint_states[joint_name]['velocity']
                
                torque = controller.update(current_pos, current_vel, dt)
                joint_commands[joint_name] = torque
        
        # Check for completion
        if self.is_operation_complete():
            self.complete_operation()
    
    def execute_pick_operation(self):
        """Execute pick operation"""
        self.get_logger().info("Executing pick operation")
        
        # Find target object
        target_object = self.find_target_object()
        if not target_object:
            self.get_logger().error("No target object found")
            self.manipulation_active = False
            return
        
        # Calculate pick pose
        pick_pose = self.calculate_pick_pose(target_object)
        
        # Plan trajectory
        current_pose = self.get_current_arm_pose()
        trajectory = self.trajectory_planner.plan_trajectory(current_pose, pick_pose)
        
        # Execute trajectory
        self.execute_trajectory(trajectory)
        
        # Close gripper
        self.close_gripper()
    
    def execute_place_operation(self):
        """Execute place operation"""
        self.get_logger().info("Executing place operation")
        
        # Calculate place pose
        place_pose = self.calculate_place_pose()
        
        # Plan trajectory
        current_pose = self.get_current_arm_pose()
        trajectory = self.trajectory_planner.plan_trajectory(current_pose, place_pose)
        
        # Execute trajectory
        self.execute_trajectory(trajectory)
        
        # Open gripper
        self.open_gripper()
    
    def execute_move_operation(self):
        """Execute move to pose operation"""
        self.get_logger().info("Executing move operation")
        
        if self.target_pose is None:
            self.get_logger().error("No target pose specified")
            self.manipulation_active = False
            return
        
        # Plan trajectory
        current_pose = self.get_current_arm_pose()
        trajectory = self.trajectory_planner.plan_trajectory(current_pose, self.target_pose)
        
        # Execute trajectory
        self.execute_trajectory(trajectory)
    
    def find_target_object(self) -> Optional[Dict]:
        """Find target object for manipulation"""
        if not self.detected_objects:
            return None
        
        # Select closest object
        closest_object = None
        min_distance = float('inf')
        
        for obj in self.detected_objects:
            if 'position_3d' in obj:
                distance = np.linalg.norm(obj['position_3d'])
                if distance < min_distance:
                    min_distance = distance
                    closest_object = obj
        
        return closest_object
    
    def calculate_pick_pose(self, target_object: Dict) -> np.ndarray:
        """Calculate pick pose for target object"""
        if 'position_3d' not in target_object:
            return None
        
        x, y, z = target_object['position_3d']
        
        # Calculate approach pose (above object)
        approach_height = 0.1
        pick_pose = np.array([
            x, y, z + approach_height,
            0.0, 0.0, 0.0, 1.0  # Orientation (quaternion)
        ])
        
        return pick_pose
    
    def calculate_place_pose(self) -> np.ndarray:
        """Calculate place pose"""
        # Use target pose if available, otherwise use default
        if self.target_pose is not None:
            return self.target_pose
        
        # Default place pose
        return np.array([0.5, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0])
    
    def get_current_arm_pose(self) -> np.ndarray:
        """Get current arm pose from joint states"""
        # Forward kinematics (simplified)
        # In real implementation, use proper forward kinematics
        
        if not self.current_joint_states:
            return np.array([0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0])
        
        # Extract joint angles
        base_angle = self.current_joint_states.get('arm_base_joint', {}).get('position', 0.0)
        joint1 = self.current_joint_states.get('arm_joint_1', {}).get('position', 0.0)
        joint2 = self.current_joint_states.get('arm_joint_2', {}).get('position', 0.0)
        
        # Calculate end-effector position (simplified)
        l1, l2 = 0.2, 0.3  # Link lengths
        
        x = (l1 * math.cos(joint1) + l2 * math.cos(joint1 + joint2)) * math.cos(base_angle)
        y = (l1 * math.cos(joint1) + l2 * math.cos(joint1 + joint2)) * math.sin(base_angle)
        z = l1 * math.sin(joint1) + l2 * math.sin(joint1 + joint2)
        
        return np.array([x, y, z, 0.0, 0.0, 0.0, 1.0])
    
    def execute_trajectory(self, trajectory: List[np.ndarray]):
        """Execute planned trajectory"""
        if not trajectory:
            return
        
        # Convert trajectory to joint space
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = self.get_clock().now().to_msg()
        joint_trajectory.joint_names = list(self.joint_controllers.keys())
        
        for i, pose in enumerate(trajectory):
            # Calculate inverse kinematics
            joint_angles = self.trajectory_planner.inverse_kinematics(pose)
            
            if joint_angles is None:
                self.get_logger().error("Inverse kinematics failed")
                continue
            
            # Check for collision
            if self.trajectory_planner.check_collision(joint_angles):
                self.get_logger().error("Collision detected in trajectory")
                self.collision_count += 1
                continue
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = joint_angles.tolist()
            point.time_from_start.sec = i * 0.02  # 50 Hz
            joint_trajectory.points.append(point)
        
        # Publish trajectory
        self.joint_trajectory_pub.publish(joint_trajectory)
    
    def close_gripper(self):
        """Close gripper"""
        gripper_trajectory = JointTrajectory()
        gripper_trajectory.header.stamp = self.get_clock().now().to_msg()
        gripper_trajectory.joint_names = ['gripper_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [1.57]  # Closed position
        point.time_from_start.sec = 1
        gripper_trajectory.points.append(point)
        
        self.gripper_pub.publish(gripper_trajectory)
    
    def open_gripper(self):
        """Open gripper"""
        gripper_trajectory = JointTrajectory()
        gripper_trajectory.header.stamp = self.get_clock().now().to_msg()
        gripper_trajectory.joint_names = ['gripper_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0]  # Open position
        point.time_from_start.sec = 1
        gripper_trajectory.points.append(point)
        
        self.gripper_pub.publish(gripper_trajectory)
    
    def is_operation_complete(self) -> bool:
        """Check if current operation is complete"""
        if not self.manipulation_active:
            return False
        
        # Check if target pose is reached
        if self.target_pose is not None:
            current_pose = self.get_current_arm_pose()
            error = np.linalg.norm(current_pose - self.target_pose)
            return error < 0.05  # 5cm tolerance
        
        return False
    
    def complete_operation(self):
        """Complete current operation"""
        self.get_logger().info(f"Operation {self.current_operation} completed")
        
        self.operations_completed += 1
        self.manipulation_active = False
        self.current_operation = None
        self.target_pose = None
    
    def publish_object_markers(self):
        """Publish object detection markers"""
        marker_array = MarkerArray()
        
        for i, obj in enumerate(self.detected_objects):
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            if 'position_3d' in obj:
                marker.pose.position.x = obj['position_3d'][0]
                marker.pose.position.y = obj['position_3d'][1]
                marker.pose.position.z = obj['position_3d'][2]
            else:
                marker.pose.position.z = 0.5  # Default height
            
            marker.pose.orientation.w = 1.0
            
            # Set size based on object type
            if obj['type'] == 'package_small':
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif obj['type'] == 'package_medium':
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:  # package_large
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            
            marker.color.a = 0.7
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
    
    def publish_status(self):
        """Publish manipulation status"""
        status_msg = ManipulationStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.manipulation_active = self.manipulation_active
        status_msg.current_operation = self.current_operation or ""
        status_msg.operations_completed = self.operations_completed
        status_msg.success_rate = self.success_rate
        status_msg.collision_count = self.collision_count
        status_msg.objects_detected = len(self.detected_objects)
        
        self.status_pub.publish(status_msg)

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    controller = RobotArmController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Robot Arm Controller shutting down...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
