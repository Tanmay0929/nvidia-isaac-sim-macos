"""
Simple Robot Movement Example for Isaac Sim

This example demonstrates basic robot movement using position commands.
Perfect for beginners learning robot simulation in Isaac Sim.

Author: Isaac Sim Learning Environment
Date: 2024
"""

import omni.usd
import omni.kit.commands
from pxr import UsdGeom, Gf, UsdPhysics
import numpy as np
import time

class SimpleRobotMovement:
    def __init__(self):
        """Initialize the robot movement example."""
        self.stage = omni.usd.get_context().get_stage()
        self.robot_prim = None
        self.initial_position = None
        
    def create_simple_robot(self):
        """Create a simple robot using basic shapes."""
        print("Creating simple robot...")
        
        # Create robot base (cube)
        robot_base_path = "/World/Robot/Base"
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Cube",
            prim_path=robot_base_path
        )
        
        # Create robot body (cylinder)
        robot_body_path = "/World/Robot/Body"
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Cylinder",
            prim_path=robot_body_path
        )
        
        # Create wheels (spheres)
        wheel_paths = [
            "/World/Robot/Wheel_Front_Left",
            "/World/Robot/Wheel_Front_Right",
            "/World/Robot/Wheel_Rear_Left",
            "/World/Robot/Wheel_Rear_Right"
        ]
        
        for wheel_path in wheel_paths:
            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="Sphere",
                prim_path=wheel_path
            )
        
        # Set robot as the main prim
        self.robot_prim = self.stage.GetPrimAtPath("/World/Robot")
        
        # Store initial position
        if self.robot_prim:
            xform = UsdGeom.Xformable(self.robot_prim)
            self.initial_position = xform.GetLocalTransformation().ExtractTranslation()
            print(f"Robot created at position: {self.initial_position}")
        
        return self.robot_prim is not None
    
    def setup_physics(self):
        """Set up physics for the robot."""
        print("Setting up physics...")
        
        # Enable physics on robot base
        base_prim = self.stage.GetPrimAtPath("/World/Robot/Base")
        if base_prim:
            UsdPhysics.RigidBodyAPI.Apply(base_prim)
            UsdPhysics.CollisionAPI.Apply(base_prim)
        
        # Enable physics on wheels
        wheel_paths = [
            "/World/Robot/Wheel_Front_Left",
            "/World/Robot/Wheel_Front_Right",
            "/World/Robot/Wheel_Rear_Left",
            "/World/Robot/Wheel_Rear_Right"
        ]
        
        for wheel_path in wheel_paths:
            wheel_prim = self.stage.GetPrimAtPath(wheel_path)
            if wheel_prim:
                UsdPhysics.RigidBodyAPI.Apply(wheel_prim)
                UsdPhysics.CollisionAPI.Apply(wheel_prim)
    
    def move_robot_forward(self, distance=2.0):
        """Move the robot forward by a specified distance."""
        if not self.robot_prim:
            print("Error: Robot not created yet!")
            return False
        
        print(f"Moving robot forward by {distance} units...")
        
        # Get current position
        xform = UsdGeom.Xformable(self.robot_prim)
        current_transform = xform.GetLocalTransformation()
        current_position = current_transform.ExtractTranslation()
        
        # Calculate new position
        new_position = Gf.Vec3f(
            current_position[0] + distance,
            current_position[1],
            current_position[2]
        )
        
        # Apply new position
        new_transform = Gf.Matrix4d(1.0)
        new_transform.SetTranslateOnly(new_position)
        xform.SetLocalTransformation(new_transform)
        
        print(f"Robot moved to position: {new_position}")
        return True
    
    def move_robot_backward(self, distance=2.0):
        """Move the robot backward by a specified distance."""
        if not self.robot_prim:
            print("Error: Robot not created yet!")
            return False
        
        print(f"Moving robot backward by {distance} units...")
        
        # Get current position
        xform = UsdGeom.Xformable(self.robot_prim)
        current_transform = xform.GetLocalTransformation()
        current_position = current_transform.ExtractTranslation()
        
        # Calculate new position
        new_position = Gf.Vec3f(
            current_position[0] - distance,
            current_position[1],
            current_position[2]
        )
        
        # Apply new position
        new_transform = Gf.Matrix4d(1.0)
        new_transform.SetTranslateOnly(new_position)
        xform.SetLocalTransformation(new_transform)
        
        print(f"Robot moved to position: {new_position}")
        return True
    
    def rotate_robot(self, angle_degrees=90.0):
        """Rotate the robot by a specified angle."""
        if not self.robot_prim:
            print("Error: Robot not created yet!")
            return False
        
        print(f"Rotating robot by {angle_degrees} degrees...")
        
        # Get current transform
        xform = UsdGeom.Xformable(self.robot_prim)
        current_transform = xform.GetLocalTransformation()
        
        # Create rotation matrix
        angle_radians = np.radians(angle_degrees)
        rotation_matrix = Gf.Matrix4d(1.0)
        rotation_matrix.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 0, 1), angle_radians))
        
        # Apply rotation
        new_transform = current_transform * rotation_matrix
        xform.SetLocalTransformation(new_transform)
        
        print(f"Robot rotated by {angle_degrees} degrees")
        return True
    
    def reset_robot_position(self):
        """Reset robot to initial position."""
        if not self.robot_prim or not self.initial_position:
            print("Error: Cannot reset robot position!")
            return False
        
        print("Resetting robot to initial position...")
        
        xform = UsdGeom.Xformable(self.robot_prim)
        reset_transform = Gf.Matrix4d(1.0)
        reset_transform.SetTranslateOnly(self.initial_position)
        xform.SetLocalTransformation(reset_transform)
        
        print(f"Robot reset to position: {self.initial_position}")
        return True
    
    def run_demo(self):
        """Run a demonstration of robot movement."""
        print("=" * 50)
        print("Simple Robot Movement Demo")
        print("=" * 50)
        
        # Create robot
        if not self.create_simple_robot():
            print("Failed to create robot!")
            return
        
        # Setup physics
        self.setup_physics()
        
        # Wait a moment for physics to initialize
        time.sleep(1)
        
        # Run movement sequence
        print("\nStarting movement sequence...")
        
        # Move forward
        self.move_robot_forward(3.0)
        time.sleep(2)
        
        # Rotate
        self.rotate_robot(90)
        time.sleep(2)
        
        # Move forward again
        self.move_robot_forward(2.0)
        time.sleep(2)
        
        # Move backward
        self.move_robot_backward(1.0)
        time.sleep(2)
        
        # Reset position
        self.reset_robot_position()
        
        print("\nDemo completed!")
        print("Try running the simulation to see the robot in action!")

def main():
    """Main function to run the example."""
    print("Starting Simple Robot Movement Example...")
    
    # Create movement controller
    robot_movement = SimpleRobotMovement()
    
    # Run the demo
    robot_movement.run_demo()

if __name__ == "__main__":
    main()
