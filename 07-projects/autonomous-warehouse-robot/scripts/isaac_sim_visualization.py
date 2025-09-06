#!/usr/bin/env python3
"""
Isaac Sim Robot Visualization Script

This script loads and visualizes the warehouse robot in Isaac Sim.
It creates a simple environment and demonstrates basic robot movement.

Author: Tanmay Pancholi
Date: 2024
"""

import omni.usd
import omni.kit.commands
from pxr import UsdGeom, Gf, UsdPhysics, UsdLux
import numpy as np
import time
import math

class IsaacSimRobotVisualization:
    """Isaac Sim robot visualization and control"""
    
    def __init__(self):
        """Initialize the visualization"""
        self.stage = omni.usd.get_context().get_stage()
        self.robot_prim = None
        self.robot_path = "/World/WarehouseRobot"
        
        print("Isaac Sim Robot Visualization initialized")
    
    def create_environment(self):
        """Create a simple warehouse environment"""
        print("Creating warehouse environment...")
        
        # Create ground plane
        ground_path = "/World/Ground"
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Plane",
            prim_path=ground_path
        )
        
        # Scale the ground
        ground_prim = self.stage.GetPrimAtPath(ground_path)
        xform = UsdGeom.Xformable(ground_prim)
        scale = Gf.Vec3f(10.0, 10.0, 1.0)
        xform.AddScaleOp().Set(scale)
        
        # Add physics to ground
        UsdPhysics.CollisionAPI.Apply(ground_prim)
        
        # Create some warehouse shelves
        self.create_warehouse_shelves()
        
        # Add lighting
        self.setup_lighting()
        
        print("Environment created successfully!")
    
    def create_warehouse_shelves(self):
        """Create warehouse shelves for the environment"""
        print("Creating warehouse shelves...")
        
        # Create shelf positions
        shelf_positions = [
            (2.0, 2.0, 0.0),
            (-2.0, 2.0, 0.0),
            (2.0, -2.0, 0.0),
            (-2.0, -2.0, 0.0)
        ]
        
        for i, (x, y, z) in enumerate(shelf_positions):
            shelf_path = f"/World/Shelf_{i}"
            
            # Create shelf base
            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="Cube",
                prim_path=shelf_path
            )
            
            shelf_prim = self.stage.GetPrimAtPath(shelf_path)
            xform = UsdGeom.Xformable(shelf_prim)
            
            # Position the shelf
            xform.AddTranslateOp().Set(Gf.Vec3f(x, y, z))
            xform.AddScaleOp().Set(Gf.Vec3f(0.5, 0.2, 1.0))
            
            # Add physics
            UsdPhysics.RigidBodyAPI.Apply(shelf_prim)
            UsdPhysics.CollisionAPI.Apply(shelf_prim)
            
            # Create some packages on the shelf
            self.create_packages_on_shelf(shelf_path, x, y, z)
    
    def create_packages_on_shelf(self, shelf_path, shelf_x, shelf_y, shelf_z):
        """Create packages on a shelf"""
        package_types = [
            {"size": (0.1, 0.1, 0.1), "color": (1.0, 0.0, 0.0)},  # Small red
            {"size": (0.15, 0.15, 0.15), "color": (0.0, 1.0, 0.0)},  # Medium green
            {"size": (0.2, 0.2, 0.2), "color": (0.0, 0.0, 1.0)}   # Large blue
        ]
        
        for i, package_type in enumerate(package_types):
            package_path = f"{shelf_path}/Package_{i}"
            
            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="Cube",
                prim_path=package_path
            )
            
            package_prim = self.stage.GetPrimAtPath(package_path)
            xform = UsdGeom.Xformable(package_prim)
            
            # Position package on shelf
            package_x = shelf_x + (i - 1) * 0.3
            package_y = shelf_y
            package_z = shelf_z + 0.5 + package_type["size"][2] / 2
            
            xform.AddTranslateOp().Set(Gf.Vec3f(package_x, package_y, package_z))
            xform.AddScaleOp().Set(Gf.Vec3f(*package_type["size"]))
            
            # Add physics
            UsdPhysics.RigidBodyAPI.Apply(package_prim)
            UsdPhysics.CollisionAPI.Apply(package_prim)
    
    def setup_lighting(self):
        """Setup lighting for the scene"""
        print("Setting up lighting...")
        
        # Create directional light
        light_path = "/World/DirectionalLight"
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="DirectionalLight",
            prim_path=light_path
        )
        
        light_prim = self.stage.GetPrimAtPath(light_path)
        light = UsdLux.DirectionalLight(light_prim)
        light.CreateIntensityAttr(1000.0)
        light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        
        # Position the light
        xform = UsdGeom.Xformable(light_prim)
        xform.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 45.0, 0.0))
    
    def create_robot(self):
        """Create the warehouse robot"""
        print("Creating warehouse robot...")
        
        # Create robot base
        robot_base_path = f"{self.robot_path}/Base"
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Cube",
            prim_path=robot_base_path
        )
        
        robot_base_prim = self.stage.GetPrimAtPath(robot_base_path)
        xform = UsdGeom.Xformable(robot_base_prim)
        
        # Position robot at origin
        xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.1))
        xform.AddScaleOp().Set(Gf.Vec3f(0.6, 0.4, 0.2))
        
        # Add physics to robot base
        UsdPhysics.RigidBodyAPI.Apply(robot_base_prim)
        UsdPhysics.CollisionAPI.Apply(robot_base_prim)
        
        # Create robot body
        robot_body_path = f"{self.robot_path}/Body"
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Cylinder",
            prim_path=robot_body_path
        )
        
        robot_body_prim = self.stage.GetPrimAtPath(robot_body_path)
        xform = UsdGeom.Xformable(robot_body_prim)
        xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.3))
        xform.AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.2))
        
        # Create wheels
        self.create_robot_wheels()
        
        # Create LiDAR sensor
        self.create_lidar_sensor()
        
        # Create camera
        self.create_camera_sensor()
        
        # Create robot arm
        self.create_robot_arm()
        
        # Set robot as main prim
        self.robot_prim = self.stage.GetPrimAtPath(self.robot_path)
        
        print("Robot created successfully!")
    
    def create_robot_wheels(self):
        """Create robot wheels"""
        wheel_positions = [
            (0.0, 0.25, 0.1),   # Left wheel
            (0.0, -0.25, 0.1),  # Right wheel
            (0.2, 0.0, 0.05),   # Caster wheel
        ]
        
        wheel_names = ["LeftWheel", "RightWheel", "CasterWheel"]
        
        for i, (x, y, z) in enumerate(wheel_positions):
            wheel_path = f"{self.robot_path}/{wheel_names[i]}"
            
            if i < 2:  # Main wheels
                omni.kit.commands.execute(
                    "CreatePrimWithDefaultXform",
                    prim_type="Cylinder",
                    prim_path=wheel_path
                )
                
                wheel_prim = self.stage.GetPrimAtPath(wheel_path)
                xform = UsdGeom.Xformable(wheel_prim)
                xform.AddTranslateOp().Set(Gf.Vec3f(x, y, z))
                xform.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, 0.0))
                xform.AddScaleOp().Set(Gf.Vec3f(0.1, 0.1, 0.05))
                
            else:  # Caster wheel
                omni.kit.commands.execute(
                    "CreatePrimWithDefaultXform",
                    prim_type="Sphere",
                    prim_path=wheel_path
                )
                
                wheel_prim = self.stage.GetPrimAtPath(wheel_path)
                xform = UsdGeom.Xformable(wheel_prim)
                xform.AddTranslateOp().Set(Gf.Vec3f(x, y, z))
                xform.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.05))
            
            # Add physics to wheels
            UsdPhysics.RigidBodyAPI.Apply(wheel_prim)
            UsdPhysics.CollisionAPI.Apply(wheel_prim)
    
    def create_lidar_sensor(self):
        """Create LiDAR sensor"""
        lidar_path = f"{self.robot_path}/LiDAR"
        
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Cylinder",
            prim_path=lidar_path
        )
        
        lidar_prim = self.stage.GetPrimAtPath(lidar_path)
        xform = UsdGeom.Xformable(lidar_prim)
        xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.2))
        xform.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.1))
        
        # Make it red to indicate it's a sensor
        # Note: In a real implementation, you'd add material properties
    
    def create_camera_sensor(self):
        """Create camera sensor"""
        camera_path = f"{self.robot_path}/Camera"
        
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Cube",
            prim_path=camera_path
        )
        
        camera_prim = self.stage.GetPrimAtPath(camera_path)
        xform = UsdGeom.Xformable(camera_prim)
        xform.AddTranslateOp().Set(Gf.Vec3f(0.2, 0.0, 0.3))
        xform.AddScaleOp().Set(Gf.Vec3f(0.1, 0.1, 0.05))
    
    def create_robot_arm(self):
        """Create simple robot arm"""
        arm_base_path = f"{self.robot_path}/ArmBase"
        
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Cylinder",
            prim_path=arm_base_path
        )
        
        arm_base_prim = self.stage.GetPrimAtPath(arm_base_path)
        xform = UsdGeom.Xformable(arm_base_prim)
        xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.25))
        xform.AddScaleOp().Set(Gf.Vec3f(0.08, 0.08, 0.1))
        
        # Create arm link 1
        arm_link1_path = f"{self.robot_path}/ArmLink1"
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Cube",
            prim_path=arm_link1_path
        )
        
        arm_link1_prim = self.stage.GetPrimAtPath(arm_link1_path)
        xform = UsdGeom.Xformable(arm_link1_prim)
        xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.35))
        xform.AddScaleOp().Set(Gf.Vec3f(0.1, 0.1, 0.2))
        
        # Create arm link 2
        arm_link2_path = f"{self.robot_path}/ArmLink2"
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Cube",
            prim_path=arm_link2_path
        )
        
        arm_link2_prim = self.stage.GetPrimAtPath(arm_link2_path)
        xform = UsdGeom.Xformable(arm_link2_prim)
        xform.AddTranslateOp().Set(Gf.Vec3f(0.15, 0.0, 0.35))
        xform.AddScaleOp().Set(Gf.Vec3f(0.3, 0.08, 0.08))
        
        # Create gripper
        gripper_path = f"{self.robot_path}/Gripper"
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Cube",
            prim_path=gripper_path
        )
        
        gripper_prim = self.stage.GetPrimAtPath(gripper_path)
        xform = UsdGeom.Xformable(gripper_prim)
        xform.AddTranslateOp().Set(Gf.Vec3f(0.3, 0.0, 0.35))
        xform.AddScaleOp().Set(Gf.Vec3f(0.1, 0.1, 0.05))
    
    def move_robot(self, x, y, theta):
        """Move the robot to a new position"""
        if not self.robot_prim:
            print("Robot not created yet!")
            return
        
        print(f"Moving robot to position: ({x}, {y}, {theta})")
        
        xform = UsdGeom.Xformable(self.robot_prim)
        
        # Set position
        xform.AddTranslateOp().Set(Gf.Vec3f(x, y, 0.0))
        
        # Set rotation
        xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, math.degrees(theta)))
    
    def run_demo(self):
        """Run a simple demonstration"""
        print("Starting robot demonstration...")
        
        # Create environment and robot
        self.create_environment()
        self.create_robot()
        
        # Wait a moment for everything to load
        time.sleep(2)
        
        # Demo movement sequence
        demo_positions = [
            (0.0, 0.0, 0.0),      # Start
            (2.0, 0.0, 0.0),      # Move forward
            (2.0, 2.0, 1.57),     # Turn right and move
            (0.0, 2.0, 3.14),     # Turn around and move back
            (0.0, 0.0, 4.71),     # Return to start
        ]
        
        for i, (x, y, theta) in enumerate(demo_positions):
            print(f"Demo step {i+1}: Moving to ({x}, {y}, {theta})")
            self.move_robot(x, y, theta)
            time.sleep(3)  # Wait 3 seconds between moves
        
        print("Demo completed!")
        print("You can now interact with the robot in Isaac Sim!")
        print("Try moving the camera around to see the robot from different angles.")

def main():
    """Main function"""
    print("=" * 60)
    print("Isaac Sim Robot Visualization")
    print("=" * 60)
    
    # Create visualization instance
    viz = IsaacSimRobotVisualization()
    
    # Run the demo
    viz.run_demo()
    
    print("=" * 60)
    print("Visualization complete!")
    print("The robot should now be visible in Isaac Sim.")
    print("You can:")
    print("- Move the camera around to explore")
    print("- Use the timeline to control playback")
    print("- Modify the robot or environment")
    print("=" * 60)

if __name__ == "__main__":
    main()
