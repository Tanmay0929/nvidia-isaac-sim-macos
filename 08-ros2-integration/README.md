# ROS2 Integration with Isaac Sim

This section covers integrating Isaac Sim with ROS2 for advanced robot simulation and control.

## 🎯 Overview

ROS2 integration allows you to:
- Control robots in Isaac Sim from ROS2 nodes
- Stream sensor data from Isaac Sim to ROS2
- Use ROS2 navigation and control packages
- Bridge between simulation and real robots

## 📋 Prerequisites

- Isaac Sim installed and running
- ROS2 Docker container running (see `09-docker-setup/`)
- Basic understanding of ROS2 concepts
- Python 3.8+ knowledge

## 🏗️ Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Isaac Sim     │    │   ROS2 Bridge   │    │   ROS2 Nodes    │
│                 │    │                 │    │                 │
│  - Robot Models │◄──►│  - Topic Bridge │◄──►│  - Controllers  │
│  - Sensors      │    │  - Service Bridge│    │  - Navigation   │
│  - Physics      │    │  - Action Bridge │    │  - Perception   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 🚀 Quick Start

### 1. Start ROS2 Container
```bash
cd 09-docker-setup
./start-ros2-container.sh
```

### 2. Start Isaac Sim with ROS2 Bridge
```bash
cd 09-docker-setup
./start-isaac-sim.sh
```

### 3. Enable ROS2 Bridge in Isaac Sim
1. Open Isaac Sim
2. Go to Window → Extensions
3. Search for "ROS2 Bridge"
4. Enable the extension
5. Configure connection settings

## 📚 Tutorials

### Basic Integration
- [Setting up ROS2 Bridge](setup-ros2-bridge.md)
- [Basic Topic Communication](basic-topic-communication.md)
- [Robot Control via ROS2](robot-control-ros2.md)

### Advanced Topics
- [Sensor Data Streaming](sensor-data-streaming.md)
- [Service and Action Integration](service-action-integration.md)
- [Multi-Robot Coordination](multi-robot-coordination.md)

### Real-World Applications
- [Navigation Stack Integration](navigation-stack-integration.md)
- [SLAM in Simulation](slam-simulation.md)
- [Manipulation Planning](manipulation-planning.md)

## 🛠️ Configuration

### ROS2 Bridge Settings
```python
# Example bridge configuration
bridge_config = {
    "ros_domain_id": 0,
    "ros_distro": "humble",
    "bridge_host": "localhost",
    "bridge_port": 11311
}
```

### Topic Mapping
```python
# Example topic mappings
topic_mappings = {
    "/cmd_vel": "/isaac/cmd_vel",
    "/odom": "/isaac/odom",
    "/scan": "/isaac/scan",
    "/camera/image": "/isaac/camera/image"
}
```

## 📁 Project Structure

```
08-ros2-integration/
├── README.md                    # This file
├── setup-ros2-bridge.md        # Bridge setup guide
├── basic-topic-communication.md # Basic communication
├── robot-control-ros2.md       # Robot control
├── sensor-data-streaming.md    # Sensor integration
├── service-action-integration.md # Services and actions
├── multi-robot-coordination.md # Multi-robot setup
├── navigation-stack-integration.md # Navigation
├── slam-simulation.md          # SLAM integration
├── manipulation-planning.md    # Manipulation
├── examples/                   # Code examples
│   ├── basic_bridge.py        # Basic bridge example
│   ├── robot_controller.py    # Robot controller
│   ├── sensor_publisher.py    # Sensor publisher
│   └── navigation_node.py     # Navigation node
├── config/                    # Configuration files
│   ├── bridge_config.yaml    # Bridge settings
│   ├── topic_mappings.yaml   # Topic mappings
│   └── robot_config.yaml     # Robot settings
└── launch/                   # Launch files
    ├── isaac_sim.launch.py   # Isaac Sim launch
    ├── robot_control.launch.py # Robot control
    └── navigation.launch.py  # Navigation stack
```

## 🔧 Common Tasks

### Publishing Robot State
```python
# In Isaac Sim
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.robots import Robot

# Create robot
robot = Robot(prim_path="/World/Robot")
robot.initialize()

# Publish odometry
odom_publisher = robot.get_odometry_publisher()
odom_publisher.publish()
```

### Subscribing to Commands
```python
# In Isaac Sim
from omni.isaac.core.utils.stage import add_reference_to_stage

# Subscribe to velocity commands
def cmd_vel_callback(msg):
    robot.set_linear_velocity([msg.linear.x, msg.linear.y, 0])
    robot.set_angular_velocity([0, 0, msg.angular.z])

cmd_vel_subscriber = robot.get_velocity_subscriber(cmd_vel_callback)
```

## 🐛 Troubleshooting

### Common Issues

**Bridge Connection Failed:**
- Check ROS2 container is running
- Verify network settings
- Check firewall configuration

**Topics Not Appearing:**
- Verify topic names match
- Check ROS2 domain ID
- Restart bridge extension

**Performance Issues:**
- Reduce publish frequency
- Use compressed topics
- Optimize message types

### Debug Commands
```bash
# Check ROS2 topics
docker exec -it isaac-sim-ros2-dev bash
ros2 topic list

# Monitor topic data
ros2 topic echo /isaac/cmd_vel

# Check node status
ros2 node list
```

## 📖 Next Steps

After mastering ROS2 integration:
1. Explore `07-projects/` for complete applications
2. Check out `06-advanced-topics/` for advanced features
3. Build your own robot control systems

## 🔗 Resources

- [Isaac Sim ROS2 Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials.html)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Isaac Sim ROS2 Examples](https://github.com/NVIDIA-Omniverse/IsaacSim-ros_workspaces)
