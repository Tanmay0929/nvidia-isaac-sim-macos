# NVIDIA Isaac Sim for macOS

A comprehensive learning environment for robot simulations using NVIDIA Isaac Sim on macOS with ROS2 integration via Docker.

## 📁 Project Structure

```
nvidia-issac-sim/
├── 01-getting-started/          # Basic setup and first steps
├── 02-basic-simulations/        # Fundamental simulation concepts
├── 03-robot-models/            # Robot model creation and import
├── 04-sensors-and-perception/  # Camera, LiDAR, IMU sensors
├── 05-navigation-and-control/  # Path planning and robot control
├── 06-advanced-topics/         # Advanced simulation features
├── 07-projects/               # Complete project examples
├── 08-ros2-integration/       # ROS2 bridge and communication
├── 09-docker-setup/           # Docker configuration for ROS2
├── 10-documentation/          # Learning materials and guides
├── 11-scripts/                # Helper scripts and utilities
├── 12-assets/                 # 3D models, environments, textures
│   ├── environments/          # Simulation environments
│   ├── robots/               # Robot models and URDF files
│   ├── objects/              # Objects and props
│   └── textures/             # Materials and textures
├── 13-examples/              # Code examples by category
│   ├── basic-movement/       # Basic robot movement
│   ├── physics-simulation/   # Physics and dynamics
│   ├── sensor-integration/   # Sensor data processing
│   └── multi-robot/          # Multi-robot scenarios
├── 14-tutorials/             # Step-by-step tutorials
│   ├── step-by-step/         # Beginner tutorials
│   ├── advanced-concepts/    # Advanced topics
│   └── ros2-bridge/          # ROS2 integration tutorials
└── 15-tools-and-utilities/   # Development tools
```

## 🚀 Quick Start

### Prerequisites
- **macOS** (Intel or Apple Silicon)
- **NVIDIA Isaac Sim** (latest version)
- **Docker Desktop** (for ROS2)
- **Python 3.8+**
- **Git**

### Setup Steps
1. **Install Isaac Sim**: Download from [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
2. **Setup ROS2 with Docker**: Follow the guide in `09-docker-setup/`
3. **Clone this repository**: `git clone <your-repo-url>`
4. **Start with basics**: Begin with `01-getting-started/`

## 📚 Learning Path

### Beginner (Week 1-2)
1. **Getting Started** (`01-getting-started/`)
   - Isaac Sim installation and setup
   - Basic interface navigation
   - First simulation

2. **Basic Simulations** (`02-basic-simulations/`)
   - Creating simple environments
   - Adding basic objects
   - Physics simulation basics

### Intermediate (Week 3-4)
3. **Robot Models** (`03-robot-models/`)
   - Importing URDF files
   - Creating custom robots
   - Robot kinematics

4. **Sensors and Perception** (`04-sensors-and-perception/`)
   - Camera setup and calibration
   - LiDAR integration
   - Sensor data processing

### Advanced (Week 5-6)
5. **Navigation and Control** (`05-navigation-and-control/`)
   - Path planning algorithms
   - Robot control systems
   - SLAM implementation

6. **ROS2 Integration** (`08-ros2-integration/`)
   - ROS2 bridge setup
   - Topic communication
   - Service and action integration

## 🐳 Docker Setup for ROS2 on macOS

This project provides a complete Docker-based ROS2 setup specifically optimized for macOS (Intel and Apple Silicon). See `09-docker-setup/` for detailed instructions.

## 🛠️ Development Workflow

1. **Environment Setup**: Use scripts in `11-scripts/`
2. **Asset Management**: Store models in `12-assets/`
3. **Code Examples**: Reference `13-examples/`
4. **Documentation**: Check `10-documentation/`

## 📖 Resources

- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Isaac Sim Tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorials.html)

## 🤝 Contributing

Feel free to add your own examples, tutorials, and improvements to this learning environment.

## 📝 License

This project is for educational purposes. Please respect NVIDIA's licensing terms for Isaac Sim.
