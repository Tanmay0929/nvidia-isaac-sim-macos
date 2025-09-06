# ROS2 Docker Setup for macOS Intel

This guide will help you set up ROS2 using Docker on your macOS Intel system for Isaac Sim integration.

## 🐳 Prerequisites

- Docker Desktop installed and running
- At least 8GB RAM allocated to Docker
- 50GB+ free disk space

## 📋 Setup Steps

### 1. Docker Desktop Configuration

1. Open Docker Desktop
2. Go to Settings → Resources → Advanced
3. Set Memory to at least 8GB
4. Set CPUs to at least 4 cores
5. Apply & Restart

### 2. Create ROS2 Development Environment

We'll create a custom Docker setup optimized for Isaac Sim integration.

### 3. Network Configuration

For Isaac Sim to communicate with ROS2, we need proper network setup.

## 🚀 Quick Start

```bash
# Navigate to docker setup directory
cd 09-docker-setup

# Build the ROS2 development image
./build-ros2-image.sh

# Start the ROS2 development container
./start-ros2-container.sh

# In another terminal, start Isaac Sim
./start-isaac-sim.sh
```

## 📁 Files in this Directory

- `Dockerfile` - ROS2 development environment
- `docker-compose.yml` - Multi-container setup
- `build-ros2-image.sh` - Build script
- `start-ros2-container.sh` - Container startup script
- `start-isaac-sim.sh` - Isaac Sim startup script
- `ros2-workspace/` - Your ROS2 workspace
- `config/` - Configuration files

## 🔧 Troubleshooting

### Common Issues

1. **Permission Denied**: Make sure Docker Desktop is running
2. **Network Issues**: Check Docker network configuration
3. **Performance**: Increase Docker resources in settings

### Useful Commands

```bash
# Check running containers
docker ps

# View container logs
docker logs <container-name>

# Access container shell
docker exec -it <container-name> /bin/bash

# Stop all containers
docker-compose down
```

## 📚 Next Steps

After setup, proceed to:
1. `08-ros2-integration/` - ROS2 bridge configuration
2. `13-examples/` - Example projects
3. `14-tutorials/ros2-bridge/` - Integration tutorials
