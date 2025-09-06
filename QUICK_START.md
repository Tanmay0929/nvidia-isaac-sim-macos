# 🚀 Quick Start Guide

Welcome to your Isaac Sim learning environment! This guide will get you up and running quickly.

## ⚡ 5-Minute Setup

### 1. Prerequisites Check
Make sure you have:
- ✅ Docker Desktop installed and running
- ✅ NVIDIA Isaac Sim installed (from Omniverse)
- ✅ At least 8GB RAM allocated to Docker

### 2. Environment Setup
```bash
# Run the setup script
./11-scripts/setup-environment.sh

# Source your shell profile
source ~/.zshrc  # or ~/.bash_profile
```

### 3. Start Development Environment
```bash
# Start everything (Docker + Isaac Sim)
./11-scripts/start-development.sh
```

### 4. Verify Everything Works
```bash
# Check status
./11-scripts/check-status.sh

# Access ROS2 container
docker exec -it isaac-sim-ros2-dev bash
ros2 topic list
```

## 🎯 Your First Simulation

### 1. In Isaac Sim
1. Wait for Isaac Sim to fully load
2. Go to Window → Extensions
3. Enable "ROS2 Bridge" extension
4. Create a new scene (File → New)

### 2. Add Your First Robot
1. Right-click in viewport → Add → Cube
2. Select the cube
3. In Property Panel → Physics → Enable "Rigid Body"
4. Click Play button to see it fall!

### 3. Try the Example Code
```bash
# In Isaac Sim Script Editor, run:
exec(open('13-examples/basic-movement/simple_robot_movement.py').read())
```

## 📚 Learning Path

### Week 1: Basics
- [ ] Complete `01-getting-started/` tutorials
- [ ] Try examples in `13-examples/basic-movement/`
- [ ] Read `10-documentation/learning-roadmap.md`

### Week 2: Robot Models
- [ ] Import your first robot in `03-robot-models/`
- [ ] Learn about URDF files
- [ ] Practice robot control

### Week 3: Sensors
- [ ] Add cameras and LiDAR in `04-sensors-and-perception/`
- [ ] Process sensor data
- [ ] Try perception examples

### Week 4: ROS2 Integration
- [ ] Set up ROS2 bridge in `08-ros2-integration/`
- [ ] Control robots via ROS2 topics
- [ ] Build your first ROS2 node

## 🛠️ Useful Commands

### Quick Navigation
```bash
cd-isaac          # Go to workspace
cd-examples       # Go to examples
cd-tutorials      # Go to tutorials
cd-ros2          # Go to ROS2 integration
```

### Docker Commands
```bash
docker-start      # Start ROS2 container
docker-stop       # Stop ROS2 container
ros2-shell        # Access ROS2 container
ros2-topics       # List ROS2 topics
```

### Development Commands
```bash
isaac-start       # Start Isaac Sim
build-ros2        # Rebuild ROS2 image
clean-docker      # Clean up Docker
```

## 🐛 Troubleshooting

### Common Issues

**Docker won't start:**
```bash
# Check Docker Desktop is running
docker info

# Restart Docker Desktop
# Increase memory allocation in settings
```

**Isaac Sim won't connect to ROS2:**
```bash
# Check container is running
docker ps

# Restart bridge extension in Isaac Sim
# Check network settings
```

**Performance issues:**
```bash
# Check system resources
./11-scripts/check-status.sh

# Reduce viewport quality in Isaac Sim
# Close unnecessary applications
```

### Get Help
- Check `10-documentation/` for detailed guides
- Look at `13-examples/` for working code
- Join community forums for support

## 🎉 Next Steps

1. **Complete the basics** - Work through `01-getting-started/`
2. **Try examples** - Experiment with `13-examples/`
3. **Build projects** - Create your own in `07-projects/`
4. **Share your work** - Contribute to the community!

## 📖 Resources

- **Main README**: `README.md`
- **Learning Roadmap**: `10-documentation/learning-roadmap.md`
- **macOS Setup**: `10-documentation/macos-setup-guide.md`
- **ROS2 Integration**: `08-ros2-integration/README.md`

---

**Happy Learning! 🤖✨**

Remember: Take your time, experiment, and don't hesitate to ask for help. The robotics community is friendly and supportive!
