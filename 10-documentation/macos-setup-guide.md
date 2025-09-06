# macOS Intel Setup Guide for Isaac Sim

Complete setup guide for running Isaac Sim with ROS2 on macOS Intel systems.

## 🖥️ System Requirements

### Hardware Requirements
- **CPU**: Intel Core i5 or better (multi-core recommended)
- **RAM**: 16GB minimum, 32GB recommended
- **GPU**: NVIDIA RTX 3060 or better (RTX 4070+ recommended)
- **Storage**: 100GB+ free space (SSD recommended)
- **Network**: Stable internet connection

### Software Requirements
- **macOS**: 10.15 (Catalina) or later
- **Docker Desktop**: Latest version
- **Python**: 3.8 or later
- **Git**: Latest version

## 📋 Installation Steps

### 1. Install NVIDIA Omniverse

#### Step 1: Download Omniverse Launcher
1. Go to [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
2. Click "Get Started" → "Download Omniverse Launcher"
3. Download the macOS version
4. Install the launcher

#### Step 2: Install Isaac Sim
1. Open Omniverse Launcher
2. Sign in with your NVIDIA account
3. Go to "Exchange" tab
4. Search for "Isaac Sim"
5. Click "Install" on Isaac Sim
6. Wait for installation to complete (this may take 30-60 minutes)

### 2. Install Docker Desktop

#### Step 1: Download Docker Desktop
1. Go to [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop/)
2. Download the Intel version (not Apple Silicon)
3. Install Docker Desktop

#### Step 2: Configure Docker Desktop
1. Open Docker Desktop
2. Go to Settings → Resources → Advanced
3. Set the following:
   - **Memory**: 8GB minimum (12GB recommended)
   - **CPUs**: 4 cores minimum (6+ recommended)
   - **Disk image size**: 100GB minimum
4. Click "Apply & Restart"

### 3. Install Development Tools

#### Install Homebrew (if not already installed)
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

#### Install Python and Git
```bash
brew install python@3.9 git
```

#### Install VS Code (recommended)
```bash
brew install --cask visual-studio-code
```

### 4. Clone This Repository
```bash
cd ~/dev
git clone <your-repo-url> nvidia-issac-sim
cd nvidia-issac-sim
```

## 🐳 ROS2 Docker Setup

### 1. Build ROS2 Development Image
```bash
cd 09-docker-setup
./build-ros2-image.sh
```

### 2. Start ROS2 Container
```bash
./start-ros2-container.sh
```

### 3. Verify Installation
```bash
# Check if container is running
docker ps

# Access the container
docker exec -it isaac-sim-ros2-dev bash

# Inside container, test ROS2
ros2 topic list
```

## 🚀 First Run

### 1. Start Isaac Sim
```bash
cd 09-docker-setup
./start-isaac-sim.sh
```

### 2. Enable ROS2 Bridge
1. In Isaac Sim, go to Window → Extensions
2. Search for "ROS2 Bridge"
3. Enable the extension
4. Configure connection settings:
   - Host: localhost
   - Port: 11311
   - Domain ID: 0

### 3. Test Basic Functionality
1. Create a new scene in Isaac Sim
2. Add a simple object (cube)
3. Enable physics
4. Run simulation
5. Check ROS2 topics in container

## 🔧 Configuration

### Isaac Sim Settings
1. **Graphics Settings**:
   - Go to Window → Settings
   - Set viewport quality to "High"
   - Enable hardware acceleration

2. **Physics Settings**:
   - Set physics timestep to 1/60 (0.0167)
   - Enable GPU acceleration if available

3. **Performance Settings**:
   - Reduce viewport quality if needed
   - Close unnecessary applications
   - Monitor system resources

### Docker Settings
1. **Resource Allocation**:
   - Increase memory if running out
   - Add more CPUs for better performance
   - Monitor disk usage

2. **Network Settings**:
   - Ensure host networking is enabled
   - Check firewall settings
   - Verify port accessibility

## 🐛 Troubleshooting

### Common Issues

#### Isaac Sim Won't Start
**Symptoms**: Application crashes or won't launch
**Solutions**:
- Check system requirements
- Update graphics drivers
- Restart Omniverse Launcher
- Clear cache and reinstall

#### Docker Container Issues
**Symptoms**: Container won't start or crashes
**Solutions**:
- Check Docker Desktop is running
- Increase allocated resources
- Restart Docker Desktop
- Check system resources

#### ROS2 Bridge Connection Failed
**Symptoms**: Can't connect to ROS2 from Isaac Sim
**Solutions**:
- Verify container is running
- Check network settings
- Restart bridge extension
- Check firewall configuration

#### Performance Issues
**Symptoms**: Slow simulation or lag
**Solutions**:
- Reduce viewport quality
- Lower physics timestep
- Close unnecessary applications
- Check system resources

### Debug Commands

#### Check System Resources
```bash
# Check memory usage
top -l 1 | grep PhysMem

# Check disk space
df -h

# Check GPU usage (if available)
nvidia-smi
```

#### Check Docker Status
```bash
# Check Docker status
docker info

# Check running containers
docker ps

# Check container logs
docker logs isaac-sim-ros2-dev
```

#### Check ROS2 Status
```bash
# Access container
docker exec -it isaac-sim-ros2-dev bash

# Check ROS2 topics
ros2 topic list

# Check ROS2 nodes
ros2 node list

# Monitor topic data
ros2 topic echo /isaac/cmd_vel
```

## 📚 Next Steps

After successful setup:
1. Complete the learning roadmap in `10-documentation/learning-roadmap.md`
2. Start with `01-getting-started/` tutorials
3. Explore `13-examples/` for code examples
4. Join the community forums for support

## 🔗 Useful Resources

### Official Documentation
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Docker Desktop Documentation](https://docs.docker.com/desktop/mac/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

### Community Support
- [Omniverse Community Forums](https://forums.developer.nvidia.com/c/agentes-autonomous-machines/isaac/17)
- [ROS2 Community](https://discourse.ros.org/)
- [Docker Community](https://forums.docker.com/)

### Hardware Requirements
- [NVIDIA GPU Requirements](https://www.nvidia.com/en-us/omniverse/system-requirements/)
- [macOS Compatibility](https://www.apple.com/macos/compatibility/)

## 📝 Notes

- This setup is optimized for Intel-based Macs
- Apple Silicon Macs may have different requirements
- Performance may vary based on hardware specifications
- Regular updates are recommended for best performance

Remember: If you encounter issues, don't hesitate to ask for help in the community forums or check the troubleshooting section above!
