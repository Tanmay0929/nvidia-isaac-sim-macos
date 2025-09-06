# 🤖 Isaac Sim Robot Visualization Guide

This guide will help you visualize and interact with your warehouse robot in NVIDIA Isaac Sim.

## 🚀 Quick Start

### 1. **Setup Isaac Sim**
```bash
cd /Users/tpancholi/dev/nvidia-issac-sim/07-projects/autonomous-warehouse-robot
python scripts/setup_isaac_sim.py
```

### 2. **Launch Isaac Sim**
```bash
./scripts/launch_isaac_sim.sh
```

### 3. **Run Robot Visualization**
1. In Isaac Sim, go to **Window → Script Editor**
2. Open: `scripts/isaac_sim_visualization.py`
3. Click **"Run"** to execute the script

## 🎯 What You'll See

The visualization script will create:

### **Warehouse Environment**
- ✅ **Ground plane** (10m × 10m)
- ✅ **Warehouse shelves** with packages
- ✅ **Proper lighting** for realistic visualization
- ✅ **Physics simulation** enabled

### **Warehouse Robot**
- ✅ **Robot base** with differential drive
- ✅ **LiDAR sensor** (red cylinder)
- ✅ **Camera sensor** (white cube)
- ✅ **6-DOF robot arm** with gripper
- ✅ **Wheels** for movement

### **Demo Sequence**
The robot will automatically:
1. **Start** at origin (0, 0, 0)
2. **Move forward** to (2, 0, 0)
3. **Turn right** and move to (2, 2, 0)
4. **Turn around** and move to (0, 2, 0)
5. **Return** to start position

## 🎮 Interactive Controls

### **Camera Controls**
- **Orbit**: Middle mouse button + drag
- **Pan**: Shift + middle mouse button + drag
- **Zoom**: Mouse wheel or Alt + middle mouse button + drag

### **Timeline Controls**
- **Play/Pause**: Space bar
- **Step Forward/Back**: Arrow keys
- **Reset**: R key

### **View Controls**
- **Focus on Robot**: Select robot and press 'F'
- **Frame All**: Press 'A' to see entire scene
- **Wireframe**: Press 'W' to toggle wireframe mode

## 🛠️ Customization

### **Modify Robot Movement**
Edit `isaac_sim_visualization.py` and change the demo positions:

```python
demo_positions = [
    (0.0, 0.0, 0.0),      # Start
    (2.0, 0.0, 0.0),      # Move forward
    (2.0, 2.0, 1.57),     # Turn right and move
    (0.0, 2.0, 3.14),     # Turn around and move back
    (0.0, 0.0, 4.71),     # Return to start
]
```

### **Add More Objects**
Add more warehouse elements:

```python
def create_additional_objects(self):
    # Create more shelves, packages, or obstacles
    pass
```

### **Change Robot Appearance**
Modify robot colors and materials:

```python
# Add material properties to robot parts
# Change colors, textures, or lighting
```

## 📊 Performance Tips

### **For Better Performance**
- **Reduce viewport quality** if simulation is slow
- **Close unnecessary applications**
- **Use lower resolution** for initial testing
- **Disable physics** if not needed

### **For Better Visualization**
- **Increase viewport quality** for better graphics
- **Enable shadows** for realistic lighting
- **Use high-resolution textures**
- **Enable anti-aliasing**

## 🔧 Troubleshooting

### **Common Issues**

#### **Isaac Sim Won't Start**
- Check if Isaac Sim is properly installed
- Verify system requirements
- Restart Omniverse Launcher
- Check for updates

#### **Script Won't Run**
- Ensure Isaac Sim is fully loaded
- Check Script Editor is open
- Verify file path is correct
- Check for Python errors in console

#### **Robot Not Visible**
- Check if script ran successfully
- Look for error messages in console
- Verify robot is created at correct path
- Try resetting the scene

#### **Performance Issues**
- Reduce viewport quality
- Close other applications
- Check system resources
- Restart Isaac Sim

### **Debug Steps**
1. **Check Console**: Look for error messages
2. **Verify Paths**: Ensure all file paths are correct
3. **Test Simple**: Try creating just a basic cube first
4. **Check Permissions**: Ensure files are readable

## 📚 Next Steps

### **Explore Further**
- **Modify the robot model** in the URDF file
- **Add more sensors** (cameras, IMU, etc.)
- **Create custom environments**
- **Add physics interactions**

### **Advanced Features**
- **ROS2 integration** for real-time control
- **Custom materials** and textures
- **Animation sequences**
- **Multi-robot scenarios**

### **Learning Resources**
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Isaac Sim Tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorials.html)
- [Omniverse Community](https://forums.developer.nvidia.com/c/agentes-autonomous-machines/isaac/17)

## 🎉 Success!

If everything works correctly, you should see:
- ✅ A warehouse environment with shelves and packages
- ✅ Your robot moving through the environment
- ✅ Smooth camera controls and interaction
- ✅ Realistic physics simulation

**Congratulations! You've successfully visualized your robot in Isaac Sim!** 🚀

## 📞 Need Help?

If you encounter issues:
1. Check the troubleshooting section above
2. Look at the Isaac Sim documentation
3. Join the Omniverse community forums
4. Check the GitHub repository for updates

---

**Happy Robot Visualization!** 🤖✨
