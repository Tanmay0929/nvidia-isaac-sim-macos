# Getting Started with Isaac Sim

Welcome to your Isaac Sim learning journey! This section covers the basics of setting up and using Isaac Sim for robot simulations.

## 📋 Prerequisites

- NVIDIA Isaac Sim installed (latest version)
- Basic understanding of 3D concepts
- Python 3.8+ knowledge (helpful but not required)

## 🎯 Learning Objectives

By the end of this section, you will:
- Understand Isaac Sim's interface and workflow
- Create your first simulation environment
- Add basic objects and physics
- Run your first simulation

## 📚 Tutorials

### 1. First Steps
- [Installation and Setup](installation-setup.md)
- [Interface Overview](interface-overview.md)
- [Creating Your First Scene](first-scene.md)

### 2. Basic Concepts
- [Understanding the Stage](understanding-stage.md)
- [Working with Prims](working-with-prims.md)
- [Physics Simulation](physics-simulation.md)

### 3. Essential Tools
- [Viewport Navigation](viewport-navigation.md)
- [Timeline and Playback](timeline-playback.md)
- [Property Panel](property-panel.md)

## 🚀 Quick Start Guide

### Step 1: Launch Isaac Sim
1. Open NVIDIA Omniverse Launcher
2. Click on Isaac Sim
3. Wait for the application to load

### Step 2: Create a New Scene
1. Go to File → New
2. Choose "Empty Stage"
3. Save your scene (Ctrl+S)

### Step 3: Add Your First Object
1. Right-click in the viewport
2. Select "Add" → "Cube"
3. Notice the cube appears in the viewport

### Step 4: Enable Physics
1. Select the cube
2. In the Property Panel, find "Physics" section
3. Check "Rigid Body"
4. Set "Collision" to "Convex Hull"

### Step 5: Run Simulation
1. Click the "Play" button in the timeline
2. Watch your cube fall due to gravity!

## 🛠️ Common Tasks

### Adding Objects
- **Primitives**: Right-click → Add → [Shape]
- **From USD**: File → Import → [Select USD file]
- **From Library**: Window → Content Browser

### Setting Up Physics
- Select object → Property Panel → Physics
- Enable "Rigid Body" for dynamic objects
- Set "Collision" shape (Box, Sphere, Convex Hull, etc.)

### Camera Controls
- **Orbit**: Middle mouse button + drag
- **Pan**: Shift + middle mouse button + drag
- **Zoom**: Mouse wheel or Alt + middle mouse button + drag

## 📖 Next Steps

After completing this section:
1. Move to `02-basic-simulations/` for more complex scenarios
2. Explore `03-robot-models/` to learn about robot integration
3. Check out `13-examples/` for code examples

## 🔗 Useful Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Isaac Sim Tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorials.html)
- [Omniverse Community](https://forums.developer.nvidia.com/c/agentes-autonomous-machines/isaac/17)

## ❓ Troubleshooting

### Common Issues

**Isaac Sim won't start:**
- Check system requirements
- Update graphics drivers
- Restart Omniverse Launcher

**Objects not falling:**
- Ensure physics is enabled
- Check gravity settings in Physics → Scene
- Verify rigid body is enabled on objects

**Performance issues:**
- Reduce viewport quality
- Lower physics timestep
- Close unnecessary applications
