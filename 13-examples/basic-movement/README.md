# Basic Movement Examples

This directory contains examples of basic robot movement in Isaac Sim, perfect for beginners learning robot simulation.

## 📁 Examples

### 1. Simple Robot Movement
- **File**: `simple_robot_movement.py`
- **Description**: Basic example of moving a robot using position commands
- **Skills**: Robot control, position commands, basic scripting

### 2. Joint Control
- **File**: `joint_control.py`
- **Description**: Control individual robot joints
- **Skills**: Joint manipulation, kinematics, robot control

### 3. Path Following
- **File**: `path_following.py`
- **Description**: Make a robot follow a predefined path
- **Skills**: Path planning, trajectory execution, waypoint navigation

### 4. Obstacle Avoidance
- **File**: `obstacle_avoidance.py`
- **Description**: Basic obstacle avoidance behavior
- **Skills**: Collision detection, navigation, reactive control

## 🚀 How to Run Examples

### Prerequisites
1. Isaac Sim installed and running
2. Basic Python knowledge
3. ROS2 container running (for ROS2 examples)

### Running Python Examples
```bash
# In Isaac Sim's Script Editor or terminal
cd /path/to/example
python simple_robot_movement.py
```

### Running ROS2 Examples
```bash
# Start ROS2 container first
cd 09-docker-setup
./start-ros2-container.sh

# In another terminal, run the example
docker exec -it isaac-sim-ros2-dev bash
cd /ros2_ws
source install/setup.bash
ros2 run your_package your_node
```

## 📚 Learning Path

1. **Start with**: `simple_robot_movement.py`
2. **Then try**: `joint_control.py`
3. **Advanced**: `path_following.py`
4. **Expert**: `obstacle_avoidance.py`

## 🛠️ Customization

Each example is designed to be easily customizable:
- Modify robot parameters
- Change movement patterns
- Add new behaviors
- Integrate with your own robots

## 📖 Next Steps

After mastering these examples:
- Move to `sensor-integration/` for sensor-based movement
- Explore `physics-simulation/` for advanced physics
- Check out `multi-robot/` for coordinated movement
