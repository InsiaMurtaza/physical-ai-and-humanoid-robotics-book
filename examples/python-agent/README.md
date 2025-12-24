# Python Agent for Robot Control

This example demonstrates a Python-based AI agent that controls a simulated robot through ROS 2 communication patterns. The agent processes sensor data and makes intelligent decisions to control robot movement, bridging Python AI/ML applications with ROS 2 robotic systems.

## Requirements

- ROS 2 Humble Hawksbill (recommended) or Foxy Fitzroy
- Python 3.8+ (Humble) or Python 3.6+ (Foxy)
- NumPy: `pip install numpy` or `sudo apt install python3-numpy`
- Standard ROS 2 packages: `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`

## Installation and Setup

### 1. Install ROS 2
For Ubuntu (Humble Hawksbill - ROS 2 version 22.04):
```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```

For Foxy Fitzroy (ROS 2 version 20.04):
```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-foxy-desktop
sudo apt install python3-colcon-common-extensions
```

### 2. Install Python Dependencies
```bash
pip3 install numpy
```

### 3. Source ROS 2 Environment
Before running the example, source the ROS 2 environment:
```bash
# For Humble
source /opt/ros/humble/setup.bash

# For Foxy
source /opt/ros/foxy/setup.bash
```

## How to Run the Python Agent

### 1. Terminal 1 - Launch a Robot Simulation
You'll need a ROS 2 robot simulation that provides:
- `/scan` topic with LaserScan messages
- `/odom` topic with Odometry messages
- `/cmd_vel` topic for velocity commands

Example with TurtleBot3 simulation:
```bash
# Install TurtleBot3 packages first
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo

# Set simulation environment
export TURTLEBOT3_MODEL=burger

# Launch Gazebo simulation
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 2. Terminal 2 - Run the Python Agent
```bash
python3 examples/python-agent/python_agent.py
```

### 3. Monitor the Agent's Behavior
In another terminal, monitor the agent's published topics:
```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist

# Monitor agent status
ros2 topic echo /agent_status std_msgs/msg/String

# Monitor distance traveled
ros2 topic echo /agent_distance std_msgs/msg/Float64
```

## Expected Behavior

The Python agent will:
1. Connect to sensor topics (`/scan`, `/odom`)
2. Process sensor data to understand the environment
3. Make navigation decisions to reach the target (5.0, 5.0)
4. Avoid obstacles while moving toward the goal
5. Stop when the target is reached
6. Publish status and metrics to monitoring topics

## Troubleshooting

### Common Issues

1. **ModuleNotFoundError: No module named 'rclpy'**
   - Ensure ROS 2 is properly installed and sourced
   - Run: `source /opt/ros/humble/setup.bash` (or foxy)

2. **No sensor data received**
   - Verify that a robot simulation is running
   - Check available topics: `ros2 topic list`
   - Look for `/scan`, `/odom`, and `/cmd_vel` topics

3. **Robot not responding to commands**
   - Verify that a robot controller is subscribed to `/cmd_vel`
   - Check that the simulation environment is properly configured

4. **Python dependencies missing**
   - Install numpy: `pip3 install numpy`

### Verifying Setup

To verify your setup is working correctly:
```bash
# Check that rclpy is available
python3 -c "import rclpy; print('rclpy available')"

# Check for required message types
python3 -c "from sensor_msgs.msg import LaserScan; print('LaserScan available')"

# List ROS 2 nodes after starting the agent
ros2 node list
```

## Customization

You can modify the agent behavior by changing parameters in the PythonRobotAgent class:
- `self.linear_speed` and `self.angular_speed`: Control movement speed
- `self.safe_distance`: Minimum distance to obstacles
- `self.target_position`: Destination coordinates
- `self.arrival_threshold`: Distance to target for arrival detection

## Integration with AI/ML

This agent demonstrates the pattern for integrating AI/ML models with ROS 2:
- Sensor data is processed in the callback methods
- The `make_ai_decision()` method can be replaced with ML inference
- Control commands are published based on AI decisions
- Performance metrics can be used for learning algorithms

## Files in This Directory

- `python_agent.py`: Main Python agent implementation
- `README.md`: This file with setup and usage instructions