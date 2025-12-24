# ROS 2 Examples

This directory contains runnable examples for different ROS 2 communication patterns:

## Nodes Example
- `nodes/simple_node.py` - Basic node structure with parameters and timer

## Topics Examples
- `topics/publisher_example.py` - Publisher that sends String messages
- `topics/subscriber_example.py` - Subscriber that receives String messages

## Services Examples
- `services/service_server.py` - Service server that adds two integers
- `services/service_client.py` - Service client that calls the add service

## How to Run Examples

### 1. Publisher and Subscriber
Terminal 1 (run publisher):
```bash
python3 examples/topics/publisher_example.py
```

Terminal 2 (run subscriber):
```bash
python3 examples/topics/subscriber_example.py
```

### 2. Services
Terminal 1 (run service server):
```bash
python3 examples/services/service_server.py
```

Terminal 2 (run service client):
```bash
python3 examples/services/service_client.py 10 20
```

### 3. Simple Node
```bash
python3 examples/nodes/simple_node.py
```

## Setup Instructions for ROS 2 Humble Hawksbill and Foxy Fitzroy

### Installing ROS 2
For Ubuntu (Humble Hawksbill - ROS 2 version 22.04):
```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions

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

### Sourcing ROS 2 Environment
Before running the examples, source the ROS 2 environment:
```bash
# For Humble
source /opt/ros/humble/setup.bash

# For Foxy
source /opt/ros/foxy/setup.bash
```

### Python Dependencies
The examples require the following ROS 2 packages:
- `rclpy` - Python client library for ROS 2
- `std_msgs` - Standard message types
- `example_interfaces` - Example service definitions

These are typically installed with the desktop variant, but if needed separately:
```bash
# For Humble
sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-example-interfaces

# For Foxy
sudo apt install ros-foxy-rclpy ros-foxy-std-msgs ros-foxy-example-interfaces
```

### Alternative: Using ROS 2 Development Environment
If you're developing with these examples, you can create a workspace:
```bash
mkdir -p ~/ros2_examples_ws/src
cd ~/ros2_examples_ws
colcon build
source install/setup.bash
```

## Requirements
- ROS 2 Humble Hawksbill (recommended) or Foxy Fitzroy
- Python 3.8+ (Humble) or Python 3.6+ (Foxy)
- `rclpy` package
- `std_msgs` package
- `example_interfaces` package