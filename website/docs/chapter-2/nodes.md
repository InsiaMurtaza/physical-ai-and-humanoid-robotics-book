---
sidebar_label: 'Node Lifecycle'
sidebar_position: 2
---

# Node Lifecycle in ROS 2

## Introduction to Node Lifecycle

In ROS 2, a node is a fundamental unit of computation that can perform various tasks, such as controlling hardware, processing sensor data, or implementing algorithms. Understanding the node lifecycle is crucial for creating robust and reliable robotic systems. The lifecycle of a node encompasses its creation, initialization, execution, and cleanup phases.

## Node Creation and Initialization

### Creating a Node

In ROS 2, nodes are typically implemented as classes that inherit from the `rclpy.Node` class (in Python) or `rclcpp::Node` (in C++). The node constructor is responsible for:

- Setting the node name
- Initializing the node's context
- Setting up parameter declarations
- Creating publishers, subscribers, services, and clients
- Creating timers and other callback objects

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('MinimalNode created')
```

### Node Parameters

Nodes can declare parameters during initialization, which can be configured at runtime:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('max_velocity', 1.0)

        # Access parameter values
        self.frequency = self.get_parameter('frequency').value
        self.max_velocity = self.get_parameter('max_velocity').value
```

## Node States and Transitions

ROS 2 nodes can exist in different states and transition between them based on system events or explicit commands. The lifecycle state machine allows for more sophisticated management of nodes, especially in complex robotic systems where nodes need to be brought up and down in a controlled manner.

### Standard Lifecycle States

1. **Unconfigured**: Initial state when the node is created but not yet configured
2. **Inactive**: Configured but not yet active, resources may be allocated
3. **Active**: Fully operational, executing main functionality
4. **Finalized**: Node is shut down and cannot be reactivated

### Lifecycle State Transitions

```
Unconfigured -> Inactive -> Active -> Inactive -> Unconfigured -> Finalized
     |           |         |         |          |           |
     +-----------+---------+---------+----------+-----------+
         [configure]   [activate] [deactivate] [cleanup] [shutdown]
```

## Lifecycle Node Implementation

To implement a lifecycle node in ROS 2, you can use the `LifecycleNode` class:

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleMinimalNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_minimal_node')
        self.get_logger().info('LifecycleMinimalNode created')

    def on_configure(self, state):
        self.get_logger().info('on_configure() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('on_activate() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('on_deactivate() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS
```

## Node Execution and Spin

### Main Loop and Callbacks

Unlike traditional programs with a main loop, ROS 2 nodes operate using a callback-based architecture:

```python
def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

    # The spin function keeps the node running and processes callbacks
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Timer-based Execution

Nodes often use timers to perform periodic tasks:

```python
class TimedNode(Node):
    def __init__(self):
        super().__init__('timed_node')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info('Timer callback: %d' % self.i)
        self.i += 1
```

## Resource Management

### Publisher and Subscriber Creation

Resources like publishers and subscribers should be created during node initialization:

```python
class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')

        # Create publisher
        self.publisher = self.create_publisher(String, 'topic', 10)

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Create service
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

## Best Practices for Node Lifecycle

### Proper Cleanup

Always ensure proper cleanup of resources:

```python
def destroy_node(self):
    # Clean up any custom resources
    if hasattr(self, 'custom_resource'):
        self.custom_resource.cleanup()

    # Call parent's destroy_node
    super().destroy_node()
```

### Error Handling

Implement error handling in lifecycle methods:

```python
def on_configure(self, state):
    try:
        # Attempt to initialize resources
        self.initialize_resources()
        return TransitionCallbackReturn.SUCCESS
    except Exception as e:
        self.get_logger().error(f'Failed to configure: {e}')
        return TransitionCallbackReturn.FAILURE
```

### Parameter Validation

Validate parameters during configuration:

```python
def on_configure(self, state):
    # Get and validate parameters
    frequency = self.get_parameter('frequency').value
    if frequency <= 0:
        self.get_logger().error('Invalid frequency parameter')
        return TransitionCallbackReturn.ERROR

    return TransitionCallbackReturn.SUCCESS
```

## Lifecycle Management Tools

ROS 2 provides command-line tools to manage lifecycle nodes:

```bash
# List lifecycle nodes
ros2 lifecycle list <node_name>

# Get current state
ros2 lifecycle get <node_name>

# Trigger state transitions
ros2 lifecycle configure <node_name>
ros2 lifecycle activate <node_name>
ros2 lifecycle deactivate <node_name>
ros2 lifecycle cleanup <node_name>
```

## References

<div class="reference-list">

- Lalancette, C., & Pérez, A. (2018). *Understanding ROS 2 lifecycle nodes*. ROSCon 2018.
- Quigley, M., Gerkey, B., & Smart, W. D. (2015). Programming robots with ROS: A practical introduction to the Robot Operating System. O'Reilly Media.
- ROS 2 Documentation. (2023). *Node Lifecycle Management*. Retrieved from https://docs.ros.org/

</div>