---
sidebar_label: 'Writing Nodes in Python'
sidebar_position: 3
---

# Writing ROS 2 Nodes in Python with rclpy

## Basic Node Structure

Creating a ROS 2 node in Python using rclpy follows a standard pattern. The node class inherits from `rclpy.node.Node` and implements the required functionality:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Initialization and Setup

### Constructor Best Practices

The node constructor should handle all initialization in a specific order:

1. Call the parent constructor with the node name
2. Declare parameters with default values
3. Initialize communication objects (publishers, subscribers, services)
4. Create timers and other callback objects
5. Initialize internal state variables

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BestPracticeNode(Node):
    def __init__(self):
        # 1. Call parent constructor with node name
        super().__init__('best_practice_node')

        # 2. Declare parameters with defaults
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('topic_name', 'chatter')

        # Get parameter values
        self.frequency = self.get_parameter('frequency').value
        self.topic_name = self.get_parameter('topic_name').value

        # 3. Initialize communication objects
        self.publisher = self.create_publisher(String, self.topic_name, 10)

        # 4. Create timers and other objects
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)

        # 5. Initialize internal state
        self.counter = 0

        self.get_logger().info(
            f'Node initialized with frequency: {self.frequency}, topic: {self.topic_name}'
        )

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1
```

## Advanced Node Patterns

### Parameter Validation and Handling

Proper parameter handling is crucial for robust nodes:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with validation
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('min_distance', 0.1)
        self.declare_parameter('control_mode', 'velocity')

        # Validate parameters after declaration
        self.validate_and_set_parameters()

        # Set up parameter callback for dynamic changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def validate_and_set_parameters(self):
        max_vel = self.get_parameter('max_velocity').value
        min_dist = self.get_parameter('min_distance').value

        if max_vel <= 0:
            self.get_logger().error('Max velocity must be positive')
            self.max_velocity = 1.0  # Default value
        else:
            self.max_velocity = max_vel

        if min_dist <= 0:
            self.get_logger().error('Min distance must be positive')
            self.min_distance = 0.1  # Default value
        else:
            self.min_distance = min_dist

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.value <= 0:
                return SetParametersResult(successful=False, reason='Max velocity must be positive')
        return SetParametersResult(successful=True)
```

### Node with Multiple Communication Patterns

A node can implement multiple communication patterns:

```python
from std_msgs.msg import String, Float64
from example_interfaces.srv import SetBool
from example_interfaces.msg import Float64MultiArray

class MultiPatternNode(Node):
    def __init__(self):
        super().__init__('multi_pattern_node')

        # Publishers
        self.status_publisher = self.create_publisher(String, 'status', 10)
        self.data_publisher = self.create_publisher(Float64MultiArray, 'sensor_data', 10)

        # Subscribers
        self.command_subscriber = self.create_subscription(
            String, 'commands', self.command_callback, 10)
        self.control_subscriber = self.create_subscription(
            Float64, 'control_input', self.control_callback, 10)

        # Services
        self.enable_service = self.create_service(
            SetBool, 'enable_control', self.enable_callback)

        # Clients (for calling services from other nodes)
        self.action_client = self.create_client(
            SetBool, 'robot_action')

        # Timers
        self.status_timer = self.create_timer(1.0, self.status_timer_callback)

        self.enabled = True
        self.get_logger().info('Multi-pattern node initialized')

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        if msg.data == 'enable':
            self.enabled = True
        elif msg.data == 'disable':
            self.enabled = False

    def control_callback(self, msg):
        if self.enabled:
            # Process control input
            self.get_logger().info(f'Processing control: {msg.data}')
        else:
            self.get_logger().info('Control ignored - node disabled')

    def enable_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f'Control {"enabled" if self.enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response

    def status_timer_callback(self):
        status_msg = String()
        status_msg.data = f'Node status: {"enabled" if self.enabled else "disabled"}'
        self.status_publisher.publish(status_msg)
```

## Node Lifecycle Management

### Proper Resource Management

Always implement proper cleanup in your nodes:

```python
class ResourceManagedNode(Node):
    def __init__(self):
        super().__init__('resource_managed_node')

        # Initialize resources
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Custom resources that need cleanup
        self.custom_resource = SomeCustomResource()

        self.get_logger().info('Node with resource management initialized')

    def destroy_node(self):
        # Clean up custom resources before destroying the node
        if hasattr(self, 'custom_resource'):
            self.custom_resource.cleanup()

        # Call parent's destroy method
        super().destroy_node()
        self.get_logger().info('Node destroyed with proper cleanup')
```

### Lifecycle Nodes (Optional Advanced Pattern)

For more complex applications, you can use lifecycle nodes:

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleManagedNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_managed_node')
        self.get_logger().info('Lifecycle node created')

    def on_configure(self, state):
        self.get_logger().info('Configuring node')
        # Initialize resources
        self.publisher = self.create_publisher(String, 'topic', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating node')
        # Activate resources
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating node')
        # Deactivate resources
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up node')
        # Clean up resources
        return TransitionCallbackReturn.SUCCESS
```

## Error Handling and Robustness

### Exception Handling in Callbacks

Always handle exceptions in callbacks to prevent node crashes:

```python
class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        self.subscription = self.create_subscription(
            String, 'input', self.safe_callback, 10)

    def safe_callback(self, msg):
        try:
            # Process message
            result = self.process_message(msg)
            self.publish_result(result)
        except ValueError as e:
            self.get_logger().error(f'Invalid message format: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in callback: {e}')
```

### Node Health Monitoring

Implement health checks and monitoring:

```python
class HealthMonitoredNode(Node):
    def __init__(self):
        super().__init__('health_monitored_node')

        # Health monitoring
        self.last_callback_time = self.get_clock().now()
        self.health_timer = self.create_timer(5.0, self.health_check)

        # Main functionality
        self.subscription = self.create_subscription(
            String, 'input', self.update_health, 10)

    def update_health(self, msg):
        self.last_callback_time = self.get_clock().now()
        # Process message

    def health_check(self):
        current_time = self.get_clock().now()
        time_since_last = (current_time - self.last_callback_time).nanoseconds / 1e9

        if time_since_last > 10.0:  # 10 seconds without message
            self.get_logger().warn('Node may be unhealthy - no messages received recently')
```

## Testing Nodes

### Basic Node Testing

Structure your nodes to be easily testable:

```python
class TestableNode(Node):
    def __init__(self, node_name='testable_node'):
        super().__init__(node_name)
        self.publisher = self.create_publisher(String, 'output', 10)
        self.received_messages = []  # For testing purposes

    def process_and_publish(self, input_data):
        # Process the input
        processed = self.process_data(input_data)
        # Publish the result
        msg = String()
        msg.data = processed
        self.publisher.publish(msg)
        return processed

    def process_data(self, data):
        # Separate processing logic for easy testing
        return f"Processed: {data}"
```

## Performance Optimization

### Efficient Message Handling

For high-frequency applications:

```python
class EfficientNode(Node):
    def __init__(self):
        super().__init__('efficient_node')

        # Pre-allocate message objects to reduce allocation overhead
        self.msg_cache = String()

        self.publisher = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(
            String, 'input', self.efficient_callback, 10)

    def efficient_callback(self, msg):
        # Reuse message object
        self.msg_cache.data = f'Echo: {msg.data}'
        self.publisher.publish(self.msg_cache)
```

## Best Practices Summary

1. **Initialize in the correct order**: Parameters → Communication → State
2. **Always implement proper cleanup**: Override `destroy_node()` if needed
3. **Handle exceptions in callbacks**: Prevent node crashes from bad messages
4. **Use appropriate QoS settings**: Match your application requirements
5. **Validate parameters**: Check bounds and validity during initialization
6. **Separate concerns**: Keep processing logic separate from ROS communication
7. **Log appropriately**: Use different log levels for different types of messages
8. **Test thoroughly**: Design nodes to be easily testable

## References

<div class="reference-list">

- Open Robotics. (2023). *rclpy API Documentation*. Retrieved from https://docs.ros.org/
- Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming robots with ROS: A practical introduction to the Robot Operating System*. O'Reilly Media.
- Siciliano, B., & Khatib, O. (2016). *Springer handbook of robotics*. Springer Publishing Company.

</div>