---
sidebar_label: 'rclpy Operations'
sidebar_position: 4
---

# rclpy Operations: Publishing, Subscribing, and Services

## Introduction

This section covers the core communication operations in rclpy: publishing messages to topics, subscribing to topics to receive messages, and implementing services for request-response communication. These operations form the foundation for connecting Python agents to ROS 2 systems.

## Publishing Messages

### Basic Publisher Implementation

Creating a publisher in rclpy involves using the `create_publisher()` method:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # Create publisher with topic name and queue size
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Timer to periodically publish messages
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.i += 1
```

### Advanced Publisher Features

#### Quality of Service (QoS) Configuration

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class QoSPublisherNode(Node):
    def __init__(self):
        super().__init__('qos_publisher_node')

        # Custom QoS profile for reliable communication
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(String, 'reliable_topic', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_reliable_message)

    def publish_reliable_message(self):
        msg = String()
        msg.data = 'Reliable message'
        self.publisher.publish(msg)
```

#### Publisher Status Monitoring

```python
class MonitoredPublisherNode(Node):
    def __init__(self):
        super().__init__('monitored_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.subscription_count_timer = self.create_timer(1.0, self.check_subscribers)

    def check_subscribers(self):
        subscription_count = self.publisher.get_subscription_count()
        if subscription_count > 0:
            self.get_logger().info(f'Publisher has {subscription_count} subscribers')
        else:
            self.get_logger().info('No subscribers connected')
```

## Subscribing to Topics

### Basic Subscriber Implementation

Creating a subscriber in rclpy involves using the `create_subscription()` method:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # Create subscription with topic name, callback, and queue size
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)

        # Prevent unused variable warning
        self.subscription  # type: ignore

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### Advanced Subscriber Features

#### Quality of Service (QoS) for Subscribers

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class QoSSubscriberNode(Node):
    def __init__(self):
        super().__init__('qos_subscriber_node')

        # Custom QoS profile matching the publisher
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.subscription = self.create_subscription(
            String,
            'reliable_topic',
            self.reliable_callback,
            qos_profile)

        self.subscription  # type: ignore

    def reliable_callback(self, msg):
        self.get_logger().info(f'Received reliable message: {msg.data}')
```

#### Message Filtering and Processing

```python
class FilteredSubscriberNode(Node):
    def __init__(self):
        super().__init__('filtered_subscriber')

        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.filtered_callback,
            10)

        self.subscription  # type: ignore

    def filtered_callback(self, msg):
        # Apply custom filtering logic
        if self.should_process_message(msg):
            self.process_message(msg)

    def should_process_message(self, msg):
        # Custom logic to determine if message should be processed
        return len(msg.data) > 0 and not msg.data.startswith('ignore')

    def process_message(self, msg):
        self.get_logger().info(f'Processing: {msg.data}')
```

## Service Implementation

### Basic Service Server

Creating a service server in rclpy involves using the `create_service()` method:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server')

        # Create service with service name and callback
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response
```

### Advanced Service Features

#### Service with Error Handling

```python
class RobustServiceServerNode(Node):
    def __init__(self):
        super().__init__('robust_service_server')
        self.srv = self.create_service(AddTwoInts, 'robust_add', self.robust_add_callback)

    def robust_add_callback(self, request, response):
        try:
            # Validate input parameters
            if request.a < 0 or request.b < 0:
                response.sum = 0
                self.get_logger().warn('Negative values detected, returning 0')
                return response

            # Perform the operation
            result = self.perform_calculation(request.a, request.b)
            response.sum = result

        except Exception as e:
            # Handle any errors gracefully
            self.get_logger().error(f'Service error: {e}')
            response.sum = 0

        return response

    def perform_calculation(self, a, b):
        # Custom calculation logic
        return a + b
```

## Service Client Implementation

### Basic Service Client

Creating a service client in rclpy involves using the `create_client()` method:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client')

        # Create client with service name
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b

        # Make asynchronous service call
        self.future = self.cli.call_async(self.req)
        return self.future
```

### Advanced Service Client Features

#### Asynchronous Service Calls with Callbacks

```python
class AsyncServiceClientNode(Node):
    def __init__(self):
        super().__init__('async_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_async_request(self, a, b):
        self.req.a = a
        self.req.b = b

        # Make asynchronous call
        future = self.cli.call_async(self.req)

        # Add callback to handle response
        future.add_done_callback(self.service_response_callback)

        return future

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
```

## Integration: Publisher-Subscriber-Service Combination

### Complete Example Node

Here's a node that combines all three communication patterns:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from example_interfaces.srv import SetBool, AddTwoInts
from example_interfaces.msg import Float64MultiArray

class CompleteIntegrationNode(Node):
    def __init__(self):
        super().__init__('complete_integration_node')

        # Publishers
        self.status_publisher = self.create_publisher(String, 'status', 10)
        self.result_publisher = self.create_publisher(Float64, 'result', 10)

        # Subscribers
        self.command_subscriber = self.create_subscription(
            String, 'command', self.command_callback, 10)
        self.input_subscriber = self.create_subscription(
            Float64MultiArray, 'input_data', self.input_callback, 10)

        # Services
        self.control_service = self.create_service(
            SetBool, 'enable_service', self.enable_callback)
        self.calculate_service = self.create_service(
            AddTwoInts, 'calculate_service', self.calculate_callback)

        # Clients (for calling external services)
        self.external_client = self.create_client(AddTwoInts, 'external_calculator')

        # Internal state
        self.enabled = True
        self.get_logger().info('Complete integration node initialized')

    def command_callback(self, msg):
        if msg.data == 'enable':
            self.enabled = True
            self.get_logger().info('Node enabled')
        elif msg.data == 'disable':
            self.enabled = False
            self.get_logger().info('Node disabled')

        # Publish status update
        status_msg = String()
        status_msg.data = f'Enabled: {self.enabled}'
        self.status_publisher.publish(status_msg)

    def input_callback(self, msg):
        if not self.enabled:
            return  # Ignore if disabled

        self.get_logger().info(f'Received input: {msg.data}')

        # Process input and publish result
        if len(msg.data) >= 2:
            result = msg.data[0] + msg.data[1]
            result_msg = Float64()
            result_msg.data = result
            self.result_publisher.publish(result_msg)

    def enable_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f'Service {"enabled" if self.enabled else "disabled"}'

        # Publish status
        status_msg = String()
        status_msg.data = response.message
        self.status_publisher.publish(status_msg)

        return response

    def calculate_callback(self, request, response):
        if not self.enabled:
            response.sum = 0
            return response

        response.sum = request.a + request.b
        self.get_logger().info(f'Calculated: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CompleteIntegrationNode()

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

## Python Agent Integration Patterns

### AI/ML Agent with ROS 2 Communication

Here's how to integrate AI/ML components with ROS 2:

```python
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

class AIControlNode(Node):
    def __init__(self):
        super().__init__('ai_control_node')

        # Publishers for robot control
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers for sensor data
        self.laser_subscriber = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.image_subscriber = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

        # Internal AI model (simplified example)
        self.ai_model = self.initialize_ai_model()

        # Timer for AI decision making
        self.ai_timer = self.create_timer(0.1, self.ai_decision_callback)

        # Internal state
        self.last_laser_data = None
        self.last_image_data = None
        self.get_logger().info('AI control node initialized')

    def initialize_ai_model(self):
        # Placeholder for actual AI model initialization
        # This could be a TensorFlow model, PyTorch model, etc.
        return {"weights": np.random.random(10)}

    def laser_callback(self, msg):
        # Convert ROS LaserScan to numpy array
        self.last_laser_data = np.array(msg.ranges)
        self.get_logger().debug(f'Received laser data with {len(msg.ranges)} points')

    def image_callback(self, msg):
        # Convert ROS Image to numpy array
        # This is a simplified example - actual conversion would be more complex
        height, width = msg.height, msg.width
        self.last_image_data = np.random.random((height, width, 3))  # Placeholder
        self.get_logger().debug(f'Received image: {width}x{height}')

    def ai_decision_callback(self):
        if self.last_laser_data is not None and self.last_image_data is not None:
            # Make AI-based control decision
            control_command = self.make_ai_decision(
                self.last_laser_data,
                self.last_image_data
            )

            # Publish control command
            cmd_msg = Twist()
            cmd_msg.linear.x = control_command['linear']
            cmd_msg.angular.z = control_command['angular']
            self.cmd_vel_publisher.publish(cmd_msg)

    def make_ai_decision(self, laser_data, image_data):
        # Simplified AI decision making
        # In a real application, this would use your trained model
        obstacle_distance = np.min(laser_data) if len(laser_data) > 0 else float('inf')

        if obstacle_distance < 0.5:  # Obstacle too close
            return {'linear': 0.0, 'angular': 0.5}  # Turn
        else:
            return {'linear': 0.5, 'angular': 0.0}  # Move forward

def main(args=None):
    rclpy.init(args=args)
    node = AIControlNode()

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

## Performance Considerations

### Efficient Message Handling

```python
class EfficientOperationsNode(Node):
    def __init__(self):
        super().__init__('efficient_operations')

        # Pre-allocate message objects to reduce allocation overhead
        self.string_msg = String()
        self.float_msg = Float64()

        # Publishers
        self.publisher = self.create_publisher(String, 'topic', 10)

        # Subscribers
        self.subscription = self.create_subscription(
            String, 'input', self.efficient_callback, 10)

        # Timer for periodic publishing
        self.timer = self.create_timer(0.01, self.efficient_publish)

    def efficient_callback(self, msg):
        # Process message efficiently
        processed_data = msg.data.upper()

        # Reuse message object
        self.string_msg.data = processed_data
        self.publisher.publish(self.string_msg)

    def efficient_publish(self):
        # Reuse message object for publishing
        self.string_msg.data = f'Efficient message at {self.get_clock().now().nanoseconds}'
        self.publisher.publish(self.string_msg)
```

## Error Handling and Diagnostics

### Comprehensive Error Handling

```python
class RobustOperationsNode(Node):
    def __init__(self):
        super().__init__('robust_operations')

        # Initialize with error handling
        try:
            self.publisher = self.create_publisher(String, 'topic', 10)
            self.subscription = self.create_subscription(
                String, 'input', self.safe_callback, 10)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize communications: {e}')
            raise

    def safe_callback(self, msg):
        try:
            # Validate message
            if not msg or not hasattr(msg, 'data'):
                self.get_logger().error('Invalid message received')
                return

            # Process message
            if len(msg.data) == 0:
                self.get_logger().warn('Empty message received')
                return

            # Publish result
            result_msg = String()
            result_msg.data = f'Processed: {msg.data}'
            self.publisher.publish(result_msg)

        except AttributeError as e:
            self.get_logger().error(f'Attribute error in callback: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in callback: {e}')
```

## Best Practices

1. **Always check service availability**: Use `wait_for_service()` before making service calls
2. **Use appropriate QoS settings**: Match publisher and subscriber QoS profiles
3. **Handle exceptions gracefully**: Prevent node crashes from bad messages or service calls
4. **Monitor connection status**: Check publisher subscriber counts and service availability
5. **Reuse message objects**: Reduce allocation overhead for high-frequency operations
6. **Validate inputs**: Check message contents before processing
7. **Log appropriately**: Use different log levels for different types of messages
8. **Separate concerns**: Keep communication logic separate from business logic

## References

<div class="reference-list">

- Open Robotics. (2023). *rclpy Communication Patterns Documentation*. Retrieved from https://docs.ros.org/
- Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming robots with ROS: A practical introduction to the Robot Operating System*. O'Reilly Media.
- Lütkebohle, I., et al. (2012). *ROS: From research to industry*. Robot Operating System (ROS), 1, 1-22.

</div>