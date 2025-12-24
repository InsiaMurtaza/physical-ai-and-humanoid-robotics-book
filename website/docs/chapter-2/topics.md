---
sidebar_label: 'Publisher/Subscriber Patterns'
sidebar_position: 3
---

# Publisher/Subscriber Communication Patterns in ROS 2

## Introduction to Topics and Messages

In ROS 2, topics form the backbone of the publish-subscribe communication model. Topics enable asynchronous, decoupled communication between nodes, allowing for flexible and scalable robotic systems. This pattern is particularly useful for streaming data such as sensor readings, robot states, or sensor fusion outputs.

### Core Concepts

- **Topics**: Named buses over which nodes exchange messages
- **Publishers**: Nodes that send messages to topics
- **Subscribers**: Nodes that receive messages from topics
- **Messages**: Data structures that define the format of information exchanged

The publish-subscribe pattern provides temporal and spatial decoupling, meaning publishers and subscribers don't need to exist simultaneously or be aware of each other's existence.

## Creating Publishers and Subscribers

### Publisher Implementation

To create a publisher in ROS 2, you use the `create_publisher()` method of a node. Here's a basic example:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Implementation

Subscribers are created using the `create_subscription()` method:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) in Publisher/Subscriber Communication

QoS profiles allow you to configure the behavior of publishers and subscribers for different communication requirements:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Example of configuring QoS for a publisher
qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
    history=QoSHistoryPolicy.KEEP_LAST,        # or KEEP_ALL
)

publisher = self.create_publisher(String, 'topic', qos_profile)
```

### Common QoS Settings

- **Reliability**: Ensures message delivery (RELIABLE) or best-effort delivery (BEST_EFFORT)
- **History**: Controls how many messages to store (KEEP_LAST or KEEP_ALL)
- **Depth**: Number of messages to store in the history
- **Durability**: Whether messages persist for late-joining subscribers

## Advanced Publisher/Subscriber Patterns

### Multiple Publishers and Subscribers

ROS 2 supports multiple publishers and subscribers on the same topic, enabling fan-out and fan-in patterns:

```python
class MultiTopicNode(Node):
    def __init__(self):
        super().__init__('multi_topic_node')

        # Multiple publishers on different topics
        self.pub1 = self.create_publisher(String, 'topic1', 10)
        self.pub2 = self.create_publisher(String, 'topic2', 10)

        # Multiple subscribers to different topics
        self.sub1 = self.create_subscription(String, 'topic1', self.callback1, 10)
        self.sub2 = self.create_subscription(String, 'topic2', self.callback2, 10)
```

### Message Filtering and Type Adaptation

You can implement custom filtering logic in subscriber callbacks:

```python
def filtered_callback(self, msg):
    # Apply custom filtering logic
    if self.should_process_message(msg):
        self.process_message(msg)

def should_process_message(self, msg):
    # Custom logic to determine if message should be processed
    return len(msg.data) > 0
```

## Message Types and Custom Messages

### Using Standard Message Types

ROS 2 provides standard message types in packages like `std_msgs`, `geometry_msgs`, and `sensor_msgs`:

```python
from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, Image
```

### Creating Custom Messages

Custom messages are defined in `.msg` files and can be used in publishers and subscribers:

```python
# Assuming you have a custom message RobotStatus
from my_robot_msgs.msg import RobotStatus

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)

    def publish_status(self, status_code, battery_level):
        msg = RobotStatus()
        msg.status_code = status_code
        msg.battery_level = battery_level
        self.publisher.publish(msg)
```

## Best Practices for Publisher/Subscriber Implementation

### Publisher Best Practices

1. **Use appropriate QoS settings**: Match QoS policies to your application requirements
2. **Manage message frequency**: Don't publish more frequently than necessary
3. **Handle publisher lifecycle**: Ensure publishers are properly created and destroyed
4. **Use message pools**: For high-frequency publishing, consider using message pools to reduce allocation overhead

### Subscriber Best Practices

1. **Process messages efficiently**: Keep callback functions lightweight
2. **Handle message timestamps**: Use message timestamps for synchronization
3. **Implement proper error handling**: Handle malformed or unexpected messages gracefully
4. **Use callback groups**: For complex nodes with multiple subscribers, organize callbacks into groups

## Publisher/Subscriber Synchronization

### Time-based Synchronization

For applications requiring synchronization across multiple topics:

```python
from rclpy.qos import QoSProfile
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Synchronize messages from multiple topics based on timestamps
def sync_callback(msg1, msg2):
    # Process synchronized messages
    pass

# Note: This requires additional message_filters package
```

### Publisher Readiness Checks

Check if subscribers are available before publishing:

```python
def check_publishers(self):
    if self.publisher.get_subscription_count() > 0:
        # Safe to publish
        self.publish_data()
    else:
        self.get_logger().info('No subscribers yet, waiting...')
```

## Performance Considerations

### Memory Management

- **Message allocation**: Consider reusing message objects for high-frequency publishing
- **Queue sizes**: Balance between memory usage and message buffering needs
- **Callback execution**: Keep subscriber callbacks short to avoid blocking

### Network Efficiency

- **Message size**: Optimize message structures to minimize network overhead
- **Publishing frequency**: Match publishing rate to actual data update rate
- **Connection handling**: Consider the impact of multiple connections on performance

## Error Handling and Diagnostics

### Publisher Error Handling

```python
def robust_publish(self, msg):
    try:
        if self.publisher.get_subscription_count() > 0:
            self.publisher.publish(msg)
    except Exception as e:
        self.get_logger().error(f'Failed to publish message: {e}')
```

### Subscriber Error Handling

```python
def safe_callback(self, msg):
    try:
        # Process message
        self.process_message(msg)
    except Exception as e:
        self.get_logger().error(f'Error processing message: {e}')
        # Optionally publish error status
```

## Real-World Use Cases

### Sensor Data Streaming

Publishing sensor data from multiple sources:

```python
class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.laser_pub = self.create_publisher(LaserScan, 'laser_scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        self.camera_pub = self.create_publisher(Image, 'camera/image_raw', 10)

        # Timer to periodically read and publish sensor data
        self.timer = self.create_timer(0.1, self.read_sensors)

    def read_sensors(self):
        # Read actual sensor data and publish
        pass
```

### Robot State Broadcasting

Broadcasting robot state information:

```python
class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.state_pub = self.create_publisher(RobotState, 'robot_state', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Publish state at fixed rate
        self.timer = self.create_timer(0.05, self.publish_robot_state)
```

## Testing Publisher/Subscriber Systems

### Unit Testing

```python
import unittest
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

class TestPublisherSubscriber(unittest.TestCase):
    def test_message_flow(self):
        # Create publisher and subscriber nodes
        pub_node = MinimalPublisher()
        sub_node = MinimalSubscriber()

        # Execute a few cycles to ensure message flow
        executor = SingleThreadedExecutor()
        executor.add_node(pub_node)
        executor.add_node(sub_node)

        # Run for a short time to allow message exchange
        executor.spin_once(timeout_sec=1.0)
```

## References

<div class="reference-list">

- Lütkebohle, I., et al. (2012). *ROS: From research to industry*. Robot Operating System (ROS), 1, 1-22.
- Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming robots with ROS: A practical introduction to the Robot Operating System*. O'Reilly Media.
- Siciliano, B., & Khatib, O. (2016). *Springer handbook of robotics*. Springer Publishing Company.

</div>