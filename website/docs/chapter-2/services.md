---
sidebar_label: 'Service/Client Patterns'
sidebar_position: 4
---

# Service/Client Communication Patterns in ROS 2

## Introduction to Services and Clients

Service/Client communication in ROS 2 implements a synchronous request-response pattern, which is fundamentally different from the asynchronous publish-subscribe model. Services are ideal for operations that require a direct response, such as configuration changes, computation requests, or triggering specific actions that return results.

### Core Concepts

- **Services**: Provide synchronous request-response communication
- **Service Servers**: Nodes that offer specific functionality and respond to requests
- **Service Clients**: Nodes that request specific functionality and wait for responses
- **Service Definitions**: `.srv` files that define request and response message structures

The service pattern is appropriate when:
- You need guaranteed delivery and response
- The operation has a clear start and end
- You require specific results from the operation
- The interaction is more transactional than streaming

## Creating Service Servers and Clients

### Service Definition Files

Services are defined using `.srv` files that specify both request and response message structures. The format separates request and response with three dashes (`---`):

```
# Request message fields
string name
int32 value
---
# Response message fields
bool success
string message
int32 result
```

Example service definition (`AddTwoInts.srv`):
```
int64 a
int64 b
---
int64 sum
```

### Service Server Implementation

To create a service server in ROS 2, you use the `create_service()` method:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Implementation

Service clients are created using the `create_client()` method:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()

    # Send request
    future = minimal_client.send_request(1, 2)

    # Wait for response
    rclpy.spin_until_future_complete(minimal_client, future)

    if future.result() is not None:
        response = future.result()
        minimal_client.get_logger().info(
            'Result of add_two_ints: %d' % response.sum)
    else:
        minimal_client.get_logger().info('Service call failed')

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Service Patterns

### Asynchronous Service Clients

For non-blocking service calls, you can use callbacks:

```python
class AsyncClient(Node):
    def __init__(self):
        super().__init__('async_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_async_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service response: %d' % response.sum)
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))
```

### Service with Custom Message Types

Creating services with custom message types:

```python
# Assuming you have a custom service definition in your package
from my_robot_msgs.srv import MoveRobot

class RobotService(Node):
    def __init__(self):
        super().__init__('robot_service')
        self.srv = self.create_service(MoveRobot, 'move_robot', self.move_robot_callback)

    def move_robot_callback(self, request, response):
        # Process the move request
        try:
            # Perform robot movement logic
            success = self.execute_movement(request.target_pose)
            response.success = success
            response.message = "Movement completed" if success else "Movement failed"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"

        return response
```

## Quality of Service (QoS) for Services

While services don't use the same QoS profiles as topics, they do have timeout and reliability considerations:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

# Configure timeout for service calls
def call_with_timeout(self, request, timeout_sec=5.0):
    future = self.cli.call_async(request)
    rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
    return future
```

## Service Error Handling and Diagnostics

### Server-Side Error Handling

```python
def robust_service_callback(self, request, response):
    try:
        # Validate request parameters
        if request.a < 0 or request.b < 0:
            response.success = False
            response.message = "Negative values not allowed"
            return response

        # Perform the operation
        result = self.perform_operation(request)
        response.success = True
        response.result = result

    except ValueError as e:
        response.success = False
        response.message = f"Invalid input: {str(e)}"
    except Exception as e:
        response.success = False
        response.message = f"Internal error: {str(e)}"
        self.get_logger().error(f'Service error: {e}')

    return response
```

### Client-Side Error Handling

```python
def robust_service_call(self, request):
    try:
        # Check if service is available
        if not self.cli.service_is_ready():
            self.get_logger().warn('Service not ready, waiting...')
            if not self.cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError('Service not available')

        # Make the call
        future = self.cli.call_async(request)

        # Wait for response with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            return future.result()
        else:
            raise RuntimeError('Service call returned None')

    except Exception as e:
        self.get_logger().error(f'Service call failed: {e}')
        raise
```

## Service Composition and Orchestration

### Multiple Services in a Single Node

A node can provide multiple services:

```python
class MultiServiceNode(Node):
    def __init__(self):
        super().__init__('multi_service_node')

        # Multiple service servers
        self.config_srv = self.create_service(SetConfig, 'set_config', self.config_callback)
        self.move_srv = self.create_service(MoveRobot, 'move_robot', self.move_callback)
        self.status_srv = self.create_service(GetStatus, 'get_status', self.status_callback)

    def config_callback(self, request, response):
        # Handle configuration requests
        pass

    def move_callback(self, request, response):
        # Handle movement requests
        pass

    def status_callback(self, request, response):
        # Handle status requests
        pass
```

### Service Chaining

Chaining multiple services together:

```python
class ServiceOrchestrator(Node):
    def __init__(self):
        super().__init__('service_orchestrator')

        # Client to other services
        self.config_client = self.create_client(SetConfig, 'set_config')
        self.move_client = self.create_client(MoveRobot, 'move_robot')

        # Our service
        self.orchestrate_srv = self.create_service(Orchestrate, 'orchestrate', self.orchestrate_callback)

    def orchestrate_callback(self, request, response):
        # Call multiple services in sequence
        try:
            # Step 1: Configure robot
            config_req = SetConfig.Request(config=request.config)
            config_future = self.config_client.call_async(config_req)
            rclpy.spin_until_future_complete(self, config_future)

            if not config_future.result().success:
                response.success = False
                response.message = "Configuration failed"
                return response

            # Step 2: Move robot
            move_req = MoveRobot.Request(target=request.target)
            move_future = self.move_client.call_async(move_req)
            rclpy.spin_until_future_complete(self, move_future)

            response.success = move_future.result().success
            response.message = move_future.result().message

        except Exception as e:
            response.success = False
            response.message = f"Orchestration failed: {str(e)}"

        return response
```

## Real-World Service Use Cases

### Configuration Services

Services for setting robot parameters:

```python
from my_robot_msgs.srv import SetParameter

class ConfigService(Node):
    def __init__(self):
        super().__init__('config_service')
        self.params = {}  # Store current parameters
        self.srv = self.create_service(SetParameter, 'set_parameter', self.set_parameter_callback)

    def set_parameter_callback(self, request, response):
        try:
            # Validate parameter
            if not self.validate_parameter(request.name, request.value):
                response.success = False
                response.message = f"Invalid parameter value for {request.name}"
                return response

            # Set parameter
            self.params[request.name] = request.value
            response.success = True
            response.message = f"Parameter {request.name} set to {request.value}"

        except Exception as e:
            response.success = False
            response.message = f"Error setting parameter: {str(e)}"

        return response
```

### Action Services

Services for performing specific robot actions:

```python
from my_robot_msgs.srv import ExecuteAction

class ActionService(Node):
    def __init__(self):
        super().__init__('action_service')
        self.srv = self.create_service(ExecuteAction, 'execute_action', self.execute_action_callback)

    def execute_action_callback(self, request, response):
        try:
            # Determine action type and execute
            if request.action_type == 'GRIPPER':
                response.success = self.execute_gripper_action(request.parameters)
            elif request.action_type == 'MOVE':
                response.success = self.execute_move_action(request.parameters)
            else:
                response.success = False
                response.message = f"Unknown action type: {request.action_type}"
                return response

            response.message = "Action completed successfully" if response.success else "Action failed"

        except Exception as e:
            response.success = False
            response.message = f"Action execution error: {str(e)}"

        return response
```

## Service Testing and Validation

### Unit Testing Services

```python
import unittest
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

class TestService(unittest.TestCase):
    def test_service_call(self):
        # Create service server and client
        service_node = MinimalService()
        client_node = MinimalClient()

        # Set up executor
        executor = SingleThreadedExecutor()
        executor.add_node(service_node)
        executor.add_node(client_node)

        # Make service call
        future = client_node.send_request(1, 2)
        executor.spin_until_future_complete(future, timeout_sec=5.0)

        # Verify result
        self.assertIsNotNone(future.result())
        self.assertEqual(future.result().sum, 3)
```

### Integration Testing

```python
def test_service_integration():
    # Test the full service workflow
    rclpy.init()

    try:
        service_node = RobotService()
        client_node = RobotClient()

        executor = SingleThreadedExecutor()
        executor.add_node(service_node)
        executor.add_node(client_node)

        # Run both nodes
        spin_thread = threading.Thread(target=executor.spin)
        spin_thread.start()

        # Make several service calls
        for i in range(5):
            result = client_node.call_service(i, i+1)
            assert result.success is True

        executor.shutdown()
        spin_thread.join()

    finally:
        rclpy.shutdown()
```

## Performance Considerations

### Service Latency and Throughput

- **Response time**: Services should respond within reasonable timeframes
- **Concurrency**: Consider thread safety if handling multiple requests
- **Resource usage**: Service operations shouldn't consume excessive resources
- **Timeout handling**: Always implement appropriate timeouts to avoid hanging

### Service Availability

```python
def check_service_availability(self):
    if self.cli.wait_for_service(timeout_sec=0.1):
        return True
    else:
        self.get_logger().warn('Service temporarily unavailable')
        return False
```

## Best Practices for Service Implementation

### Server Best Practices

1. **Validate inputs**: Always validate request parameters before processing
2. **Handle errors gracefully**: Return appropriate error responses
3. **Implement timeouts**: Don't allow service calls to hang indefinitely
4. **Log important events**: Log service calls for debugging and monitoring
5. **Use appropriate data types**: Choose message types that match your needs

### Client Best Practices

1. **Check service availability**: Verify the service is available before calling
2. **Implement timeouts**: Always use timeouts to prevent hanging
3. **Handle failures**: Implement retry logic for transient failures
4. **Validate responses**: Check response validity before using results
5. **Manage resources**: Properly clean up client resources when done

## Comparing Services with Other Communication Patterns

| Pattern | Use Case | Synchronization | Multiple Clients |
|---------|----------|-----------------|------------------|
| Topics | Streaming data | Asynchronous | Yes |
| Services | Request/Response | Synchronous | Yes |
| Actions | Long-running tasks | Asynchronous with feedback | Yes |

Services are particularly well-suited for operations that have a clear start and end, require specific results, and need guaranteed delivery.

## References

<div class="reference-list">

- Lütkebohle, I., et al. (2012). *ROS: From research to industry*. Robot Operating System (ROS), 1, 1-22.
- Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming robots with ROS: A practical introduction to the Robot Operating System*. O'Reilly Media.
- Siciliano, B., & Khatib, O. (2016). *Springer handbook of robotics*. Springer Publishing Company.

</div>