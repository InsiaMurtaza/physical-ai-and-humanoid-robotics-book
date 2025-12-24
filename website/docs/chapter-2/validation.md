# Validation: ROS 2 Examples Expected Output

This document describes the expected output when running the ROS 2 examples to validate they work correctly on ROS 2 Humble/Foxy.

## Nodes Example

### Running the Simple Node
Command:
```bash
python3 examples/nodes/simple_node.py
```

Expected Output:
```
[INFO] [1692345678.123456789] [simple_node]: simple_node initialized with frequency: 1.0Hz
[INFO] [1692345678.623456789] [simple_node]: Timer callback executed: 0
[INFO] [1692345679.123456789] [simple_node]: Timer callback executed: 1
[INFO] [1692345679.623456789] [simple_node]: Timer callback executed: 2
...
```

The node should:
- Initialize with the specified frequency (default 1.0Hz)
- Log timer callbacks at the specified interval
- Continue running until interrupted with Ctrl+C

## Topics Examples

### Running the Publisher
Command:
```bash
python3 examples/topics/publisher_example.py
```

Expected Output:
```
[INFO] [1692345678.123456789] [publisher_example]: Publisher node initialized, publishing to /chatter
[INFO] [1692345678.623456789] [publisher_example]: Publishing: "Hello World: 0"
[INFO] [1692345679.123456789] [publisher_example]: Publishing: "Hello World: 1"
[INFO] [1692345679.623456789] [publisher_example]: Publishing: "Hello World: 2"
...
```

The publisher should:
- Initialize and log that it's publishing to `/chatter`
- Publish messages at 2Hz (every 0.5 seconds)
- Increment the counter in each message

### Running the Subscriber
Command:
```bash
python3 examples/topics/subscriber_example.py
```

Expected Output:
```
[INFO] [1692345678.123456789] [subscriber_example]: Subscriber node initialized, listening to /chatter
[INFO] [1692345678.623456789] [subscriber_example]: I heard: "Hello World: 0"
[INFO] [1692345679.123456789] [subscriber_example]: I heard: "Hello World: 1"
[INFO] [1692345679.623456789] [subscriber_example]: I heard: "Hello World: 2"
...
```

The subscriber should:
- Initialize and log that it's listening to `/chatter`
- Receive and log messages from the publisher
- Display the same messages that were published

### Combined Publisher/Subscriber Test
1. Start the publisher in one terminal
2. Start the subscriber in another terminal
3. The subscriber should receive and display messages published by the publisher
4. Both nodes should continue running until interrupted

## Services Examples

### Running the Service Server
Command:
```bash
python3 examples/services/service_server.py
```

Expected Output:
```
[INFO] [1692345678.123456789] [service_server_example]: Service server initialized, offering /add_two_ints
```

The service server should:
- Initialize and log that it's offering the `/add_two_ints` service
- Wait for service requests
- Remain running and ready to respond to requests

### Running the Service Client
Command:
```bash
python3 examples/services/service_client.py 10 20
```

Expected Output:
```
[INFO] [1692345678.123456789] [service_client_example]: Result of add_two_ints: 10 + 20 = 30
```

The service client should:
- Connect to the service server
- Send the request with the provided numbers (10 and 20 in this example)
- Receive and display the result (30 in this example)

### Combined Service Server/Client Test
1. Start the service server in one terminal
2. Run the service client in another terminal with two integers as arguments
3. The client should receive the sum of the two integers
4. The server should log the calculation it performed

## Troubleshooting

### Common Issues and Solutions

#### 1. ModuleNotFoundError: No module named 'rclpy'
**Solution**: Ensure ROS 2 is properly installed and sourced:
```bash
source /opt/ros/humble/setup.bash  # For Humble
# OR
source /opt/ros/foxy/setup.bash     # For Foxy
```

#### 2. Service not available, waiting again...
**Solution**: Make sure the service server is running before starting the client.

#### 3. No messages received by subscriber
**Solution**:
- Ensure the publisher is running
- Check that both nodes are on the same ROS domain
- Verify ROS 2 environment is properly sourced

#### 4. Permission errors during installation
**Solution**: Use `sudo` for system package installation commands.

## Verification Checklist

- [ ] Simple node example runs and logs timer callbacks
- [ ] Publisher example runs and publishes messages to `/chatter`
- [ ] Subscriber example runs and receives messages from `/chatter`
- [ ] Publisher and subscriber communicate correctly when run together
- [ ] Service server runs and offers the `/add_two_ints` service
- [ ] Service client connects to server and receives correct response
- [ ] Service call returns the sum of the two input integers
- [ ] All examples work with both ROS 2 Humble and Foxy (where applicable)

## ROS 2 Version Compatibility

All examples have been designed to work with:
- ROS 2 Humble Hawksbill (recommended)
- ROS 2 Foxy Fitzroy

The examples use standard ROS 2 interfaces that are stable across these versions.