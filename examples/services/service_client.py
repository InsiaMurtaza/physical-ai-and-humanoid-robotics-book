#!/usr/bin/env python3

"""
Service Client Example for ROS 2

This example demonstrates how to create a service client that calls
the add_two_ints service to add two integers.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceClientExample(Node):
    def __init__(self):
        super().__init__('service_client_example')

        # Create service client
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


def main(args=None):
    rclpy.init(args=args)

    # Check command line arguments
    if len(sys.argv) != 3:
        print('Usage: python3 service_client.py <int1> <int2>')
        return

    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print('Please provide two integers as arguments')
        return

    service_client_example = ServiceClientExample()

    # Send request
    future = service_client_example.send_request(a, b)

    # Wait for response
    try:
        rclpy.spin_until_future_complete(service_client_example, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            service_client_example.get_logger().info(
                f'Result of add_two_ints: {a} + {b} = {response.sum}')
        else:
            service_client_example.get_logger().info('Service call failed')

    except KeyboardInterrupt:
        pass
    finally:
        service_client_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()