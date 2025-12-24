#!/usr/bin/env python3

"""
Service Server Example for ROS 2

This example demonstrates how to create a service server that adds two integers.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceServerExample(Node):
    def __init__(self):
        super().__init__('service_server_example')

        # Create service server
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

        self.get_logger().info('Service server initialized, offering /add_two_ints')

    def add_callback(self, request, response):
        response.sum = request.a + request.b

        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')

        return response


def main(args=None):
    rclpy.init(args=args)

    service_server_example = ServiceServerExample()

    try:
        rclpy.spin(service_server_example)
    except KeyboardInterrupt:
        pass
    finally:
        service_server_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()