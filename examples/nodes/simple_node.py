#!/usr/bin/env python3

"""
Simple Node Example for ROS 2

This example demonstrates the basic structure of a ROS 2 node,
including initialization, parameter declaration, and cleanup.
"""

import rclpy
from rclpy.node import Node


class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')

        # Declare parameters with default values
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('node_name', 'simple_node')

        # Access parameter values
        self.frequency = self.get_parameter('frequency').value
        self.node_name = self.get_parameter('node_name').value

        # Log node creation
        self.get_logger().info(f'{self.node_name} initialized with frequency: {self.frequency}Hz')

        # Example timer
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback executed: {self.counter}')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    node = SimpleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()