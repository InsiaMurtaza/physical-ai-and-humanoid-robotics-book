#!/usr/bin/env python3

"""
Publisher Example for ROS 2

This example demonstrates how to create a publisher node that sends
String messages to a topic at a regular interval.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherExample(Node):
    def __init__(self):
        super().__init__('publisher_example')

        # Create publisher with QoS profile
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Set timer period (in seconds)
        timer_period = 0.5  # 2 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

        self.get_logger().info('Publisher node initialized, publishing to /chatter')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher_example = PublisherExample()

    try:
        rclpy.spin(publisher_example)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()