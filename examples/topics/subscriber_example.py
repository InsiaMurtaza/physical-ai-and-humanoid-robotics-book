#!/usr/bin/env python3

"""
Subscriber Example for ROS 2

This example demonstrates how to create a subscriber node that listens
to String messages from a topic and logs them.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberExample(Node):
    def __init__(self):
        super().__init__('subscriber_example')

        # Create subscription to the 'chatter' topic
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)  # QoS depth

        # Prevent unused variable warning
        self.subscription  # type: ignore

        self.get_logger().info('Subscriber node initialized, listening to /chatter')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    subscriber_example = SubscriberExample()

    try:
        rclpy.spin(subscriber_example)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()