#!/usr/bin/env python3

"""
Simple ROS 2 Subscriber Example

This example demonstrates how to create a basic subscriber node in ROS 2 using Python.
The node subscribes to a topic and logs the received messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal ROS 2 subscriber node that listens to messages on a topic.
    """

    def __init__(self):
        """
        Initialize the node and create a subscription.
        """
        super().__init__('minimal_subscriber')

        # Create a subscription to the 'topic' with String message type and queue size 10
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Prevent unused variable warning
        self.subscription  # type: ignore[attr-defined]

        # Log that the node has been initialized
        self.get_logger().info('Minimal subscriber node initialized')

    def listener_callback(self, msg):
        """
        Callback function executed when a message is received on the topic.

        Args:
            msg (String): The received message containing string data.
        """
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to initialize ROS 2, create the node, and spin to execute callbacks.
    """
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        # Keep the node running to execute callbacks
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up resources
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()