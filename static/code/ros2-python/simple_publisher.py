#!/usr/bin/env python3

"""
Simple ROS 2 Publisher Example

This example demonstrates how to create a basic publisher node in ROS 2 using Python.
The node publishes a simple string message to a topic at a regular interval.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal ROS 2 publisher node that publishes a string message at regular intervals.
    """

    def __init__(self):
        """
        Initialize the node, create publisher, and set up timer.
        """
        super().__init__('minimal_publisher')

        # Create a publisher for the 'topic' with String message type and queue size 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Set timer period to 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Log that the node has been initialized
        self.get_logger().info('Minimal publisher node initialized')

    def timer_callback(self):
        """
        Callback function executed at regular intervals by the timer.
        Publishes a message with an incrementing counter.
        """
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    """
    Main function to initialize ROS 2, create the node, and spin to execute callbacks.
    """
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        # Keep the node running to execute callbacks
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up resources
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()