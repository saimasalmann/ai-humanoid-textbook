#!/usr/bin/env python3

"""
ROS 2 Parameter Example

This example demonstrates how to use parameters in a ROS 2 node using Python.
The node declares parameters with default values and uses them during execution.
"""

import rclpy
from rclpy.node import Node


class ParameterNode(Node):
    """
    A ROS 2 node that demonstrates parameter usage.
    """

    def __init__(self):
        """
        Initialize the node and declare parameters.
        """
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('operating_mode', 'autonomous')

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.operating_mode = self.get_parameter('operating_mode').value

        # Log parameter values
        self.get_logger().info(f'Robot: {self.robot_name}')
        self.get_logger().info(f'Max velocity: {self.max_velocity}')
        self.get_logger().info(f'Operating mode: {self.operating_mode}')

    def update_parameters_callback(self):
        """
        Callback function to handle parameter updates.
        """
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.operating_mode = self.get_parameter('operating_mode').value

        self.get_logger().info('Parameters updated')


def main(args=None):
    """
    Main function to initialize ROS 2, create the node, and spin to execute callbacks.
    """
    rclpy.init(args=args)

    parameter_node = ParameterNode()

    try:
        # Keep the node running to execute callbacks
        rclpy.spin(parameter_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up resources
        parameter_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()