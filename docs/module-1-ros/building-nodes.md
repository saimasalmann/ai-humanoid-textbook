---
title: Building ROS 2 Nodes
description: Creating and implementing ROS 2 nodes for robotic applications
sidebar_position: 3
learning_objectives:
  - Create ROS 2 nodes in Python
  - Implement different communication patterns
  - Use parameters and configuration in nodes
  - Debug and test ROS 2 nodes
---

# Building ROS 2 Nodes

## Learning Objectives

After completing this chapter, you will be able to:

1. Create ROS 2 nodes in Python
2. Implement different communication patterns
3. Use parameters and configuration in nodes
4. Debug and test ROS 2 nodes

## Introduction

Creating ROS 2 nodes is fundamental to developing robotic applications. This chapter covers the process of building nodes that can communicate with other nodes, handle parameters, and execute robot-specific tasks. We'll explore how to structure nodes properly and implement common robotics patterns.

## Creating a Basic Node

To create a ROS 2 node, you need to:

1. Import the necessary ROS 2 libraries
2. Create a class that inherits from `rclpy.node.Node`
3. Initialize the node with a name
4. Implement the node's functionality
5. Set up the main function to run the node

```python
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info('Robot Controller node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing Publishers

![Node Communication Pattern](/img/node-communication.svg)

*Figure 1: Node Communication Pattern showing publisher-subscriber interaction*

Publishers allow nodes to send messages to topics. Here's how to implement a publisher in a ROS 2 node:

```python
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Implementing Subscribers

Subscribers receive messages from topics. Here's how to implement a subscriber:

```python
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Parameters in Nodes

ROS 2 nodes can accept parameters at runtime, making them configurable:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('max_velocity', 0.5)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value

        self.get_logger().info(f'Robot: {self.robot_name}, Max velocity: {self.max_velocity}')
```

## Launch Files

Launch files allow you to start multiple nodes with specific configurations:

```xml
<!-- example_launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='robot_controller',
            name='robot_controller',
            parameters=[
                {'robot_name': 'turtlebot'},
                {'max_velocity': 0.5}
            ]
        ),
        Node(
            package='my_robot',
            executable='sensor_processor',
            name='sensor_processor'
        )
    ])
```

## Safety Disclaimer

When building ROS 2 nodes for physical robots, consider the following safety aspects:
- Implement proper error handling and recovery mechanisms
- Include timeouts and fail-safes in all node operations
- Ensure nodes can be safely stopped or restarted without compromising robot safety
- Test all node interactions thoroughly in simulation before hardware deployment

## Summary

Building ROS 2 nodes involves understanding the node lifecycle, implementing communication patterns, and configuring parameters. Properly structured nodes form the building blocks of complex robotic systems. Testing and debugging are essential skills for developing reliable nodes.

## Exercises

1. Create a ROS 2 node that publishes sensor data and another node that subscribes to it
2. Implement a node that uses parameters to configure its behavior and launch it with different parameter values

## References

1. ROS 2 Documentation. (2023). "Tutorials: Writing a simple publisher and subscriber". Retrieved from https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
2. ROS 2 Documentation. (2023). "Tutorials: Using parameters in a class". Retrieved from https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html
3. MXRC Team. (2023). "ROS 2 Python Client Library". Open Robotics.
4. Faconti, P., et al. (2019). "ROS 2 Design Overview". Open Robotics.
5. ROS 2 Working Group. (2023). "Node Lifecycle". Open Robotics.
6. Smart, W. D., et al. (2010). "Programming with ROS". A Gentle Introduction to ROS.
7. Colomé, A., et al. (2016). "Real-time control with ROS". IEEE/RSJ IROS.
8. Quigley, M., et al. (2009). "ROS: an open-source Robot Operating System". ICRA Workshop.