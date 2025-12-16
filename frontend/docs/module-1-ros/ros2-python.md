---
title: ROS 2 with Python
description: Developing robotic applications using Python in ROS 2
sidebar_position: 4
learning_objectives:
  - Implement ROS 2 nodes using Python
  - Use Python client libraries for ROS 2
  - Handle messages and data types in Python
  - Integrate Python with other ROS 2 components
---

# ROS 2 with Python

## Learning Objectives

After completing this chapter, you will be able to:

1. Implement ROS 2 nodes using Python
2. Use Python client libraries for ROS 2
3. Handle messages and data types in Python
4. Integrate Python with other ROS 2 components

## Introduction

Python is one of the most popular languages for robotics development due to its simplicity, extensive libraries, and strong community support. ROS 2 provides robust Python client libraries that allow developers to create sophisticated robotic applications. This chapter explores the Python ecosystem in ROS 2 and demonstrates how to leverage Python's strengths for robotics.

## Python Client Libraries

![Python in ROS 2 Architecture](/img/python-ros2-integration.svg)

*Figure 1: Python Integration with ROS 2 showing rclpy client library*

ROS 2 provides the `rclpy` library as the Python client library for ROS 2. This library provides all the functionality needed to create ROS 2 nodes, publish and subscribe to topics, provide and use services, and execute actions.

### Installation and Setup

The Python client libraries are typically installed with ROS 2. To use them in your Python scripts, simply import the necessary modules:

```python
import rclpy
from rclpy.node import Node
```

## Creating Nodes in Python

Creating nodes in Python follows a consistent pattern that leverages Python's object-oriented features:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PythonRobotNode(Node):
    def __init__(self):
        super().__init__('python_robot_node')
        self.publisher_ = self.create_publisher(String, 'robot_commands', 10)
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10)
        self.get_logger().info('Python Robot Node initialized')

    def status_callback(self, msg):
        self.get_logger().info(f'Received status: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PythonRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Messages

ROS 2 messages in Python are handled through automatically generated Python modules. You can import and use messages from any package:

```python
# Import standard messages
from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, Image

# Import custom messages
from my_robot_msgs.msg import RobotState, SensorData

class MessageProcessor(Node):
    def __init__(self):
        super().__init__('message_processor')

        # Publisher with custom message
        self.state_publisher = self.create_publisher(RobotState, 'robot_state', 10)

        # Subscriber with sensor message
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'laser_scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        # Process laser scan data
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Minimum obstacle distance: {min_distance}')
```

## Services in Python

Services allow for request/response communication in ROS 2. Here's how to implement both service servers and clients in Python:

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client

```python
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions in Python

Actions are used for long-running tasks that require feedback. Here's how to implement actions in Python:

```python
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose

class ActionClientNode(Node):
    def __init__(self):
        super().__init__('navigate_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')
```

## Python Ecosystem Integration

Python's rich ecosystem makes it ideal for robotics applications that require data processing, machine learning, and visualization:

```python
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

class PythonIntegrationNode(Node):
    def __init__(self):
        super().__init__('python_integration_node')

        # Example: Using NumPy for transformations
        self.rotation_matrix = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])

        # Example: Using OpenCV for image processing
        self.image_subscriber = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format and process
        # Process with NumPy, visualize with Matplotlib, etc.
        pass
```

## Best Practices

When developing with Python in ROS 2, consider these best practices:

1. **Error Handling**: Always handle exceptions properly, especially when working with hardware
2. **Threading**: Use appropriate callback groups for thread safety
3. **Memory Management**: Be mindful of memory usage in long-running nodes
4. **Performance**: Profile your code to identify bottlenecks

## Safety Disclaimer

When using Python for ROS 2 development on physical robots, be aware of:
- Python's garbage collection can introduce unpredictable timing delays in real-time systems
- Always implement proper exception handling to prevent node crashes that could affect robot safety
- Consider using more performance-critical languages (like C++) for time-sensitive control loops
- Thoroughly test all Python-based nodes in simulation before hardware deployment

## Summary

Python provides a powerful and flexible environment for ROS 2 development. The `rclpy` client library provides full access to ROS 2 features while leveraging Python's extensive ecosystem. Understanding how to properly implement nodes, handle messages, and integrate with other Python libraries is essential for effective robotics development.

## Exercises

1. Create a Python ROS 2 node that processes sensor data using NumPy and publishes the results
2. Implement a service server and client in Python that performs a mathematical operation on robot data

## References

1. ROS 2 Documentation. (2023). "Tutorials: Using rclpy". Retrieved from https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html
2. ROS 2 Documentation. (2023). "Client Libraries". Retrieved from https://docs.ros.org/en/humble/Concepts/About-Client-Libraries.html
3. Open Robotics. (2023). "rclpy: Python Client Library for ROS 2". Open Robotics.
4. van der Walt, S., et al. (2011). "The NumPy array: a structure for efficient numerical computation". Computing in Science & Engineering.
5. Bradski, G. (2000). "The OpenCV Library". Dr. Dobb's Journal of Software Tools.
6. Quigley, M., et al. (2009). "ROS: an open-source Robot Operating System". ICRA Workshop.
7. MXRC Team. (2023). "Python in Robotics". Open Robotics.
8. Colomé, A., et al. (2016). "Python for robotics applications". IEEE/RSJ IROS.