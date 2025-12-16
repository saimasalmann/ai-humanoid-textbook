---
title: API Reference
description: Documentation for key APIs and libraries used in robotics development
sidebar_position: 3
learning_objectives:
  - Understand ROS 2 client library APIs
  - Navigate common robotics library interfaces
  - Apply API patterns in robotics development
  - Integrate with standard robotics frameworks
---

# API Reference

## Learning Objectives

After reviewing this API reference, you will be able to:

1. Understand ROS 2 client library APIs
2. Navigate common robotics library interfaces
3. Apply API patterns in robotics development
4. Integrate with standard robotics frameworks

## Introduction

This API reference provides documentation for key interfaces and libraries used throughout the Physical AI & Humanoid Robotics textbook. Understanding these APIs is essential for implementing the concepts covered in the modules and creating effective robotic systems.

## ROS 2 Client Library (rclpy) API

### Node API

The `rclpy.node.Node` class is the fundamental building block for ROS 2 Python nodes:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('my_robot_node')

        # Create a publisher
        self.publisher = self.create_publisher(
            msg_type=String,           # Message type
            topic='topic_name',        # Topic name
            qos_profile=10            # Quality of Service profile
        )

        # Create a subscriber
        self.subscriber = self.create_subscription(
            msg_type=String,           # Message type
            topic='topic_name',        # Topic name
            callback=self.callback,    # Callback function
            qos_profile=10            # Quality of Service profile
        )

        # Create a timer
        self.timer = self.create_timer(
            timer_period_sec=0.5,      # Period in seconds
            callback=self.timer_callback  # Callback function
        )

        # Declare and use parameters
        self.declare_parameter(
            name='param_name',         # Parameter name
            value='default_value'      # Default value
        )

        # Access parameter value
        param_value = self.get_parameter('param_name').value
```

### Message Types

Common message types used in robotics:

#### Standard Messages
- `std_msgs.msg.String` - Simple string messages
- `std_msgs.msg.Int32`, `std_msgs.msg.Float64` - Numeric values
- `std_msgs.msg.Bool` - Boolean values

#### Geometry Messages
- `geometry_msgs.msg.Twist` - Velocity commands (linear and angular)
- `geometry_msgs.msg.Pose` - Position and orientation
- `geometry_msgs.msg.Point` - 3D point coordinates
- `geometry_msgs.msg.Quaternion` - Rotation representation

#### Sensor Messages
- `sensor_msgs.msg.LaserScan` - Laser range finder data
- `sensor_msgs.msg.Image` - Camera image data
- `sensor_msgs.msg.JointState` - Robot joint positions and velocities

### Example Usage

```python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 10
        )

        # Subscriber for laser data
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

    def send_velocity_command(self, linear_x, angular_z):
        """Send velocity command to robot."""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)

    def scan_callback(self, msg):
        """Process laser scan data."""
        # Access scan ranges
        ranges = msg.ranges
        # Access angle information
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
```

## ROS 2 Command Line Interface (CLI) Tools

### Node Commands

```bash
# List active nodes
ros2 node list

# Get information about a specific node
ros2 node info <node_name>

# List a node's parameters
ros2 param list <node_name>

# Get a parameter value
ros2 param get <node_name> <param_name>

# Set a parameter value
ros2 param set <node_name> <param_name> <value>
```

### Topic Commands

```bash
# List topics
ros2 topic list

# Get topic information
ros2 topic info <topic_name>

# Echo messages on a topic
ros2 topic echo <topic_name>

# Publish a message to a topic
ros2 topic pub <topic_name> <msg_type> '<data>'

# Show topic statistics
ros2 topic hz <topic_name>
```

### Service Commands

```bash
# List services
ros2 service list

# Call a service
ros2 service call <service_name> <service_type> '<request_data>'

# Get service information
ros2 service info <service_name>
```

### Action Commands

```bash
# List actions
ros2 action list

# Send a goal to an action
ros2 action send_goal <action_name> <action_type> '<goal_data>'
```

## Common Robotics Libraries

### OpenCV (cv2)

OpenCV is essential for computer vision in robotics:

```python
import cv2
import numpy as np

# Reading and displaying images
image = cv2.imread('path_to_image.jpg')
cv2.imshow('Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Basic image processing
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(image, (5, 5), 0)
edges = cv2.Canny(gray, 50, 150)

# Feature detection
sift = cv2.SIFT_create()
keypoints, descriptors = sift.detectAndCompute(gray, None)

# Contour detection
contours, hierarchy = cv2.findContours(
    edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
)

# Drawing on images
cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
cv2.circle(image, (cx, cy), radius, (255, 0, 0), -1)
```

### NumPy for Robotics

NumPy is crucial for mathematical operations in robotics:

```python
import numpy as np

# Vector operations for robot kinematics
position = np.array([1.0, 2.0, 0.0])
velocity = np.array([0.5, 0.0, 0.0])

# Matrix operations for transformations
rotation_matrix = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta), np.cos(theta)]
])

# Creating transformation matrices
def create_translation_matrix(x, y, z):
    """Create a 4x4 translation matrix."""
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

# Array operations for sensor data processing
sensor_data = np.random.random(360)  # Simulated laser scan
filtered_data = sensor_data[sensor_data > 0.1]  # Remove invalid readings
```

### SciPy for Scientific Computing

SciPy provides advanced mathematical functions:

```python
from scipy.spatial.transform import Rotation as R
from scipy import ndimage
import scipy.optimize as opt

# 3D rotations using quaternions
rotation = R.from_quat([x, y, z, w])  # x, y, z, w quaternion
euler_angles = rotation.as_euler('xyz')  # Convert to Euler angles

# Optimization for path planning
def optimize_path(waypoints):
    """Optimize a path using scipy optimization."""
    result = opt.minimize(
        fun=path_cost_function,
        x0=initial_path,
        method='BFGS'
    )
    return result.x

# Image processing with scipy
filtered_image = ndimage.gaussian_filter(image, sigma=1.0)
```

## Robot State Publisher API

The `robot_state_publisher` is essential for handling robot kinematics:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import tf_transformations

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10
        )

    def joint_callback(self, msg):
        """Process joint state messages and broadcast transforms."""
        # Process each joint and broadcast its transform
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                # Create and broadcast transform
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'base_link'
                t.child_frame_id = joint_name
                # Set transform values based on joint position
                self.tf_broadcaster.sendTransform(t)
```

## Navigation Stack API

The ROS 2 Navigation stack provides path planning and execution:

```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigationClient:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(
            node,
            NavigateToPose,
            'navigate_to_pose'
        )

    def navigate_to_pose(self, x, y, theta):
        """Send navigation goal to the navigation stack."""
        goal_msg = NavigateToPose.Goal()

        # Set the target pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta

        # Wait for server and send goal
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        return future
```

## Isaac Sim and Isaac ROS APIs

### Isaac ROS Perception Nodes

Isaac ROS provides GPU-accelerated perception:

```python
# Example of using Isaac ROS Visual SLAM
from isaac_ros_visual_slam_msgs.msg import IsaacROSVisualSLAM

class IsaacPerceptionNode(Node):
    def __init__(self):
        super().__init__('isaac_perception_node')

        # Isaac ROS typically provides optimized interfaces
        # for visual SLAM, object detection, and other perception tasks
        self.visual_slam_sub = self.create_subscription(
            IsaacROSVisualSLAM,
            'visual_slam_results',
            self.slam_callback,
            10
        )

    def slam_callback(self, msg):
        """Process Isaac ROS SLAM results."""
        # Handle optimized SLAM data from Isaac
        position = msg.position
        orientation = msg.orientation
        # Process optimized results
```

## Common Design Patterns in Robotics APIs

### Publisher-Subscriber Pattern

```python
class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic', 10)

    def publish_data(self, data):
        msg = String()
        msg.data = data
        self.publisher.publish(msg)

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscriber = self.create_subscription(
            String, 'topic', self.callback, 10
        )

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

### Service Client-Server Pattern

```python
# Server
class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_callback
        )

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        return response

# Client
class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.cli.call_async(request)
        return future
```

## Error Handling and Best Practices

### Exception Handling in Robotics Nodes

```python
class RobustRobotNode(Node):
    def __init__(self):
        super().__init__('robust_robot_node')

    def safe_publish(self, publisher, msg):
        """Safely publish a message with error handling."""
        try:
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish message: {e}')

    def validate_sensor_data(self, sensor_msg):
        """Validate sensor data before processing."""
        if sensor_msg is None:
            self.get_logger().warn('Received None sensor data')
            return False

        # Check for invalid values
        if any(np.isnan(sensor_msg.ranges)):
            self.get_logger().warn('Sensor data contains NaN values')
            return False

        return True
```

### Resource Management

```python
class ResourceManagedNode(Node):
    def __init__(self):
        super().__init__('resource_managed_node')
        # Initialize resources
        self.camera = None
        self.laser = None

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        if self.camera:
            self.camera.release()
        if self.laser:
            self.laser.shutdown()
        super().destroy_node()
```

## Summary

This API reference covers the essential interfaces and libraries used in robotics development with ROS 2. Understanding these APIs is crucial for implementing the concepts covered in this textbook. The reference includes ROS 2 client libraries, command-line tools, common robotics libraries like OpenCV and NumPy, and specialized APIs for navigation and perception systems.

## Exercises

1. Create a ROS 2 node that uses multiple message types (Twist, LaserScan, Image)
2. Implement a service server and client for a robotics-specific operation
3. Use NumPy and OpenCV to process simulated sensor data

## References

1. ROS 2 Documentation. (2023). "rclpy API Documentation". Retrieved from https://docs.ros.org/en/humble/p/rclpy/
2. Open Robotics. (2023). "ROS 2 Client Libraries". Retrieved from https://docs.ros.org/en/humble/Concepts/About-Client-Libraries.html
3. OpenCV Team. (2023). "OpenCV Python API". Retrieved from https://docs.opencv.org/
4. NumPy Team. (2023). "NumPy Documentation". Retrieved from https://numpy.org/doc/
5. SciPy Team. (2023). "SciPy API Reference". Retrieved from https://docs.scipy.org/doc/scipy/reference/
6. NVIDIA. (2023). "Isaac ROS API Documentation". NVIDIA Corporation.
7. OSRF. (2023). "Navigation2 API". Open Source Robotics Foundation.
8. MXRC Team. (2023). "ROS 2 API Best Practices". Open Robotics.

## Safety Disclaimer

When using robotics APIs, always implement proper error handling and safety checks. Robot systems can cause physical harm if operated incorrectly. Ensure that all API calls are validated, implement emergency stop mechanisms, and test all robotic systems thoroughly in simulation before physical deployment. Never bypass safety protocols when using robotics APIs.