---
title: Capstone Implementation - Step-by-Step Implementation Guide
description: Detailed implementation guide for building the autonomous conversational humanoid robot
sidebar_position: 3
learning_objectives:
  - Implement the system architecture in code
  - Integrate all textbook concepts into a working system
  - Validate system functionality through testing
  - Deploy and operate the complete system
---

# Capstone Implementation: Step-by-Step Implementation Guide

## Learning Objectives
- Implement the system architecture in code
- Integrate all textbook concepts into a working system
- Validate system functionality through testing
- Deploy and operate the complete system

## Introduction
This document provides a detailed implementation guide for building the autonomous conversational humanoid robot system. The implementation follows the architectural design and integrates all concepts learned in the textbook modules. The guide is structured to enable systematic implementation and validation of each subsystem before integration.

## Implementation Strategy

### Phased Approach
The implementation follows a phased approach that builds complexity incrementally:

1. **Foundation Phase**: Core ROS 2 infrastructure and basic communication
2. **Perception Phase**: Basic sensing and environment understanding
3. **Navigation Phase**: Movement and obstacle avoidance
4. **Interaction Phase**: Human-robot interaction capabilities
5. **Integration Phase**: Full system integration and validation

### Development Environment Setup
Before beginning implementation, ensure your development environment is properly configured:

```bash
# Verify ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 topic list

# Verify Gazebo installation
gazebo --version

# Verify Isaac Sim (if available)
# Ensure NVIDIA GPU and drivers are properly configured

# Create workspace
mkdir -p ~/capstone_ws/src
cd ~/capstone_ws
colcon build
source install/setup.bash
```

## Phase 1: Foundation Implementation

### 1.1 Core Node Structure
Implement the core node structure following ROS 2 best practices:

```python
# capstone_humanoid/capstone_humanoid/main_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
import threading
import time

class CapstoneHumanoidNode(Node):
    def __init__(self):
        super().__init__('capstone_humanoid_main')

        # QoS profiles for different message types
        self.sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Initialize subsystems
        self.initialize_safety_system()
        self.initialize_communication()

        # Main control timer
        self.control_rate = 10.0  # Hz
        self.control_timer = self.create_timer(1.0/self.control_rate, self.control_loop)

        self.get_logger().info('Capstone Humanoid Foundation Initialized')

    def initialize_safety_system(self):
        """Initialize safety-critical systems."""
        self.emergency_active = False
        self.safety_override = False

        # Emergency stop publisher
        self.emergency_pub = self.create_publisher(
            Bool, 'emergency_stop', self.command_qos)

        # Safety monitoring timer
        self.safety_timer = self.create_timer(0.05, self.safety_monitor)  # 20Hz safety check

    def initialize_communication(self):
        """Initialize all communication interfaces."""
        # Command interfaces
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', self.command_qos)

        # Sensor interfaces
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, self.sensor_qos)

        self.camera_sub = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, self.sensor_qos)

    def laser_callback(self, msg):
        """Handle laser scan data."""
        # Store for use in control loop
        self.last_laser_scan = msg
        self.last_laser_time = self.get_clock().now()

    def camera_callback(self, msg):
        """Handle camera image data."""
        # Store for use in control loop
        self.last_camera_image = msg
        self.last_camera_time = self.get_clock().now()

    def safety_monitor(self):
        """Monitor system safety status."""
        try:
            # Check for safety violations
            if self.emergency_active:
                self.emergency_stop()
                return

            # Check sensor data freshness
            current_time = self.get_clock().now()
            if hasattr(self, 'last_laser_time'):
                time_since_scan = (current_time - self.last_laser_time).nanoseconds / 1e9
                if time_since_scan > 1.0:  # More than 1 second without scan
                    self.get_logger().warn('Laser scan timeout - stopping robot')
                    self.emergency_stop()

        except Exception as e:
            self.get_logger().error(f'Safety monitor error: {e}')
            self.emergency_stop()

    def control_loop(self):
        """Main control loop - called at control_rate frequency."""
        try:
            if self.emergency_active:
                return

            # Collect sensor data
            sensor_data = self.collect_sensor_data()

            # Process and generate commands
            commands = self.process_control_logic(sensor_data)

            # Validate commands for safety
            if self.validate_commands(commands):
                self.execute_commands(commands)
            else:
                self.get_logger().warn('Commands failed validation - not executing')

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
            self.emergency_stop()

    def collect_sensor_data(self):
        """Collect and preprocess sensor data."""
        data = {}

        if hasattr(self, 'last_laser_scan'):
            data['laser'] = self.last_laser_scan
        if hasattr(self, 'last_camera_image'):
            data['camera'] = self.last_camera_image

        return data

    def process_control_logic(self, sensor_data):
        """Process sensor data and generate control commands."""
        # Placeholder - implement actual control logic
        cmd = Twist()

        # Default: stop if no valid data
        if not sensor_data:
            return cmd

        # Example: simple obstacle avoidance
        if 'laser' in sensor_data:
            ranges = sensor_data['laser'].ranges
            min_range = min(ranges) if ranges else float('inf')

            if min_range < 0.5:  # Obstacle closer than 0.5m
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5  # Turn away
            else:
                cmd.linear.x = 0.2  # Move forward slowly

        return cmd

    def validate_commands(self, commands):
        """Validate commands for safety."""
        if self.emergency_active or self.safety_override:
            return False

        # Check velocity bounds
        max_linear = 0.5  # m/s
        max_angular = 1.0  # rad/s

        if abs(commands.linear.x) > max_linear:
            return False
        if abs(commands.angular.z) > max_angular:
            return False

        return True

    def execute_commands(self, commands):
        """Execute validated commands."""
        self.cmd_vel_pub.publish(commands)

    def emergency_stop(self):
        """Execute emergency stop procedure."""
        if not self.emergency_active:
            self.get_logger().fatal('EMERGENCY STOP ACTIVATED')

        # Publish stop command
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # Set emergency flag
        self.emergency_active = True

        # Publish emergency status
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)

def main(args=None):
    rclpy.init(args=args)

    node = CapstoneHumanoidNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Capstone Humanoid Node')
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 1.2 Package Structure
Create the ROS 2 package structure:

```bash
# Create package
cd ~/capstone_ws/src
ros2 pkg create --dependencies rclpy std_msgs geometry_msgs sensor_msgs message_generation --cpp-node-name capstone_main capstone_humanoid
```

### 1.3 Configuration Files
Create necessary configuration files:

```yaml
# config/safety_params.yaml
capstone_humanoid_main:
  ros__parameters:
    safety:
      linear_velocity_max: 0.5
      angular_velocity_max: 1.0
      emergency_distance: 0.3
      safety_timeout: 1.0
    control:
      loop_rate: 10.0
      safety_rate: 20.0
```

## Phase 2: Perception Implementation

### 2.1 Perception Node
Implement the perception subsystem:

```python
# capstone_humanoid/capstone_humanoid/perception_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('capstone_perception')

        self.bridge = CvBridge()

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, 'perception/detections', 10)

        self.obstacle_pub = self.create_publisher(
            PointCloud2, 'perception/obstacles', 10)

        # Processing parameters
        self.min_detection_confidence = 0.7
        self.max_detection_distance = 3.0

        self.get_logger().info('Perception Node Initialized')

    def image_callback(self, msg):
        """Process incoming camera image."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform object detection (simplified example)
            detections = self.detect_objects(cv_image)

            # Publish detections
            self.publish_detections(detections, msg.header)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def laser_callback(self, msg):
        """Process laser scan for obstacle detection."""
        try:
            # Simple obstacle detection from laser scan
            obstacles = self.detect_obstacles_from_scan(msg)

            # Publish obstacle points
            self.publish_obstacles(obstacles, msg.header)

        except Exception as e:
            self.get_logger().error(f'Laser processing error: {e}')

    def detect_objects(self, image):
        """Detect objects in image - simplified for example."""
        # In a real implementation, this would use a trained model
        # For this example, we'll simulate detection

        detections = []

        # Example: detect a person-like object in center of image
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2

        # Simulate detection at center
        detection = {
            'class': 'person',
            'confidence': 0.85,
            'bbox': [center_x - 50, center_y - 100, 100, 200],  # x, y, w, h
            'center': [center_x, center_y]
        }

        if detection['confidence'] > self.min_detection_confidence:
            detections.append(detection)

        return detections

    def detect_obstacles_from_scan(self, scan_msg):
        """Detect obstacles from laser scan."""
        obstacles = []

        for i, range_val in enumerate(scan_msg.ranges):
            if not (np.isnan(range_val) or np.isinf(range_val)):
                if range_val < self.max_detection_distance:
                    angle = scan_msg.angle_min + i * scan_msg.angle_increment
                    x = range_val * np.cos(angle)
                    y = range_val * np.sin(angle)
                    obstacles.append([x, y, 0.0])  # x, y, z

        return obstacles

    def publish_detections(self, detections, header):
        """Publish object detections."""
        detection_array = Detection2DArray()
        detection_array.header = header

        for det in detections:
            detection_msg = Detection2D()
            detection_msg.header = header

            # Bounding box
            bbox = det['bbox']
            detection_msg.bbox.center.x = bbox[0] + bbox[2] / 2.0
            detection_msg.bbox.center.y = bbox[1] + bbox[3] / 2.0
            detection_msg.bbox.size_x = bbox[2]
            detection_msg.bbox.size_y = bbox[3]

            # Hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = det['class']
            hypothesis.score = det['confidence']
            detection_msg.results.append(hypothesis)

            detection_array.detections.append(detection_msg)

        self.detection_pub.publish(detection_array)

    def publish_obstacles(self, obstacles, header):
        """Publish obstacle points."""
        # Simplified - in practice would create proper PointCloud2 message
        pass

def main(args=None):
    rclpy.init(args=args)

    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Perception Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 3: Navigation Implementation

### 3.1 Navigation Node
Implement the navigation subsystem:

```python
# capstone_humanoid/capstone_humanoid/navigation_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import math
from collections import deque

class NavigationNode(Node):
    def __init__(self):
        super().__init__('capstone_navigation')

        # Robot state
        self.current_pose = None
        self.current_twist = None
        self.target_pose = None
        self.is_moving = False
        self.emergency_stop = False

        # Navigation parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.arrival_threshold = 0.2
        self.rotation_threshold = 0.1

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        self.target_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)

        self.emergency_sub = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'current_path', 10)

        # Navigation timer
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)

        self.get_logger().info('Navigation Node Initialized')

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def scan_callback(self, msg):
        """Process laser scan for navigation safety."""
        # Check for obstacles in path
        if self.is_moving and self.current_twist:
            # Simple obstacle detection
            min_range = min(msg.ranges) if msg.ranges else float('inf')

            if min_range < 0.3:  # Stop if obstacle too close
                self.stop_robot()
                self.get_logger().warn('Obstacle detected - stopping navigation')

    def goal_callback(self, msg):
        """Receive navigation goal."""
        self.target_pose = msg.pose
        self.get_logger().info(f'New navigation goal received: ({msg.pose.position.x}, {msg.pose.position.y})')
        self.is_moving = True

    def emergency_callback(self, msg):
        """Handle emergency stop."""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.stop_robot()

    def navigation_loop(self):
        """Main navigation control loop."""
        if self.emergency_stop or not self.target_pose or not self.current_pose:
            return

        if not self.is_moving:
            return

        # Calculate navigation commands
        cmd = self.calculate_navigation_command()

        # Publish command
        self.cmd_pub.publish(cmd)

        # Check if reached destination
        distance = self.calculate_distance_to_target()
        if distance < self.arrival_threshold:
            self.stop_robot()
            self.get_logger().info('Destination reached')

    def calculate_navigation_command(self):
        """Calculate navigation command based on current and target pose."""
        cmd = Twist()

        if not self.current_pose or not self.target_pose:
            return cmd

        # Calculate distance and angle to target
        dx = self.target_pose.position.x - self.current_pose.position.x
        dy = self.target_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate current and target angles
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        target_angle = math.atan2(dy, dx)

        # Calculate angle difference
        angle_diff = target_angle - current_yaw
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # If pointing roughly toward target, move forward
        if abs(angle_diff) < self.rotation_threshold:
            cmd.linear.x = min(self.linear_speed, distance * 2.0)  # Scale speed with distance
        else:
            # Rotate toward target
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, angle_diff * 2.0))

        return cmd

    def calculate_distance_to_target(self):
        """Calculate distance to navigation target."""
        if not self.current_pose or not self.target_pose:
            return float('inf')

        dx = self.target_pose.position.x - self.current_pose.position.x
        dy = self.target_pose.position.y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stop_robot(self):
        """Stop robot movement."""
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        self.is_moving = False

def main(args=None):
    rclpy.init(args=args)

    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Navigation Node')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 4: Human-Robot Interaction Implementation

### 4.1 Interaction Node
Implement the human-robot interaction subsystem:

```python
# capstone_humanoid/capstone_humanoid/interaction_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Time
import openai
import json
import threading
import queue

class InteractionNode(Node):
    def __init__(self):
        super().__init__('capstone_interaction')

        # Configuration
        self.llm_model = "gpt-3.5-turbo"
        self.context_window_size = 50  # Keep last N interactions

        # Initialize context
        self.conversation_history = []
        self.robot_capabilities = [
            "navigation", "object_detection", "basic_manipulation",
            "conversation", "environment_mapping"
        ]

        # Subscriptions
        self.speech_sub = self.create_subscription(
            String, 'speech_recognition/result', self.speech_callback, 10)

        self.vision_sub = self.create_subscription(
            String, 'vision_system/description', self.vision_callback, 10)

        # Publishers
        self.tts_pub = self.create_publisher(String, 'tts/input', 10)
        self.action_pub = self.create_publisher(String, 'action_queue', 10)
        self.response_pub = self.create_publisher(String, 'conversation/response', 10)

        # Processing queue
        self.input_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_inputs)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info('Interaction Node Initialized')

    def speech_callback(self, msg):
        """Handle speech recognition results."""
        try:
            # Add to processing queue
            self.input_queue.put(('speech', msg.data))
            self.get_logger().info(f'Processing speech: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Speech callback error: {e}')

    def vision_callback(self, msg):
        """Handle vision system results."""
        try:
            self.input_queue.put(('vision', msg.data))
            self.get_logger().info(f'Processing vision: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Vision callback error: {e}')

    def process_inputs(self):
        """Process inputs in separate thread."""
        while rclpy.ok():
            try:
                input_type, input_data = self.input_queue.get(timeout=1.0)

                if input_type == 'speech':
                    response = self.process_speech_input(input_data)
                    self.publish_response(response)
                elif input_type == 'vision':
                    # Process vision input as context
                    self.add_to_context(f"Vision system reports: {input_data}")

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Input processing error: {e}')

    def process_speech_input(self, user_input):
        """Process user speech input and generate response."""
        try:
            # Add user input to context
            user_message = {"role": "user", "content": user_input}
            self.add_to_context(user_message)

            # Prepare context for LLM
            context_messages = self.get_context_messages()

            # Add system prompt with robot capabilities
            system_prompt = {
                "role": "system",
                "content": f"""You are an autonomous conversational humanoid robot. Your capabilities include: {', '.join(self.robot_capabilities)}.
                Respond naturally and appropriately to human commands and questions.
                If asked to perform an action, format your response as: ACTION: action_type(action_parameters)
                For example: ACTION: navigate_to(location="kitchen")
                Keep responses concise but informative."""
            }

            # Prepare messages for LLM
            messages = [system_prompt] + context_messages[-10:]  # Use last 10 exchanges

            # Call LLM (in practice, you'd use your API key)
            # For this example, we'll simulate the response
            response_text = self.simulate_llm_response(user_input)

            # Check if response contains an action
            if response_text.startswith("ACTION:"):
                action_part = response_text[7:].strip()  # Remove "ACTION:" prefix
                self.process_action(action_part)
                # Extract natural language response
                response_parts = response_text.split("\n", 1)
                if len(response_parts) > 1:
                    natural_response = response_parts[1]
                else:
                    natural_response = "Okay, performing that action."
            else:
                natural_response = response_text

            # Add response to context
            assistant_message = {"role": "assistant", "content": natural_response}
            self.add_to_context(assistant_message)

            return natural_response

        except Exception as e:
            self.get_logger().error(f'LLM processing error: {e}')
            return "I'm sorry, I had trouble processing that request."

    def simulate_llm_response(self, user_input):
        """Simulate LLM response for demonstration."""
        user_lower = user_input.lower()

        if "hello" in user_lower or "hi" in user_lower:
            return "Hello! I'm your autonomous humanoid assistant. How can I help you today?"
        elif "go to" in user_lower or "navigate" in user_lower:
            # Extract location
            if "kitchen" in user_lower:
                return "ACTION: navigate_to(location=\"kitchen\")\nI'm on my way to the kitchen!"
            elif "living room" in user_lower:
                return "ACTION: navigate_to(location=\"living room\")\nHeading to the living room now."
            else:
                return "I can help you navigate. Could you specify a location like kitchen or living room?"
        elif "stop" in user_lower:
            return "ACTION: emergency_stop()\nStopping now as requested."
        elif "help" in user_lower:
            return "I can help with navigation, object detection, and conversation. You can ask me to go somewhere or tell me about my surroundings."
        else:
            return "I understand. How else can I assist you?"

    def process_action(self, action_str):
        """Parse and execute action commands."""
        try:
            # Simple parsing of action commands
            if action_str.startswith("navigate_to"):
                # Extract location from navigate_to(location="kitchen")
                import re
                location_match = re.search(r'location="([^"]*)"', action_str)
                if location_match:
                    location = location_match.group(1)
                    self.publish_navigation_command(location)
            elif action_str.startswith("emergency_stop"):
                self.publish_emergency_stop()

        except Exception as e:
            self.get_logger().error(f'Action processing error: {e}')

    def publish_navigation_command(self, location):
        """Publish navigation command."""
        # In a real system, this would convert location to coordinates
        # For now, we'll publish a simple command
        cmd_msg = String()
        cmd_msg.data = f"NAVIGATE_TO:{location.upper()}"
        self.action_pub.publish(cmd_msg)
        self.get_logger().info(f'Published navigation command to {location}')

    def publish_emergency_stop(self):
        """Publish emergency stop command."""
        cmd_msg = String()
        cmd_msg.data = "EMERGENCY_STOP"
        self.action_pub.publish(cmd_msg)
        self.get_logger().info('Published emergency stop command')

    def publish_response(self, response):
        """Publish response for TTS and other systems."""
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)

        # Also publish for TTS
        tts_msg = String()
        tts_msg.data = response
        self.tts_pub.publish(tts_msg)

    def add_to_context(self, message):
        """Add message to conversation context."""
        self.conversation_history.append(message)

        # Limit context size
        if len(self.conversation_history) > self.context_window_size:
            self.conversation_history.pop(0)

    def get_context_messages(self):
        """Get context messages for LLM."""
        return self.conversation_history.copy()

def main(args=None):
    rclpy.init(args=args)

    node = InteractionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Interaction Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 5: System Integration

### 5.1 Launch File
Create a launch file to bring up the complete system:

```python
# capstone_humanoid/launch/capstone_system.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes'
    )

    ld.add_action(namespace_arg)

    # Main capstone node
    main_node = Node(
        package='capstone_humanoid',
        executable='main_node',
        name='capstone_humanoid_main',
        parameters=[
            {'use_sim_time': True},
            {'safety.linear_velocity_max': 0.5},
            {'control.loop_rate': 10.0}
        ],
        remappings=[
            ('/cmd_vel', 'cmd_vel'),
            ('/scan', 'scan'),
            ('/camera/image_raw', 'camera/image_raw')
        ]
    )

    # Perception node
    perception_node = Node(
        package='capstone_humanoid',
        executable='perception_node',
        name='capstone_perception',
        parameters=[
            {'use_sim_time': True},
            {'min_detection_confidence': 0.7}
        ],
        remappings=[
            ('/camera/image_raw', 'camera/image_raw'),
            ('/scan', 'scan')
        ]
    )

    # Navigation node
    navigation_node = Node(
        package='capstone_humanoid',
        executable='navigation_node',
        name='capstone_navigation',
        parameters=[
            {'use_sim_time': True},
            {'linear_speed': 0.3},
            {'arrival_threshold': 0.2}
        ],
        remappings=[
            ('/odom', 'odom'),
            ('/scan', 'scan'),
            ('/cmd_vel', 'cmd_vel')
        ]
    )

    # Interaction node
    interaction_node = Node(
        package='capstone_humanoid',
        executable='interaction_node',
        name='capstone_interaction',
        parameters=[
            {'use_sim_time': True}
        ],
        remappings=[
            ('/speech_recognition/result', 'speech_recognition/result'),
            ('/vision_system/description', 'vision_system/description')
        ]
    )

    # Add nodes to launch description
    ld.add_action(main_node)
    ld.add_action(perception_node)
    ld.add_action(navigation_node)
    ld.add_action(interaction_node)

    # Add logging
    ld.add_action(LogInfo(msg='Capstone Humanoid System Launched'))

    return ld
```

### 5.2 Testing and Validation

#### Unit Tests
Create unit tests for each subsystem:

```python
# test/test_capstone_nodes.py
import unittest
import rclpy
from rclpy.node import Node
from capstone_humanoid.main_node import CapstoneHumanoidNode
from capstone_humanoid.perception_node import PerceptionNode
from capstone_humanoid.navigation_node import NavigationNode
from capstone_humanoid.interaction_node import InteractionNode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TestCapstoneNodes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_main_node_initialization(self):
        """Test main node initialization."""
        node = CapstoneHumanoidNode()
        self.assertIsNotNone(node)
        self.assertEqual(node.get_name(), 'capstone_humanoid_main')
        node.destroy_node()

    def test_safety_validation(self):
        """Test safety command validation."""
        node = CapstoneHumanoidNode()

        # Test valid command
        valid_cmd = Twist()
        valid_cmd.linear.x = 0.3
        self.assertTrue(node.validate_commands(valid_cmd))

        # Test invalid command (too fast)
        invalid_cmd = Twist()
        invalid_cmd.linear.x = 10.0  # Too fast
        self.assertFalse(node.validate_commands(invalid_cmd))

        node.destroy_node()

if __name__ == '__main__':
    unittest.main()
```

## Implementation Validation

### 6.1 Simulation Testing
Test the system in Gazebo simulation:

```bash
# Build the workspace
cd ~/capstone_ws
colcon build
source install/setup.bash

# Launch Gazebo with a humanoid robot model
ros2 launch gazebo_ros empty_world.launch.py

# Spawn your robot model
ros2 run gazebo_ros spawn_entity.py -entity capstone_robot -file /path/to/robot/model.urdf

# Launch the capstone system
ros2 launch capstone_humanoid capstone_system.launch.py
```

### 6.2 Safety Validation
Perform comprehensive safety validation:

```bash
# Test emergency stop functionality
ros2 topic pub /emergency_stop std_msgs/Bool '{data: true}' --once

# Test command validation
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 10.0}}' --once

# Monitor system behavior
ros2 run rqt_plot rqt_plot
```

## Deployment Considerations

### 7.1 Hardware Requirements
- **Computing**: NVIDIA Jetson AGX Xavier or equivalent for AI/ML processing
- **Sensors**: RGB-D camera, LIDAR, IMU, microphones
- **Actuators**: Motors for locomotion and manipulation
- **Networking**: Reliable WiFi or Ethernet for communication

### 7.2 Performance Optimization
- Use lightweight neural networks for edge deployment
- Implement efficient data structures for real-time processing
- Optimize ROS 2 QoS profiles for your specific use case
- Profile and optimize CPU/GPU utilization

## Summary
The capstone implementation guide provides a systematic approach to building the autonomous conversational humanoid robot system. The phased approach ensures that each subsystem is properly implemented and validated before integration. The emphasis on safety, testing, and validation ensures a robust and reliable system suitable for human-robot interaction.

## Exercises
1. Implement a new capability for the robot (e.g., face recognition) and integrate it with the existing system
2. Create a custom behavior for a specific scenario (e.g., guided tour or object delivery)
3. Design and implement a new safety validation procedure for a specific robot behavior

## References
1. Quigley, M., et al. (2009). "ROS: an open-source Robot Operating System". ICRA Workshop.
2. MXRC Team. (2023). "ROS 2 Tutorials". Open Robotics.
3. Siciliano, B., & Khatib, O. (2016). "Springer Handbook of Robotics". Springer.
4. Thrun, S., Burgard, W., & Fox, D. (2005). "Probabilistic Robotics". MIT Press.
5. Corke, P. (2017). "Robotics, Vision and Control: Fundamental Algorithms In MATLAB". Springer.
6. ROS 2 Working Group. (2023). "ROS 2 Design Concepts". Open Robotics.
7. Smart, W. D., et al. (2010). "A Gentle Introduction to ROS". Morgan & Claypool.
8. Colomé, A., & Torras, C. (2016). "Real-time visual servoing for reaching and grasping in human environments". IEEE/RSJ IROS.

## Safety Disclaimer
Before deploying the implemented system on physical hardware, conduct comprehensive safety validation in simulation and controlled environments. Ensure all safety systems are properly tested and validated. Maintain human oversight during initial deployments and have emergency stop procedures readily available. The implementation guide provides a framework, but additional safety measures may be required based on your specific application and environment.