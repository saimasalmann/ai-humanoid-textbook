#!/usr/bin/env python3

"""
Vision-Language-Action Pipeline Example

This example demonstrates a basic VLA (Vision-Language-Action) pipeline
that integrates perception, language understanding, and action execution.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import json


class VLAPipeline(Node):
    """
    Vision-Language-Action pipeline node that integrates
    perception, language understanding, and action planning.
    """

    def __init__(self):
        """
        Initialize the VLA pipeline node.
        """
        super().__init__('vla_pipeline')

        # Initialize components
        self.bridge = CvBridge()

        # Subscribe to camera feed
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

        # Subscribe to language commands
        self.command_sub = self.create_subscription(
            String, 'language_command', self.command_callback, 10)

        # Publisher for robot actions
        self.action_pub = self.create_publisher(
            String, 'robot_action', 10)

        # Publisher for vision results
        self.vision_pub = self.create_publisher(
            String, 'vision_results', 10)

        # Internal state
        self.current_image = None
        self.pending_command = None

        self.get_logger().info('VLA Pipeline node initialized')

    def image_callback(self, msg):
        """
        Callback for processing camera images.
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image

            # Perform basic vision processing
            vision_results = self.process_vision(cv_image)

            # Publish vision results
            vision_msg = String()
            vision_msg.data = json.dumps(vision_results)
            self.vision_pub.publish(vision_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def command_callback(self, msg):
        """
        Callback for processing language commands.
        """
        try:
            command = msg.data
            self.pending_command = command

            # Process command if we have a recent image
            if self.current_image is not None:
                self.process_command_and_image(command, self.current_image)

        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    def process_vision(self, image):
        """
        Process image to extract relevant information.
        """
        # Simple example: detect objects using basic computer vision
        # In a real system, this would use deep learning models

        height, width = image.shape[:2]

        # For demonstration, return some mock object detections
        objects = [
            {
                "name": "cup",
                "bbox": [int(width * 0.3), int(height * 0.4), int(width * 0.4), int(height * 0.5)],
                "confidence": 0.85
            },
            {
                "name": "apple",
                "bbox": [int(width * 0.6), int(height * 0.3), int(width * 0.7), int(height * 0.4)],
                "confidence": 0.78
            }
        ]

        return {
            "timestamp": self.get_clock().now().nanoseconds,
            "objects": objects,
            "image_shape": [height, width]
        }

    def process_command_and_image(self, command, image):
        """
        Integrate language command with visual information to plan actions.
        """
        # Parse the command (simplified example)
        command_lower = command.lower()

        if "pick" in command_lower or "grasp" in command_lower:
            # Find the target object in the image
            vision_results = self.process_vision(image)
            target_object = self.find_target_object(command, vision_results["objects"])

            if target_object:
                action_plan = self.plan_pick_action(target_object, vision_results["image_shape"])
            else:
                action_plan = {"error": "Target object not found", "command": command}

        elif "move" in command_lower or "go to" in command_lower:
            action_plan = self.plan_navigation_action(command)

        else:
            action_plan = {"error": "Unknown command type", "command": command}

        # Publish the action plan
        action_msg = String()
        action_msg.data = json.dumps(action_plan)
        self.action_pub.publish(action_msg)

    def find_target_object(self, command, objects):
        """
        Find the object in the image that matches the command.
        """
        command_lower = command.lower()

        # Simple keyword matching for demonstration
        for obj in objects:
            if obj["name"] in command_lower:
                return obj

        # If no direct match, return the first object as fallback
        return objects[0] if objects else None

    def plan_pick_action(self, target_object, image_shape):
        """
        Plan a pick action for the target object.
        """
        # Calculate 2D position from bounding box
        bbox = target_object["bbox"]
        x = (bbox[0] + bbox[2]) / 2  # center x
        y = (bbox[1] + bbox[3]) / 2  # center y

        # Convert to normalized coordinates
        norm_x = x / image_shape[1]
        norm_y = y / image_shape[0]

        return {
            "action_type": "pick",
            "target_object": target_object["name"],
            "normalized_position": [norm_x, norm_y],
            "confidence": target_object["confidence"]
        }

    def plan_navigation_action(self, command):
        """
        Plan a navigation action based on the command.
        """
        # Simplified navigation planning
        # In a real system, this would use more sophisticated path planning

        if "kitchen" in command.lower():
            target_location = "kitchen"
        elif "living room" in command.lower():
            target_location = "living_room"
        elif "bedroom" in command.lower():
            target_location = "bedroom"
        else:
            target_location = "default_location"

        return {
            "action_type": "navigate",
            "target_location": target_location,
            "command": command
        }


def main(args=None):
    """
    Main function to initialize ROS 2, create the node, and spin to execute callbacks.
    """
    rclpy.init(args=args)

    vla_pipeline = VLAPipeline()

    try:
        # Keep the node running to execute callbacks
        rclpy.spin(vla_pipeline)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up resources
        vla_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()