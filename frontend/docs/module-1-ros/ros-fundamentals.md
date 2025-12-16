---
title: ROS 2 Fundamentals
description: Understanding the core concepts of Robot Operating System 2
sidebar_position: 2
learning_objectives:
  - Explain the ROS 2 architecture and its components
  - Identify different communication patterns in ROS 2
  - Describe the role of nodes, topics, services, and actions
  - Understand the ROS 2 launch system
---

# ROS 2 Fundamentals

## Learning Objectives

After completing this chapter, you will be able to:

1. Explain the ROS 2 architecture and its components
2. Identify different communication patterns in ROS 2
3. Describe the role of nodes, topics, services, and actions
4. Understand the ROS 2 launch system

## Introduction

Robot Operating System 2 (ROS 2) is the next generation of the Robot Operating System, designed to address the limitations of ROS 1 and provide a more robust, scalable, and production-ready framework for robotics development. Unlike its predecessor, ROS 2 is built on DDS (Data Distribution Service) for communication, providing better support for real-time systems and multi-robot applications.

This chapter will introduce you to the fundamental concepts of ROS 2 that form the foundation for all robotic applications in this textbook.

## ROS 2 Architecture

![ROS 2 Architecture](/img/ros2-architecture.svg)

*Figure 1: ROS 2 Architecture showing nodes, topics, services, and actions*

ROS 2 uses a distributed computing architecture based on the DDS (Data Distribution Service) middleware. This architecture provides:

- **Nodes**: Processes that perform computation and communicate with other nodes
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Synchronous request/response communication between nodes
- **Actions**: Asynchronous communication for long-running tasks with feedback
- **Parameters**: Configuration values that can be set at runtime

### Nodes in ROS 2

A node is the basic unit of execution in ROS 2. Each node runs in its own process and communicates with other nodes through the ROS 2 communication layer. Nodes are organized in a graph structure where they can exchange messages, provide services, or execute actions.

```python
# Example ROS 2 node structure
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()
    rclpy.spin(minimal_node)
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Communication Patterns

ROS 2 provides several communication patterns to suit different use cases:

### Topics (Publish/Subscribe)

Topics enable asynchronous, one-way communication between nodes using a publish/subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics they are subscribed to.

### Services (Request/Response)

Services provide synchronous, two-way communication where a client sends a request to a server and waits for a response. This pattern is useful for operations that require immediate results.

### Actions (Goal/Result/Feedback)

Actions are designed for long-running tasks that require goal setting, feedback during execution, and final results. They combine features of both topics and services.

## Safety Disclaimer

When implementing ROS 2 systems on physical robots, always follow proper safety protocols:
- Test all code in simulation before deploying to physical hardware
- Implement proper safety checks and emergency stop mechanisms
- Ensure proper safety-rated hardware for critical functions
- Follow all applicable safety standards for your specific robot platform

## Summary

ROS 2 fundamentals form the backbone of robotic applications. Understanding nodes, topics, services, and actions is essential for building robust robotic systems. The DDS-based architecture provides improved reliability and scalability compared to ROS 1.

## Exercises

1. Create a simple ROS 2 node that prints "Hello, ROS 2!" when executed
2. Explain the difference between topics, services, and actions with examples from robotics applications

## References

1. ROS 2 Documentation. (2023). "Concepts: About ROS 2". Retrieved from https://docs.ros.org/en/humble/Concepts/About-ROS-2.html
2. Faconti, P., et al. (2019). "ROS 2 Design Overview". Open Robotics.
3. Quigley, M., et al. (2009). "ROS: an open-source Robot Operating System". ICRA Workshop.
4. DDS Foundation. (2023). "Data Distribution Service for Real-Time Systems". Object Management Group.
5. MXRC Team. (2023). "ROS 2 Tutorials". Open Robotics.
6. ROS 2 Working Group. (2023). "ROS 2 Architecture". Open Robotics.
7. Smart, W. D., et al. (2010). "The second international competition in robotics research". Autonomous Robots, 28(2), 141-145.
8. Colomé, A., et al. (2016). "Real-time visual servoing for reaching and grasping in human environments". IEEE/RSJ IROS.