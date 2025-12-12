---
title: First Robot
description: Building your first ROS 2 robot node and running it in simulation
sidebar_position: 3
learning_objectives:
  - Create a basic robot package in ROS 2
  - Implement a simple robot controller
  - Launch the robot in simulation
  - Verify robot functionality
---

# First Robot

## Learning Objectives

After completing this tutorial, you will be able to:

1. Create a basic robot package in ROS 2
2. Implement a simple robot controller
3. Launch the robot in simulation
4. Verify robot functionality

## Introduction

This tutorial guides you through creating your first robot using ROS 2. You'll build a simple differential drive robot that can move forward, turn, and respond to basic commands. This foundational example introduces key concepts that will be expanded upon in later modules.

## Prerequisites

Before starting this tutorial, ensure you have:
- ROS 2 Humble Hawksbill installed
- Basic understanding of Linux command line
- Python programming knowledge
- Completed the Setup Guide tutorial

## Creating the Robot Package

### 1. Create the Workspace

First, create a workspace for your robot project:

```bash
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws
```

### 2. Create the Robot Package

Use `ros2 pkg create` to create a new package:

```bash
cd ~/robotics_ws/src
ros2 pkg create --build-type ament_python first_robot --dependencies rclpy geometry_msgs std_msgs sensor_msgs
```

### 3. Package Structure

After creating the package, your structure should look like:

```
first_robot/
├── first_robot/
│   ├── __init__.py
│   └── first_robot.py
├── setup.cfg
├── setup.py
├── package.xml
└── resource/
```

## Robot Controller Implementation

### 1. Create the Robot Controller Node

Edit the main robot file at `first_robot/first_robot/first_robot.py`:

```python
#!/usr/bin/env python3

"""
First Robot Controller

This example demonstrates a simple robot controller that moves forward
and turns when obstacles are detected.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class FirstRobotController(Node):
    """
    A simple robot controller that moves forward and avoids obstacles.
    """

    def __init__(self):
        """
        Initialize the robot controller node.
        """
        super().__init__('first_robot_controller')

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.obstacle_angle = 0.0

        # Control parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.safe_distance = 0.5  # meters

        self.get_logger().info('First Robot Controller initialized')

    def scan_callback(self, msg):
        """
        Callback function to process laser scan data.
        """
        # Process scan data to detect obstacles
        if len(msg.ranges) > 0:
            # Get distances in front of the robot (forward 30 degrees)
            front_ranges = []
            center_index = len(msg.ranges) // 2

            # Sample ranges around the center (front of robot)
            sample_range = 15  # Number of rays to sample
            start_idx = max(0, center_index - sample_range // 2)
            end_idx = min(len(msg.ranges), center_index + sample_range // 2)

            for i in range(start_idx, end_idx):
                if not math.isinf(msg.ranges[i]) and msg.ranges[i] > 0:
                    front_ranges.append(msg.ranges[i])

            if front_ranges:
                self.obstacle_distance = min(front_ranges)
                self.obstacle_detected = self.obstacle_distance < self.safe_distance
            else:
                self.obstacle_distance = float('inf')
                self.obstacle_detected = False

    def control_loop(self):
        """
        Main control loop that determines robot behavior.
        """
        cmd_vel = Twist()

        if self.obstacle_detected:
            # Turn away from obstacle
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = self.angular_speed
            self.get_logger().info(f'Obstacle detected at {self.obstacle_distance:.2f}m, turning...')
        else:
            # Move forward
            cmd_vel.linear.x = self.linear_speed
            cmd_vel.angular.z = 0.0
            self.get_logger().info(f'Moving forward, obstacle distance: {self.obstacle_distance:.2f}m')

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    """
    Main function to initialize ROS 2, create the node, and spin to execute callbacks.
    """
    rclpy.init(args=args)

    robot_controller = FirstRobotController()

    try:
        # Keep the node running to execute callbacks
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up resources
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Update setup.py

Edit the `setup.py` file to make the script executable:

```python
from setuptools import find_packages, setup

package_name = 'first_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='First robot example package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'first_robot = first_robot.first_robot:main',
        ],
    },
)
```

## Creating the Robot Model

### 1. Create URDF Model

Create a URDF file to define your robot's physical properties. Create `first_robot/urdf/first_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="first_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_to_wheel_left" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="base_to_wheel_right" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="wheel_left">
    <material>Gazebo/Black</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="wheel_right">
    <material>Gazebo/Black</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <!-- Differential drive plugin -->
  <gazebo>
    <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>base_to_wheel_left</left_joint>
      <right_joint>base_to_wheel_right</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

  <!-- Laser scanner -->
  <link name="laser_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="base_to_laser" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.15 0 0.05" rpy="0 0 0"/>
  </joint>

  <gazebo reference="laser_link">
    <sensor type="ray" name="laser_scan">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_scan" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### 2. Create Launch File

Create a launch file at `first_robot/launch/first_robot.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_launch_package_share_directory
import os


def generate_launch_description():
    # Get the launch directory
    pkg_gazebo_ros = get_launch_package_share_directory('gazebo_ros')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(os.path.join(
                get_launch_package_share_directory('first_robot'),
                'urdf',
                'first_robot.urdf'
            )).read()
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'first_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Robot controller node
    robot_controller = Node(
        package='first_robot',
        executable='first_robot',
        name='first_robot_controller',
        output='screen'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        robot_controller
    ])
```

## Building and Running the Robot

### 1. Build the Package

```bash
cd ~/robotics_ws
colcon build --packages-select first_robot
source install/setup.bash
```

### 2. Run the Robot

To run the complete system with Gazebo simulation:

```bash
# Terminal 1: Start Gazebo and spawn the robot
ros2 launch first_robot first_robot.launch.py
```

In another terminal, you can check the topics:

```bash
# Terminal 2: List ROS 2 topics
ros2 topic list
```

You should see topics like `/cmd_vel` and `/scan` among others.

## Verification

### 1. Check Robot Status

```bash
# Check if the robot node is running
ros2 node list

# Check topic connections
ros2 topic echo /odom
```

### 2. Send Manual Commands (Optional)

In another terminal, you can send manual velocity commands:

```bash
# Send a velocity command to make the robot move
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

## Troubleshooting

### Common Issues

1. **Robot doesn't appear in Gazebo**: Check that the URDF file is valid and the robot is properly spawned
2. **No laser scan data**: Verify the laser plugin is correctly configured in URDF
3. **Robot doesn't respond to commands**: Check topic connections with `ros2 topic list` and `ros2 topic info`

### Debugging Commands

```bash
# Check if nodes are communicating
ros2 run rqt_graph rqt_graph

# Monitor laser scan data
ros2 topic echo /scan

# Check robot tf frames
ros2 run tf2_tools view_frames
```

## Summary

This tutorial introduced you to creating a basic robot in ROS 2. You learned how to:
- Create a ROS 2 package for your robot
- Implement a simple robot controller
- Define a robot model in URDF
- Integrate the robot with Gazebo simulation
- Launch and verify the complete system

This foundation will be expanded upon in later modules as you develop more sophisticated robotic capabilities.

## Exercises

1. Modify the robot controller to change the speed parameters and observe the behavior
2. Add a second robot to the simulation with different starting positions
3. Create a simple navigation goal for the robot to reach

## References

1. ROS 2 Documentation. (2023). "Creating a ROS 2 Package". Retrieved from https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
2. ROS 2 Documentation. (2023). "Writing a simple publisher and subscriber". Retrieved from https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
3. Open Robotics. (2023). "Robot State Publisher". Retrieved from http://wiki.ros.org/robot_state_publisher
4. OSRF. (2023). "Gazebo ROS Integration". Retrieved from http://gazebosim.org/tutorials?tut=ros2_overview
5. MXRC Team. (2023). "ROS 2 Best Practices". Open Robotics.
6. Corke, P. (2017). "Robotics, Vision and Control: Fundamental Algorithms In MATLAB". Springer.
7. Siciliano, B., & Khatib, O. (2016). "Springer Handbook of Robotics". Springer.
8. ROS-Industrial Consortium. (2023). "Industrial Robot Programming Guide".

## Safety Disclaimer

When running robot simulations, always ensure that your safety parameters (like safe_distance in the example) are appropriate for your specific use case. The example code provides a basic obstacle avoidance behavior, but real-world applications may require more sophisticated safety systems. Always test robot behaviors thoroughly in simulation before considering physical deployment.