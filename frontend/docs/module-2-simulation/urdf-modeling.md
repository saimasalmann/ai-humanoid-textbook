---
title: URDF Modeling for Simulation
description: Creating robot models in URDF format for Gazebo simulation
sidebar_position: 3
learning_objectives:
  - Create robot models using URDF (Unified Robot Description Format)
  - Define robot geometry, kinematics, and dynamics
  - Integrate URDF models with Gazebo simulation
  - Validate and test robot models in simulation
---

# URDF Modeling for Simulation

## Learning Objectives

After completing this chapter, you will be able to:

1. Create robot models using URDF (Unified Robot Description Format)
2. Define robot geometry, kinematics, and dynamics
3. Integrate URDF models with Gazebo simulation
4. Validate and test robot models in simulation

## Introduction

URDF (Unified Robot Description Format) is the standard format for representing robot models in ROS and Gazebo. It describes the physical and kinematic properties of a robot, including its links, joints, and associated properties like inertial parameters, visual geometry, and collision properties. This chapter covers how to create effective URDF models for use in Gazebo simulation.

## URDF Structure and Components

![URDF Structure](/img/urdf-structure.svg)

*Figure 1: URDF Structure showing links, joints, visual, collision, and inertial components*

A URDF file consists of several key components:

- **Links**: Rigid bodies that make up the robot structure
- **Joints**: Connections between links that define how they can move relative to each other
- **Visual**: How the robot appears in simulation and visualization
- **Collision**: How the robot interacts physically with the environment
- **Inertial**: Mass, center of mass, and inertia properties

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.2 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Link Definition

Links represent rigid bodies in the robot model. Each link can have:

- **Visual properties**: How the link appears in simulation
- **Collision properties**: How the link interacts physically
- **Inertial properties**: Mass and moment of inertia for physics simulation

### Visual and Collision Elements

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Supported shapes: box, cylinder, sphere, mesh -->
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

## Joint Definition

Joints define how links connect and move relative to each other. Common joint types include:

- **Fixed**: No movement between links
- **Revolute**: Single-axis rotation with limits
- **Continuous**: Single-axis rotation without limits
- **Prismatic**: Single-axis translation with limits
- **Floating**: Six degrees of freedom

## Gazebo-Specific Elements

To enhance URDF models for Gazebo, you can include Gazebo-specific elements:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<gazebo>
  <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_diameter>0.15</wheel_diameter>
  </plugin>
</gazebo>
```

## Best Practices for URDF Modeling

1. **Start Simple**: Begin with basic shapes and add complexity gradually
2. **Use Xacro**: For complex models, use Xacro macros to reduce redundancy
3. **Validate**: Always validate URDF files using tools like `check_urdf`
4. **Consider Physics**: Ensure inertial properties match physical reality

## Summary

URDF modeling is fundamental to robotics simulation. Well-constructed URDF models enable accurate simulation of robot kinematics, dynamics, and sensor systems. Understanding the structure and components of URDF files is essential for creating effective robot models for simulation.

## Exercises

1. Create a simple URDF model of a wheeled robot with at least 3 links and 2 joints
2. Add Gazebo-specific elements to make the robot model interactive in simulation

## References

1. ROS Documentation. (2023). "URDF: Unified Robot Description Format". Retrieved from http://wiki.ros.org/urdf
2. MXRC Team. (2023). "URDF Tutorials". Open Robotics.
3. Corke, P. (2017). "Robotics, Vision and Control: Fundamental Algorithms In MATLAB". Springer.
4. Siciliano, B., & Khatib, O. (2016). "Springer Handbook of Robotics". Springer.
5. ROS-Industrial Consortium. (2023). "URDF Best Practices".
6. Gazebo Team. (2023). "URDF Integration in Gazebo". Open Robotics.
7. Smart, W. D., et al. (2010). "Robot modeling and simulation". A Gentle Introduction to ROS.
8. Craig, J. J. (2005). "Introduction to Robotics: Mechanics and Control". Pearson.

## Safety Disclaimer

When creating URDF models for simulation, ensure that the physical properties accurately represent the intended real-world robot. Inaccurate models can lead to unrealistic simulation results that may cause unsafe conditions when applied to physical robots. Always validate simulation models against physical measurements.