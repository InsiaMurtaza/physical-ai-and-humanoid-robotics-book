---
sidebar_label: 'What is ROS 2?'
sidebar_position: 2
---

# What is ROS 2?

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is not an operating system in the traditional sense, but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms, applications, and research areas.

## The Evolution from ROS 1 to ROS 2

ROS 1 was developed to address the challenges of creating complex robotic systems by providing:

- A communication infrastructure for distributed systems
- Hardware abstraction
- Device drivers
- Libraries for common robot functionality
- Tools for visualization, simulation, and testing

However, as robotics applications evolved and expanded into areas requiring higher levels of safety, security, and performance, the limitations of ROS 1 became apparent:

- Single master architecture creating a single point of failure
- Lack of real-time support
- Limited security features
- Challenges with multi-robot systems
- Difficulties with closed-loop control in distributed systems

## Core Architecture of ROS 2

ROS 2 addresses these limitations through a distributed architecture based on the Data Distribution Service (DDS) standard. This architecture provides:

- **Decentralized communication**: No single master, nodes can discover each other directly
- **Language and platform independence**: Support for multiple programming languages and operating systems
- **Real-time support**: Deterministic behavior for time-critical applications
- **Security features**: Authentication, encryption, and access control
- **Quality of Service (QoS) policies**: Configurable communication behavior for different requirements

## ROS 2 as a "Robotic Nervous System"

The concept of ROS 2 as a "robotic nervous system" refers to its role in connecting various components of a robot system, much like how the biological nervous system connects different parts of an organism. This metaphor emphasizes:

- **Sensory Integration**: Collecting data from various sensors (cameras, lidars, IMUs, etc.)
- **Information Processing**: Analyzing sensor data and making decisions
- **Motor Control**: Coordinating actuator movements based on processed information
- **Communication**: Enabling different subsystems to exchange information reliably

## Key Concepts in ROS 2

### Nodes
Nodes are the fundamental units of computation in ROS 2. Each node runs a specific task or set of tasks and communicates with other nodes through messages. Nodes can be written in different programming languages and run on different machines.

### Packages
Packages are the basic building and distribution units in ROS 2. A package contains source code, data, and configuration files that provide specific functionality. Packages can depend on other packages and are managed using the ROS 2 build system.

### Workspaces
Workspaces are directories where ROS 2 packages are built and organized. A workspace typically contains a `src` directory where source packages are placed, and build and install directories where compiled code is placed.

## Why ROS 2 Exists

ROS 2 exists to solve several critical challenges in robotics development:

1. **Complexity Management**: Building robots requires coordinating many different components and subsystems. ROS 2 provides a framework for managing this complexity.

2. **Reusability**: ROS 2 enables the sharing of robot software components across different robots and applications, reducing development time and effort.

3. **Standardization**: By providing common interfaces and communication patterns, ROS 2 facilitates collaboration between researchers and developers.

4. **Scalability**: The distributed architecture of ROS 2 allows for the development of complex multi-robot systems.

5. **Industry Adoption**: The improved reliability, security, and real-time capabilities of ROS 2 make it suitable for commercial applications.

## References

<div class="reference-list">

- Foote, T., Lalancette, C., & Perez, A. (2019). *ROS 2 Design Overview*. Open Robotics.
- Quigley, M., Gerkey, B., & Smart, W. D. (2015). Programming robots with ROS: A practical introduction to the Robot Operating System. O'Reilly Media.
- Siciliano, B., & Khatib, O. (2016). Springer handbook of robotics. Springer Publishing Company.

</div>