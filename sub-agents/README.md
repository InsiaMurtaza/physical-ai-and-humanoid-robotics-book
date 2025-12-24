# Skill-Based Sub-Agents for Physical AI & Humanoid Robotics Course

This directory contains six specialized sub-agents designed to work with the skills framework for the Physical AI & Humanoid Robotics course. Each sub-agent is focused on a single responsibility and utilizes the existing skills defined in the `/skills` folder.

## Sub-Agent Overview

### 1. Research Agent
- Conducts research and validates sources across all modules
- Uses research skills to search, extract, and summarize information
- Provides structured research findings to other agents

### 2. Technical Explanation Agent
- Breaks down complex technical concepts into understandable explanations
- Generates visual diagrams and step-by-step explanations
- Clarifies mathematical concepts and algorithms

### 3. System Design Agent
- Designs system architectures and plans hardware-software integration
- Creates system blueprints and defines interfaces
- Evaluates system performance and scalability

### 4. Code Generation Agent
- Generates production-ready code for robotic applications
- Focuses on ROS2 implementations using rclpy/rclcpp
- Implements algorithms for simulation, control, and perception systems

### 5. Control & Optimization Agent
- Designs control systems and performs dynamics analysis
- Implements motion planning algorithms and stability analysis
- Optimizes system performance and energy efficiency

### 6. Content Assembly Agent
- Creates and formats technical content for the course
- Assembles materials from various sources and agents
- Ensures consistency and quality of technical writing

## Design Principles

- **Single Responsibility**: Each agent has a focused purpose
- **Skill-Based**: Agents only utilize existing skills from the `/skills` folder
- **Module-Agnostic**: No module-specific logic embedded in agents
- **Collaborative**: Agents can work together through structured data exchange
- **Reusable**: Agents can be applied across all course modules

## Usage

The sub-agents work together to support the development of the Physical AI & Humanoid Robotics course content by leveraging the comprehensive skills framework in `/skills`. Each agent can be invoked independently or as part of a collaborative workflow depending on the task requirements.