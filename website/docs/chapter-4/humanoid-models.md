---
sidebar_label: 'Building Minimal Humanoid URDF'
sidebar_position: 4
---

# Building Minimal Humanoid URDF Models

## Introduction to Humanoid Robot Modeling

Creating a humanoid robot model in URDF requires careful consideration of the robot's structure, kinematic chains, and physical properties. A minimal humanoid model typically includes the essential components needed for basic locomotion and manipulation while maintaining simplicity for simulation and control.

## Minimal Humanoid Structure

### Essential Components

A minimal humanoid robot includes:

1. **Torso/Body**: Central trunk connecting limbs
2. **Head**: With appropriate joint for neck movement
3. **Arms**: With shoulder, elbow, and wrist joints
4. **Legs**: With hip, knee, and ankle joints
5. **End Effectors**: Simple hands and feet

### Basic Design Philosophy

The minimal approach focuses on:
- **Functionality**: Including only necessary joints for intended tasks
- **Stability**: Proper center of mass and balance considerations
- **Simplicity**: Avoiding overly complex geometries
- **Scalability**: Easy to extend with additional features

## Complete Minimal Humanoid URDF Example

Here's a complete minimal humanoid robot URDF model:

```xml
<?xml version="1.0"?>
<robot name="minimal_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Material definitions -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 1.0"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 1.0"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.2083" ixy="0.0" ixz="0.0" iyy="0.2083" iyz="0.0" izz="0.0167"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
  </joint>

  <link name="torso_link">
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.0792" ixy="0.0" ixz="0.0" iyy="0.1354" iyz="0.0" izz="0.1875"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="head_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="left_upper_arm_link"/>
    <origin xyz="0.2 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.0075" ixy="0.0" ixz="0.0" iyy="0.0075" iyz="0.0" izz="0.00125"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm_link"/>
    <child link="left_forearm_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
  </joint>

  <link name="left_forearm_link">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.0042" ixy="0.0" ixz="0.0" iyy="0.0042" iyz="0.0" izz="0.00064"/>
    </inertial>
  </link>

  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_forearm_link"/>
    <child link="left_hand_link"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="50.0" velocity="1.0"/>
  </joint>

  <link name="left_hand_link">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.08"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.00028" ixy="0.0" ixz="0.0" iyy="0.00025" iyz="0.0" izz="0.00025"/>
    </inertial>
  </link>

  <!-- Right Arm (symmetric to left) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="right_upper_arm_link"/>
    <origin xyz="-0.2 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
  </joint>

  <link name="right_upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.0075" ixy="0.0" ixz="0.0" iyy="0.0075" iyz="0.0" izz="0.00125"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm_link"/>
    <child link="right_forearm_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
  </joint>

  <link name="right_forearm_link">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.0042" ixy="0.0" ixz="0.0" iyy="0.0042" iyz="0.0" izz="0.00064"/>
    </inertial>
  </link>

  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_forearm_link"/>
    <child link="right_hand_link"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="50.0" velocity="1.0"/>
  </joint>

  <link name="right_hand_link">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.08"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.00028" ixy="0.0" ixz="0.0" iyy="0.00025" iyz="0.0" izz="0.00025"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh_link"/>
    <origin xyz="0 0.1 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.5" effort="200.0" velocity="1.0"/>
  </joint>

  <link name="left_thigh_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.0267" ixy="0.0" ixz="0.0" iyy="0.0267" iyz="0.0" izz="0.0036"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh_link"/>
    <child link="left_shin_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="200.0" velocity="1.0"/>
  </joint>

  <link name="left_shin_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.003125"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_shin_link"/>
    <child link="left_foot_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="100.0" velocity="1.0"/>
  </joint>

  <link name="left_foot_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0007" ixy="0.0" ixz="0.0" iyy="0.0021" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Right Leg (symmetric to left) -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_thigh_link"/>
    <origin xyz="0 -0.1 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.5" effort="200.0" velocity="1.0"/>
  </joint>

  <link name="right_thigh_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="brown"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.0267" ixy="0.0" ixz="0.0" iyy="0.0267" iyz="0.0" izz="0.0036"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh_link"/>
    <child link="right_shin_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="200.0" velocity="1.0"/>
  </joint>

  <link name="right_shin_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="brown"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.003125"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_shin_link"/>
    <child link="right_foot_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="100.0" velocity="1.0"/>
  </joint>

  <link name="right_foot_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0007" ixy="0.0" ixz="0.0" iyy="0.0021" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

</robot>
```

## Xacro-Based Humanoid Model (Advanced)

For more complex humanoid robots, using Xacro (XML Macros) makes the model more maintainable:

```xml
<?xml version="1.0"?>
<robot name="xacro_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="0.5" />
  <xacro:property name="torso_width" value="0.3" />
  <xacro:property name="torso_depth" value="0.2" />

  <!-- Materials -->
  <xacro:include filename="materials.xacro" />

  <!-- Macro for arm chain -->
  <xacro:macro name="humanoid_arm" params="side parent *origin">
    <!-- Shoulder -->
    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_upper_arm_link"/>
      <xacro:insert_block name="origin"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
    </joint>

    <link name="${side}_upper_arm_link">
      <visual>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.0075" ixy="0.0" ixz="0.0" iyy="0.0075" iyz="0.0" izz="0.00125"/>
      </inertial>
    </link>

    <!-- Elbow -->
    <joint name="${side}_elbow_joint" type="revolute">
      <parent link="${side}_upper_arm_link"/>
      <child link="${side}_forearm_link"/>
      <origin xyz="0 0 -0.3" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
    </joint>

    <link name="${side}_forearm_link">
      <visual>
        <geometry>
          <cylinder length="0.25" radius="0.04"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder length="0.25" radius="0.04"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.8"/>
        <inertia ixx="0.0042" ixy="0.0" ixz="0.0" iyy="0.0042" iyz="0.0" izz="0.00064"/>
      </inertial>
    </link>

    <!-- Wrist -->
    <joint name="${side}_wrist_joint" type="revolute">
      <parent link="${side}_forearm_link"/>
      <child link="${side}_hand_link"/>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="50" velocity="1"/>
    </joint>

    <link name="${side}_hand_link">
      <visual>
        <geometry>
          <box size="0.1 0.08 0.08"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box size="0.1 0.08 0.08"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.3"/>
        <inertia ixx="0.00028" ixy="0.0" ixz="0.0" iyy="0.00025" iyz="0.0" izz="0.00025"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Macro for leg chain -->
  <xacro:macro name="humanoid_leg" params="side parent *origin">
    <!-- Hip -->
    <joint name="${side}_hip_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_thigh_link"/>
      <xacro:insert_block name="origin"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/2}" upper="0.5" effort="200" velocity="1"/>
    </joint>

    <link name="${side}_thigh_link">
      <visual>
        <geometry>
          <cylinder length="0.4" radius="0.06"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder length="0.4" radius="0.06"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="2.0"/>
        <inertia ixx="0.0267" ixy="0.0" ixz="0.0" iyy="0.0267" iyz="0.0" izz="0.0036"/>
      </inertial>
    </link>

    <!-- Knee -->
    <joint name="${side}_knee_joint" type="revolute">
      <parent link="${side}_thigh_link"/>
      <child link="${side}_shin_link"/>
      <origin xyz="0 0 -0.4" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="${M_PI/2}" effort="200" velocity="1"/>
    </joint>

    <link name="${side}_shin_link">
      <visual>
        <geometry>
          <cylinder length="0.4" radius="0.05"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder length="0.4" radius="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.003125"/>
      </inertial>
    </link>

    <!-- Ankle -->
    <joint name="${side}_ankle_joint" type="revolute">
      <parent link="${side}_shin_link"/>
      <child link="${side}_foot_link"/>
      <origin xyz="0 0 -0.4" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="100" velocity="1"/>
    </joint>

    <link name="${side}_foot_link">
      <visual>
        <geometry>
          <box size="0.2 0.1 0.05"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box size="0.2 0.1 0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.0007" ixy="0.0" ixz="0.0" iyy="0.0021" iyz="0.0" izz="0.0025"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Base and torso -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 1.0"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 1.0"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.2083" ixy="0.0" ixz="0.0" iyy="0.2083" iyz="0.0" izz="0.0167"/>
    </inertial>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
  </joint>

  <link name="torso_link">
    <visual>
      <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.0792" ixy="0.0" ixz="0.0" iyy="0.1354" iyz="0.0" izz="0.1875"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 ${torso_height}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="10" velocity="1"/>
  </joint>

  <link name="head_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Instantiate arms -->
  <xacro:humanoid_arm side="left" parent="torso_link">
    <origin xyz="${torso_width/2} 0 ${torso_height*0.6}" rpy="0 0 0"/>
  </xacro:humanoid_arm>

  <xacro:humanoid_arm side="right" parent="torso_link">
    <origin xyz="${-torso_width/2} 0 ${torso_height*0.6}" rpy="0 0 0"/>
  </xacro:humanoid_arm>

  <!-- Instantiate legs -->
  <xacro:humanoid_leg side="left" parent="base_link">
    <origin xyz="0 ${torso_depth/2+0.01} -0.5" rpy="0 0 0"/>
  </xacro:humanoid_leg>

  <xacro:humanoid_leg side="right" parent="base_link">
    <origin xyz="0 ${-torso_depth/2-0.01} -0.5" rpy="0 0 0"/>
  </xacro:humanoid_leg>

</robot>
```

## Design Considerations for Humanoid URDF

### Balance and Stability

When designing humanoid robots, consider:

1. **Center of Mass**: Keep it low and within the support polygon
2. **Foot Size**: Adequate for stable standing and walking
3. **Joint Limits**: Realistic for human-like movement
4. **Mass Distribution**: Appropriate for dynamic behavior

### Simulation Performance

For efficient simulation:

- **Simplified Collision Geometry**: Use basic shapes instead of complex meshes
- **Appropriate Mass Values**: Realistic but not overly heavy
- **Conservative Joint Limits**: Prevent impossible configurations
- **Damping Values**: Add small amounts for stability

### Control Considerations

For effective control:

- **Actuator Limits**: Set realistic effort and velocity limits
- **Sensor Placement**: Consider where to place IMUs, force sensors
- **Kinematic Chains**: Ensure they're solvable for inverse kinematics

## Humanoid-Specific Features

### Bipedal Locomotion

Humanoid robots require special considerations for walking:

```xml
<!-- Gazebo-specific tags for physics simulation -->
<gazebo reference="left_foot_link">
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <material>Gazebo/White</material>
</gazebo>
```

### Manipulation Capabilities

For manipulation tasks, consider:

- **Dexterity**: More joints in hands for grasping
- **Payload**: Appropriate mass limits for carrying objects
- **Workspace**: Ensure arms can reach desired areas

## Validation and Testing

### URDF Validation

Before simulation, validate your URDF:

```bash
# Check for syntax errors
check_urdf minimal_humanoid.urdf

# Generate a graph of the kinematic structure
urdf_to_graphviz minimal_humanoid.urdf
dot -Tpng minimal_humanoid.gv -o minimal_humanoid.png
```

### Visualization in RViz

Load your robot model in RViz to check visual appearance:

1. Set Fixed Frame to `base_link` or `map`
2. Add RobotModel display
3. Check that all links appear correctly
4. Verify joint relationships

## Extending the Minimal Model

The minimal model can be extended with:

- **More detailed meshes**: Replace basic shapes with STL files
- **Additional sensors**: IMU, cameras, force/torque sensors
- **Actuators**: Proper transmission elements
- **Gazebo plugins**: For simulation control
- **Semantic information**: SRDF files for MoveIt!

## Best Practices

1. **Start Simple**: Begin with the minimal model and add complexity gradually
2. **Use Xacro**: For humanoid robots, Xacro makes models much more maintainable
3. **Symmetry**: Use macros to ensure left/right symmetry
4. **Realistic Values**: Use physically plausible mass and inertia values
5. **Validation**: Regularly check your model with URDF tools
6. **Documentation**: Comment your URDF files to explain the structure
7. **Modularity**: Organize URDF into separate files for different body parts

## References

<div class="reference-list">

- Kajita, S. (2019). *Humanoid Robotics: A Reference*. Springer Publishing Company.
- Sreenath, K., Park, H. W., & Koditschek, D. E. (2013). A compliant hybrid zero dynamics controller for stable, efficient and robust running on MABEL. The International Journal of Robotics Research, 32(14), 1655-1676.
- Open Robotics. (2023). *URDF Tutorials*. Retrieved from https://docs.ros.org/

</div>