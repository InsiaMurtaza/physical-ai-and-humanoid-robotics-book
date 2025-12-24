---
sidebar_label: 'Links, Joints, and Kinematic Chains'
sidebar_position: 3
---

# Links, Joints, and Kinematic Chains in URDF

## Understanding Robot Links

### Definition and Properties

Links in URDF represent rigid bodies or parts of a robot. Each link is a fundamental building block that can have:

- **Visual properties**: How the link appears in simulation and visualization
- **Collision properties**: Geometry used for collision detection
- **Inertial properties**: Mass, center of mass, and inertia tensor
- **Physical properties**: Material, color, and other visual attributes

### Link Structure

A basic link definition includes visual, collision, and inertial elements:

```xml
<link name="link_name">
  <!-- Visual representation -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Shape definition -->
      <box size="1.0 0.5 0.3"/>
      <!-- Other options: <cylinder radius="0.1" length="0.2"/> -->
      <!-- <sphere radius="0.1"/> -->
      <!-- <mesh filename="package://robot_description/meshes/link.stl"/> -->
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- Collision geometry -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1.0 0.5 0.3"/>
    </geometry>
  </collision>

  <!-- Inertial properties -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
</link>
```

### Visual vs Collision Geometry

- **Visual geometry**: Used for rendering in visualization tools (RViz, Gazebo)
- **Collision geometry**: Used for physics simulation and collision detection
- Collision geometry is often simplified for performance (e.g., bounding boxes instead of complex meshes)

## Joint Types and Configuration

### Joint Classification

Joints define how links connect and move relative to each other. The main joint types are:

#### 1. Fixed Joints
```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>
```
Fixed joints have no degrees of freedom and maintain a constant relationship between links.

#### 2. Revolute Joints
```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```
Revolute joints allow rotation around a single axis with defined limits.

#### 3. Continuous Joints
```xml
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit effort="100" velocity="1"/>
</joint>
```
Continuous joints allow unlimited rotation around a single axis (like wheels).

#### 4. Prismatic Joints
```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="100" velocity="1"/>
</joint>
```
Prismatic joints allow linear motion along a single axis.

### Joint Parameters

Each joint can specify:

- **Origin**: Position and orientation of the joint relative to the parent link
- **Axis**: Direction of the joint's motion (for revolute/prismatic joints)
- **Limits**: Range of motion, effort, and velocity constraints
- **Dynamics**: Friction and damping coefficients
- **Safety controllers**: Soft limits and velocity bounds

## Kinematic Chains Fundamentals

### Definition and Importance

A kinematic chain is a series of rigid bodies (links) connected by joints. In robotics:

- **Open chains**: Start at a base and end at an end-effector (like robot arms)
- **Closed chains**: Form loops (like parallel manipulators)
- **Tree structures**: Multiple branches from a single base (like humanoid robots)

### Forward Kinematics

Forward kinematics calculates the position and orientation of end-effectors given joint angles:

```python
# Simplified forward kinematics example
import numpy as np

def forward_kinematics(joint_angles, link_lengths):
    """
    Calculate end-effector position for a simple 2-DOF planar arm
    """
    theta1, theta2 = joint_angles
    l1, l2 = link_lengths

    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)

    return x, y
```

### Denavit-Hartenberg Parameters

While not directly used in URDF, understanding DH parameters helps in designing kinematic chains:

- **a**: Link length (distance along x-axis)
- **α**: Link twist (angle about x-axis)
- **d**: Link offset (distance along z-axis)
- **θ**: Joint angle (angle about z-axis)

## Humanoid Robot Kinematic Chains

### Typical Humanoid Structure

Humanoid robots have multiple kinematic chains that work together:

#### 1. Leg Chains (Bipedal Locomotion)
```
base_link → pelvis → hip → thigh → knee → shin → ankle → foot
```

#### 2. Arm Chains (Manipulation)
```
torso → shoulder → upper_arm → elbow → lower_arm → wrist → hand
```

#### 3. Spine Chain (Posture Control)
```
pelvis → spine_lower → spine_middle → spine_upper → neck → head
```

### Example Humanoid Leg Chain

```xml
<!-- Left leg chain -->
<joint name="left_hip_yaw_joint" type="revolute">
  <parent link="pelvis_link"/>
  <child link="left_hip_link"/>
  <origin xyz="0 0.1 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="200" velocity="2"/>
</joint>

<link name="left_hip_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
  </visual>
</link>

<joint name="left_hip_pitch_joint" type="revolute">
  <parent link="left_hip_link"/>
  <child link="left_thigh_link"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="0.5" effort="200" velocity="2"/>
</joint>

<link name="left_thigh_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
  </visual>
</link>

<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh_link"/>
  <child link="left_shin_link"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.5" effort="200" velocity="2"/>
</joint>

<link name="left_shin_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
  </visual>
</link>

<joint name="left_ankle_joint" type="revolute">
  <parent link="left_shin_link"/>
  <child link="left_foot_link"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="150" velocity="2"/>
</joint>

<link name="left_foot_link">
  <visual>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </visual>
</link>
```

### Kinematic Chain Constraints

Humanoid robots have special kinematic constraints:

1. **Balance**: Center of mass must remain within support polygon
2. **Gait planning**: Leg movements must follow coordinated patterns
3. **Obstacle avoidance**: Chains must not collide with each other
4. **Workspace limitations**: Each chain has limited reach

## Complex Kinematic Structures

### Tree Kinematics

Humanoid robots have tree-like structures with multiple branches:

```
                    torso
                   /  |  \
              neck   /   \
              |     /     \
            head   /       \
                 /         \
        left_shoulder    right_shoulder
              |              |
        left_upper_arm  right_upper_arm
              |              |
         left_forearm   right_forearm
              |              |
          left_hand     right_hand
```

### Redundant Degrees of Freedom

Humanoid robots often have redundant DOF (more joints than necessary for a task), which requires:

- **Null space projection**: Using extra DOF for secondary objectives
- **Optimization**: Minimizing energy, maximizing manipulability, etc.
- **Singularity handling**: Managing configurations where Jacobian loses rank

## URDF for Kinematic Analysis

### Joint Limits and Constraints

Properly defined joint limits in URDF are crucial for:

- **Safety**: Preventing damaging configurations
- **Workspace**: Defining reachable areas
- **Planning**: Guiding motion planning algorithms

### Mass and Inertia Properties

Accurate inertial properties are essential for:

- **Dynamics simulation**: Realistic physics behavior
- **Control algorithms**: Proper force/torque calculations
- **Balance**: Center of mass calculations

## Kinematic Chain Validation

### Checking Chain Integrity

Verify that kinematic chains are properly defined:

1. **Connectivity**: All links connected through joints
2. **No isolated links**: Every link reachable from base
3. **Correct joint types**: Appropriate for intended motion
4. **Consistent naming**: Joint and link names follow conventions

### Tools for Validation

```bash
# Check URDF structure
check_urdf robot.urdf

# Generate kinematic chain graph
urdf_to_graphviz robot.urdf
dot -Tpng robot.gv -o robot.png
```

## Advanced Kinematic Concepts

### Parallel Mechanisms

Some humanoid designs include parallel mechanisms:

```xml
<!-- Example: Parallel ankle mechanism -->
<joint name="ankle_1_joint" type="revolute">
  <parent link="shin_link"/>
  <child link="ankle_plate_link"/>
  <axis xyz="0 1 0"/>
</joint>

<joint name="ankle_2_joint" type="revolute">
  <parent link="shin_link"/>
  <child link="ankle_plate_link"/>
  <axis xyz="1 0 0"/>
</joint>
```

### Coupled Joints

For realistic human-like movement:

```xml
<!-- Example: Coupled finger joints -->
<transmission name="finger_coupling">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="finger_proximal_joint">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
  </joint>
  <joint name="finger_distal_joint">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
  </joint>
  <mechanicalReduction>1</mechanicalReduction>
</transmission>
```

## Best Practices for Kinematic Chain Design

1. **Start simple**: Begin with basic chains, add complexity gradually
2. **Use consistent naming**: Follow ROS conventions (e.g., `joint_name` not `JointName`)
3. **Validate early**: Check chains with URDF tools during development
4. **Consider symmetry**: Use Xacro macros for symmetrical structures
5. **Plan for control**: Design chains that are controllable with available actuators
6. **Think about simulation**: Balance visual quality with simulation performance

## References

<div class="reference-list">

- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson Prentice Hall.
- Siciliano, B., & Khatib, O. (2016). *Springer handbook of robotics*. Springer Publishing Company.
- Corke, P. (2017). *Robotics, Vision and Control: Fundamental Algorithms in MATLAB* (2nd ed.). Springer.

</div>