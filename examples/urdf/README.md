# Humanoid Robot URDF Model

This directory contains a complete validated humanoid URDF model that demonstrates the principles of humanoid robot modeling in ROS.

## The Model: minimal_humanoid.urdf

The `humanoid.urdf` file contains a complete minimal humanoid robot model with:

- **Torso**: Central body with head
- **Arms**: 3-DOF arms (shoulder, elbow, wrist) with simple hands
- **Legs**: 3-DOF legs (hip, knee, ankle) with flat feet
- **Proper inertial properties**: Realistic mass and inertia values
- **Collision geometry**: Matching visual geometry for simulation

## Validation Process

### 1. URDF Syntax Validation

Validate the URDF file structure:

```bash
# Check the URDF syntax
check_urdf humanoid.urdf
```

Expected output should show successful parsing with all links and joints properly connected.

### 2. Kinematic Structure Validation

Generate and view the kinematic tree:

```bash
# Generate the kinematic graph
urdf_to_graphviz humanoid.urdf

# Convert to image (requires graphviz installed)
dot -Tpng humanoid.gv -o humanoid.png
```

The kinematic tree should show a proper tree structure with `base_link` as the root and all other links properly connected through joints.

### 3. Physical Properties Validation

Verify that all physical properties are realistic:

- All masses are positive
- Inertia values follow physical laws
- Link dimensions are appropriate for a humanoid
- Joint limits are reasonable for human-like movement

### 4. Simulation Readiness Check

```bash
# Load in RViz to check visual appearance
ros2 run rviz2 rviz2

# Then add RobotModel display and load the URDF
```

## Expected Validation Results

When you run `check_urdf humanoid.urdf`, you should see output similar to:

```
robot name is: minimal_humanoid
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
  child(1):  torso_link
    child(1):  head_link
    child(2):  left_upper_arm_link
      child(1):  left_forearm_link
        child(1):  left_hand_link
    child(3):  right_upper_arm_link
      child(1):  right_forearm_link
        child(1):  right_hand_link
    child(4):  left_thigh_link
      child(1):  left_shin_link
        child(1):  left_foot_link
    child(5):  right_thigh_link
      child(1):  right_shin_link
        child(1):  right_foot_link
```

## Using the Model

### In Simulation

```bash
# Launch with Gazebo (if Gazebo is installed)
gz sim -r -v 4 humanoid.urdf
```

### With Robot State Publisher

```bash
# Publish the robot description
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat humanoid.urdf)
```

## Model Features

### Humanoid Design Philosophy
- **Balance**: Center of mass positioned appropriately for bipedal locomotion
- **Proportions**: Realistic humanoid proportions for simulation
- **Joints**: Appropriate degrees of freedom for basic humanoid movements
- **Stability**: Flat feet for stable standing

### Joint Configuration
- **Neck Joint**: Single revolute joint for head movement
- **Arm Joints**: 3-DOF per arm (shoulder, elbow, wrist)
- **Leg Joints**: 3-DOF per leg (hip, knee, ankle)
- **Joint Limits**: Realistic ranges of motion for human-like movement

### Physical Properties
- **Total Mass**: ~27.6 kg (realistic for a humanoid robot)
- **Inertial Properties**: Properly calculated for simulation stability
- **Dimensions**: Proportional to a small humanoid robot (~1.8m tall)

## Extending the Model

This minimal model can be extended with:

- Additional joints for more dexterity
- More complex mesh geometry
- Sensors (IMU, cameras, force/torque sensors)
- Actuators and transmission elements
- Gazebo plugins for simulation

## Troubleshooting

### Common Issues

1. **Model not appearing in RViz**:
   - Ensure robot_state_publisher is running
   - Check that the URDF is properly loaded as a parameter

2. **Physics instability in simulation**:
   - Verify mass values are reasonable
   - Check inertia values are positive definite
   - Adjust simulation parameters if needed

3. **Joint limits too restrictive**:
   - Modify joint limit values in the URDF
   - Ensure values are in radians for revolute joints

## Files in This Directory

- `humanoid.urdf`: Complete validated humanoid robot model
- `README.md`: This file with validation and usage information

## References

This model follows ROS URDF best practices and can be used as a foundation for more complex humanoid robot models.