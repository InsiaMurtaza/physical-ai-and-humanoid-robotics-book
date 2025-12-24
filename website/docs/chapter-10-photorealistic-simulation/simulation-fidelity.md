---
sidebar_position: 2
---

# Simulation Fidelity and Educational Impact

## Fidelity Considerations

Simulation fidelity refers to the degree of realism and accuracy in the virtual environment. Educational programs must balance several factors:

**Computational Requirements**: Higher fidelity simulations demand more computational resources, potentially limiting accessibility for some institutions.

**Learning Objectives**: The required fidelity level depends on specific educational goals and the complexity of robotic behaviors being taught.

**Transfer Learning**: The gap between simulation and reality (sim-to-real transfer) affects how well skills and algorithms developed in simulation apply to physical robots.

**Student Engagement**: Higher fidelity environments may increase student engagement but could also introduce complexity that obscures learning objectives.

### Computational Resource Management

Managing computational resources effectively is crucial for educational institutions implementing simulation-based robotics curricula. Different fidelity levels require significantly different hardware capabilities:

```python
# Example of computational resource estimation for different fidelity levels
import numpy as np

class FidelityResourceEstimator:
    def __init__(self):
        # Resource requirements per fidelity level (in GB GPU memory, CPU cores, etc.)
        self.fidelity_requirements = {
            'low': {
                'gpu_memory': 2.0,  # GB
                'cpu_cores': 4,
                'ram': 8.0,  # GB
                'physics_complexity': 0.3,  # 0-1 scale
                'render_quality': 0.2
            },
            'medium': {
                'gpu_memory': 6.0,
                'cpu_cores': 8,
                'ram': 16.0,
                'physics_complexity': 0.6,
                'render_quality': 0.5
            },
            'high': {
                'gpu_memory': 12.0,
                'cpu_cores': 16,
                'ram': 32.0,
                'physics_complexity': 0.9,
                'render_quality': 0.9
            }
        }

    def estimate_resources(self, fidelity_level, num_robots=1, environment_complexity=0.5):
        """Estimate computational resources needed for simulation"""
        if fidelity_level not in self.fidelity_requirements:
            raise ValueError(f"Invalid fidelity level: {fidelity_level}")

        base_reqs = self.fidelity_requirements[fidelity_level]

        # Scale requirements based on number of robots and environment complexity
        scaled_requirements = {}
        for key, value in base_reqs.items():
            if key in ['gpu_memory', 'ram']:
                # These scale more with environment complexity and number of robots
                scaled_requirements[key] = value * (1 + (num_robots - 1) * 0.3) * (1 + environment_complexity)
            elif key in ['cpu_cores']:
                # CPU cores scale with number of robots but less with environment complexity
                scaled_requirements[key] = value * (1 + (num_robots - 1) * 0.2)
            else:
                # Other metrics scale differently
                scaled_requirements[key] = value * (1 + environment_complexity * 0.5)

        return scaled_requirements

    def recommend_fidelity(self, available_resources, num_robots=1, learning_objective='basic'):
        """Recommend appropriate fidelity level based on available resources"""
        recommendations = []

        for fidelity, reqs in self.fidelity_requirements.items():
            # Check if available resources meet requirements (with some margin)
            meets_requirements = all(
                available_resources.get(key, 0) * 0.8 >= reqs[key]  # 20% margin
                for key in ['gpu_memory', 'cpu_cores', 'ram']
            )

            if meets_requirements:
                recommendations.append({
                    'fidelity': fidelity,
                    'requirements': reqs,
                    'suitable_for': self.get_learning_objective_suitability(fidelity, learning_objective)
                })

        return recommendations

    def get_learning_objective_suitability(self, fidelity, learning_objective):
        """Determine if a fidelity level is suitable for a specific learning objective"""
        objective_fidelity_map = {
            'basic_concepts': ['low', 'medium'],
            'algorithm_development': ['low', 'medium'],
            'perception_training': ['medium', 'high'],
            'realistic_behavior': ['medium', 'high'],
            'industry_preparation': ['high']
        }

        suitable = learning_objective in objective_fidelity_map and \
                   fidelity in objective_fidelity_map[learning_objective]

        return suitable

# Example usage in educational planning
def educational_simulation_planner():
    estimator = FidelityResourceEstimator()

    # Example: Institution with RTX 3080 (10GB VRAM), 16-core CPU, 32GB RAM
    available_resources = {
        'gpu_memory': 10.0,  # GB
        'cpu_cores': 16,
        'ram': 32.0  # GB
    }

    # For a basic robotics course with 1 robot
    recommendations = estimator.recommend_fidelity(
        available_resources,
        num_robots=1,
        learning_objective='algorithm_development'
    )

    print("Recommended fidelity levels for basic robotics course:")
    for rec in recommendations:
        print(f"- {rec['fidelity']}: {rec['suitable_for']}")

if __name__ == "__main__":
    educational_simulation_planner()
```

### Learning Objectives and Fidelity Alignment

Aligning simulation fidelity with specific learning objectives ensures that students receive the appropriate level of complexity for their educational goals:

```yaml
# Example configuration for aligning fidelity with learning objectives
learning_objective_fidelity_mapping:
  basic_robotics_concepts:
    fidelity_level: "low"
    target_skills:
      - "Understanding robot kinematics"
      - "Basic motion control"
      - "Simple path planning"
    recommended_environment:
      type: "grid_world"
      objects: ["simple_shapes"]
      physics: "simplified"

  intermediate_perception:
    fidelity_level: "medium"
    target_skills:
      - "Computer vision basics"
      - "Sensor fusion"
      - "Object detection"
    recommended_environment:
      type: "structured_indoor"
      objects: ["furniture", "simple_obstacles"]
      physics: "realistic"

  advanced_navigation:
    fidelity_level: "high"
    target_skills:
      - "Complex navigation"
      - "Realistic sensor simulation"
      - "Dynamic obstacle avoidance"
    recommended_environment:
      type: "photorealistic_outdoor"
      objects: ["detailed_models", "dynamic_obstacles"]
      physics: "high_fidelity"
```

### Transfer Learning and Sim-to-Real Considerations

The gap between simulation and reality (sim-to-real transfer) is a critical consideration for educational programs. Here's how to approach this challenge:

```python
# Example of sim-to-real transfer evaluation framework
class SimToRealTransferEvaluator:
    def __init__(self):
        self.metrics = {
            'performance_gap': 0.0,  # Difference between sim and real performance
            'adaptation_time': 0.0,  # Time to adapt to real robot
            'failure_modes': [],     # Types of failures when transferring
            'skill_retention': 0.0   # How well skills transfer
        }

    def evaluate_transfer(self, sim_performance, real_performance):
        """Evaluate the effectiveness of sim-to-real transfer"""
        # Calculate performance gap
        self.metrics['performance_gap'] = abs(sim_performance - real_performance)

        # Calculate transfer efficiency
        transfer_efficiency = 1.0 - self.metrics['performance_gap']

        # Identify common failure modes
        if sim_performance > 0.9 and real_performance < 0.5:
            self.metrics['failure_modes'].append('overfitting_to_simulation')

        return {
            'transfer_efficiency': transfer_efficiency,
            'sim_performance': sim_performance,
            'real_performance': real_performance,
            'metrics': self.metrics.copy()
        }

    def apply_domain_randomization(self, simulation_params):
        """Apply domain randomization to improve sim-to-real transfer"""
        randomized_params = {}

        for param, value in simulation_params.items():
            if isinstance(value, (int, float)):
                # Add random variation (±20%)
                variation = np.random.uniform(0.8, 1.2)
                randomized_params[param] = value * variation
            elif isinstance(value, (list, tuple)):
                # Randomize elements within the collection
                if len(value) > 0 and isinstance(value[0], (int, float)):
                    randomized_params[param] = [
                        v * np.random.uniform(0.8, 1.2) for v in value
                    ]
                else:
                    # For non-numeric collections, randomly select elements
                    randomized_params[param] = np.random.choice(value)
            else:
                randomized_params[param] = value

        return randomized_params

    def get_transfer_improvement_strategies(self):
        """Get strategies to improve sim-to-real transfer"""
        strategies = [
            {
                'name': 'domain_randomization',
                'description': 'Randomize simulation parameters to improve generalization',
                'implementation': 'Apply randomization to physics parameters, textures, lighting',
                'expected_improvement': 0.15  # 15% improvement in transfer efficiency
            },
            {
                'name': 'system_identification',
                'description': 'Characterize differences between sim and reality',
                'implementation': 'Compare robot dynamics and sensor responses in both environments',
                'expected_improvement': 0.20
            },
            {
                'name': 'progressive_transfer',
                'description': 'Gradually increase realism during training',
                'implementation': 'Start with low fidelity and gradually increase complexity',
                'expected_improvement': 0.18
            },
            {
                'name': 'simulated_sensor_noise',
                'description': 'Add realistic noise models to simulation',
                'implementation': 'Implement sensor-specific noise models based on real hardware',
                'expected_improvement': 0.12
            }
        ]

        return strategies

# Example usage in educational curriculum design
def design_transfer_focused_curriculum():
    evaluator = SimToRealTransferEvaluator()

    # Example simulation parameters
    sim_params = {
        'robot_mass': 10.0,  # kg
        'friction_coefficient': 0.1,
        'sensor_noise_std': 0.01,
        'motor_torque_limits': [50.0, 50.0],  # Nm
        'wheel_radius': 0.1,  # m
        'environment_textures': ['concrete', 'carpet', 'tile']
    }

    # Apply domain randomization
    randomized_params = evaluator.apply_domain_randomization(sim_params)

    # Get improvement strategies
    strategies = evaluator.get_transfer_improvement_strategies()

    print("Sim-to-real transfer improvement strategies:")
    for strategy in strategies:
        print(f"- {strategy['name']}: {strategy['expected_improvement']:.0%} improvement")

    return randomized_params, strategies

if __name__ == "__main__":
    params, strategies = design_transfer_focused_curriculum()
```

## Fidelity Options for Educational Settings

**Low Fidelity**: Simple geometric models with basic physics, suitable for algorithmic development and basic robotics concepts.

**Medium Fidelity**: Detailed geometric models with realistic physics, appropriate for intermediate robotics courses and perception algorithm development.

**High Fidelity**: Photorealistic rendering with complex material properties and environmental effects, ideal for advanced research and industry-relevant training.

### Low Fidelity Implementation

Low fidelity simulations are ideal for foundational robotics education where the focus is on algorithmic understanding rather than realistic physics:

```python
# Example of low-fidelity simulation implementation
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import matplotlib.animation as animation

class LowFidelityRobot:
    def __init__(self, start_pos=(0, 0), goal_pos=(10, 10)):
        self.position = np.array(start_pos, dtype=float)
        self.goal = np.array(goal_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0])
        self.path = [self.position.copy()]

    def simple_move_towards_goal(self, dt=0.1):
        """Simple movement towards goal without complex physics"""
        direction = self.goal - self.position
        distance = np.linalg.norm(direction)

        if distance > 0.1:  # If not very close to goal
            # Normalize direction and move with constant speed
            direction = direction / distance
            self.velocity = direction * 1.0  # Constant speed of 1 m/s
            self.position += self.velocity * dt
            self.path.append(self.position.copy())

        return distance < 0.1  # Return True if reached goal

class LowFidelityEnvironment:
    def __init__(self, width=20, height=20):
        self.width = width
        self.height = height
        self.robots = []

    def add_robot(self, robot):
        self.robots.append(robot)

    def update(self, dt=0.1):
        """Update all robots in the environment"""
        all_reached = True
        for robot in self.robots:
            reached = robot.simple_move_towards_goal(dt)
            if not reached:
                all_reached = False
        return all_reached

class LowFidelitySimulator:
    def __init__(self):
        self.env = LowFidelityEnvironment()

    def run_simulation(self, steps=100, dt=0.1):
        """Run the low-fidelity simulation"""
        robot = LowFidelityRobot(start_pos=(2, 2), goal_pos=(15, 15))
        self.env.add_robot(robot)

        for step in range(steps):
            all_reached = self.env.update(dt)
            if all_reached:
                print(f"Goal reached at step {step}")
                break

        return robot.path

    def visualize_path(self, path):
        """Visualize the robot's path"""
        path = np.array(path)

        plt.figure(figsize=(10, 10))
        plt.plot(path[:, 0], path[:, 1], 'b-', linewidth=2, label='Robot Path')
        plt.plot(path[0, 0], path[0, 1], 'go', markersize=10, label='Start')
        plt.plot(path[-1, 0], path[-1, 1], 'ro', markersize=10, label='Goal')

        plt.xlim(0, 20)
        plt.ylim(0, 20)
        plt.grid(True)
        plt.legend()
        plt.title('Low Fidelity Robot Navigation')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.show()

# Example usage
simulator = LowFidelitySimulator()
path = simulator.run_simulation()
simulator.visualize_path(path)
```

### Medium Fidelity Implementation

Medium fidelity simulations incorporate more realistic physics while maintaining computational efficiency:

```python
# Example of medium-fidelity simulation with physics
import numpy as np
import matplotlib.pyplot as plt

class MediumFidelityRobot:
    def __init__(self, start_pos=(0, 0), goal_pos=(10, 10)):
        self.position = np.array(start_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0])
        self.goal = np.array(goal_pos, dtype=float)
        self.mass = 5.0  # kg
        self.max_force = 10.0  # N
        self.max_velocity = 2.0  # m/s
        self.friction_coeff = 0.1
        self.path = [self.position.copy()]
        self.velocities = [self.velocity.copy()]

    def apply_force(self, force):
        """Apply force to the robot considering mass"""
        # Limit force magnitude
        force_magnitude = np.linalg.norm(force)
        if force_magnitude > self.max_force:
            force = force * (self.max_force / force_magnitude)

        # Calculate acceleration (F = ma)
        self.acceleration = force / self.mass

        # Apply friction (opposing motion)
        friction_force = -self.velocity * self.friction_coeff * 9.81
        friction_accel = friction_force / self.mass
        self.acceleration += friction_accel

    def update_physics(self, dt=0.01):
        """Update robot physics using simple integration"""
        # Update velocity
        self.velocity += self.acceleration * dt

        # Limit velocity
        vel_magnitude = np.linalg.norm(self.velocity)
        if vel_magnitude > self.max_velocity:
            self.velocity = self.velocity * (self.max_velocity / vel_magnitude)

        # Update position
        self.position += self.velocity * dt

        self.path.append(self.position.copy())
        self.velocities.append(self.velocity.copy())

    def simple_navigation(self):
        """Simple navigation with physics constraints"""
        direction_to_goal = self.goal - self.position
        distance_to_goal = np.linalg.norm(direction_to_goal)

        if distance_to_goal > 0.1:  # If not very close to goal
            # Normalize direction
            direction = direction_to_goal / distance_to_goal

            # Calculate desired velocity
            desired_velocity = direction * self.max_velocity * (distance_to_goal / 5.0)
            # Slow down as we get closer to goal
            if distance_to_goal < 1.0:
                desired_velocity *= distance_to_goal

            # Calculate force needed to reach desired velocity
            desired_acceleration = (desired_velocity - self.velocity) / 0.1  # 0.1s time constant
            force = desired_acceleration * self.mass

            return force
        else:
            return np.array([0.0, 0.0])  # No force needed if at goal

class MediumFidelitySimulator:
    def __init__(self):
        self.dt = 0.01  # 100 Hz simulation
        self.time = 0.0

    def run_physics_simulation(self, robot, max_time=30.0):
        """Run physics-based simulation"""
        while self.time < max_time:
            # Get navigation force
            force = robot.simple_navigation()

            # Apply force and update physics
            robot.apply_force(force)
            robot.update_physics(self.dt)

            # Check if goal is reached
            distance_to_goal = np.linalg.norm(robot.goal - robot.position)
            if distance_to_goal < 0.1:
                print(f"Goal reached at time {self.time:.2f}s")
                break

            self.time += self.dt

        return robot.path, robot.velocities

    def analyze_performance(self, path, velocities):
        """Analyze simulation performance"""
        path = np.array(path)
        velocities = np.array(velocities)

        # Calculate path length
        path_length = 0.0
        for i in range(1, len(path)):
            path_length += np.linalg.norm(path[i] - path[i-1])

        # Calculate average velocity
        avg_velocity = np.mean(np.linalg.norm(velocities, axis=1))

        # Calculate max velocity reached
        max_velocity = np.max(np.linalg.norm(velocities, axis=1))

        performance_metrics = {
            'path_length': path_length,
            'average_velocity': avg_velocity,
            'max_velocity': max_velocity,
            'total_time': self.time,
            'efficiency': path_length / self.time if self.time > 0 else 0
        }

        return performance_metrics

# Example usage
robot = MediumFidelityRobot(start_pos=(1, 1), goal_pos=(18, 18))
simulator = MediumFidelitySimulator()

path, velocities = simulator.run_physics_simulation(robot)
metrics = simulator.analyze_performance(path, velocities)

print("Medium Fidelity Simulation Results:")
for key, value in metrics.items():
    print(f"  {key}: {value:.3f}")
```

### High Fidelity Implementation

High fidelity simulations incorporate photorealistic rendering and complex physics for advanced educational applications:

```python
# Example of high-fidelity simulation concepts (pseudocode for Isaac Sim integration)
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Tuple
import json

@dataclass
class PhotorealisticSensor:
    """Represents a photorealistic sensor in high-fidelity simulation"""
    name: str
    sensor_type: str  # 'camera', 'lidar', 'imu', etc.
    position: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]  # quaternion
    parameters: Dict[str, float]

    def generate_sensor_data(self, environment_state):
        """Generate realistic sensor data based on environment state"""
        # This would interface with Isaac Sim's rendering engine
        # For this example, we'll simulate the process
        if self.sensor_type == 'camera':
            return self._generate_camera_data(environment_state)
        elif self.sensor_type == 'lidar':
            return self._generate_lidar_data(environment_state)
        else:
            return None

    def _generate_camera_data(self, environment_state):
        """Generate photorealistic camera data"""
        # Simulate realistic camera effects
        data = {
            'timestamp': environment_state['timestamp'],
            'image': self._render_photorealistic_image(environment_state),
            'depth': self._calculate_depth_map(environment_state),
            'camera_params': self.parameters
        }
        return data

    def _generate_lidar_data(self, environment_state):
        """Generate realistic LiDAR point cloud"""
        # Simulate realistic LiDAR effects
        data = {
            'timestamp': environment_state['timestamp'],
            'pointcloud': self._generate_point_cloud(environment_state),
            'intensities': self._calculate_reflectivities(environment_state)
        }
        return data

    def _render_photorealistic_image(self, environment_state):
        """Render photorealistic image using advanced rendering techniques"""
        # This would use Isaac Sim's Omniverse rendering engine
        # Simulate photorealistic effects
        base_resolution = (self.parameters.get('width', 640),
                          self.parameters.get('height', 480))

        # Add realistic effects: lens distortion, noise, lighting changes
        image_effects = {
            'lens_distortion': self.parameters.get('distortion_coeff', [0, 0, 0, 0, 0]),
            'noise_level': self.parameters.get('noise_level', 0.01),
            'exposure_time': self.parameters.get('exposure_time', 0.033),  # 30fps
            'iso': self.parameters.get('iso', 100)
        }

        # Return simulated photorealistic image data
        return {
            'resolution': base_resolution,
            'effects': image_effects,
            'lighting_conditions': environment_state['lighting']
        }

    def _calculate_depth_map(self, environment_state):
        """Calculate depth information for camera"""
        return {
            'min_distance': self.parameters.get('min_depth', 0.1),
            'max_distance': self.parameters.get('max_depth', 100.0),
            'depth_accuracy': self.parameters.get('depth_accuracy', 0.01)
        }

    def _generate_point_cloud(self, environment_state):
        """Generate realistic point cloud from LiDAR"""
        # Simulate LiDAR physics: beam divergence, multiple returns, etc.
        num_points = int(self.parameters.get('points_per_second', 100000) *
                        self.parameters.get('update_rate', 10) / 10)

        return {
            'num_points': num_points,
            'fov_horizontal': self.parameters.get('fov_horizontal', 360),
            'fov_vertical': self.parameters.get('fov_vertical', 40),
            'range_min': self.parameters.get('range_min', 0.1),
            'range_max': self.parameters.get('range_max', 100.0)
        }

    def _calculate_reflectivities(self, environment_state):
        """Calculate reflectivity values based on material properties"""
        return {
            'material_dependent': True,
            'weather_effects': environment_state.get('weather', 'clear')
        }

class HighFidelityEnvironment:
    """Represents a high-fidelity simulation environment"""
    def __init__(self, environment_config):
        self.config = environment_config
        self.sensors = []
        self.objects = []
        self.physics_properties = {}
        self.lighting = {}
        self.weather = {}

    def add_sensor(self, sensor: PhotorealisticSensor):
        """Add a photorealistic sensor to the environment"""
        self.sensors.append(sensor)

    def setup_physics(self, gravity=-9.81, air_density=1.225):
        """Configure realistic physics properties"""
        self.physics_properties = {
            'gravity': gravity,
            'air_density': air_density,
            'ground_friction': 0.8,
            'air_resistance': 0.01
        }

    def setup_lighting(self, time_of_day='noon', weather='clear'):
        """Configure realistic lighting conditions"""
        self.lighting = {
            'time_of_day': time_of_day,
            'sun_angle': self._calculate_sun_angle(time_of_day),
            'ambient_light': self._get_ambient_light(time_of_day),
            'shadow_quality': 'high'
        }

        self.weather = {
            'type': weather,
            'temperature': 20.0,
            'humidity': 0.5,
            'wind_speed': 2.0
        }

    def _calculate_sun_angle(self, time_of_day):
        """Calculate sun angle based on time of day"""
        time_map = {
            'dawn': 15, 'morning': 45, 'noon': 90,
            'afternoon': 45, 'dusk': 15, 'night': -15
        }
        return time_map.get(time_of_day, 90)

    def _get_ambient_light(self, time_of_day):
        """Get ambient light level based on time of day"""
        light_map = {
            'dawn': 0.3, 'morning': 0.8, 'noon': 1.0,
            'afternoon': 0.8, 'dusk': 0.3, 'night': 0.1
        }
        return light_map.get(time_of_day, 1.0)

    def get_environment_state(self):
        """Get current state of the environment"""
        return {
            'timestamp': 0,  # Would be actual simulation time
            'sensors': [s.name for s in self.sensors],
            'lighting': self.lighting,
            'weather': self.weather,
            'physics': self.physics_properties
        }

    def generate_sensor_data(self):
        """Generate data from all sensors in the environment"""
        environment_state = self.get_environment_state()
        sensor_data = {}

        for sensor in self.sensors:
            sensor_data[sensor.name] = sensor.generate_sensor_data(environment_state)

        return sensor_data

# Example configuration for high-fidelity environment
def create_educational_high_fidelity_env():
    """Create a high-fidelity environment for educational purposes"""

    # Environment configuration
    env_config = {
        'name': 'University Campus Simulation',
        'size': (100, 100, 20),  # x, y, z in meters
        'terrain_complexity': 'high',
        'object_density': 'medium'
    }

    # Create environment
    env = HighFidelityEnvironment(env_config)

    # Configure physics
    env.setup_physics(gravity=-9.81, air_density=1.225)

    # Configure lighting (noon, clear weather)
    env.setup_lighting(time_of_day='noon', weather='clear')

    # Add photorealistic sensors
    camera_sensor = PhotorealisticSensor(
        name='rgb_camera',
        sensor_type='camera',
        position=(0.5, 0.0, 0.8),  # mounted on robot
        rotation=(0, 0, 0, 1),  # no rotation
        parameters={
            'width': 1920,
            'height': 1080,
            'fov': 60,
            'min_depth': 0.1,
            'max_depth': 50.0,
            'noise_level': 0.005
        }
    )

    lidar_sensor = PhotorealisticSensor(
        name='hdl64_lidar',
        sensor_type='lidar',
        position=(0.5, 0.0, 1.0),
        rotation=(0, 0, 0, 1),
        parameters={
            'points_per_second': 2200000,
            'fov_horizontal': 360,
            'fov_vertical': 26.8,
            'range_min': 1.0,
            'range_max': 120.0,
            'update_rate': 10,
            'noise_level': 0.02
        }
    )

    env.add_sensor(camera_sensor)
    env.add_sensor(lidar_sensor)

    # Generate sample sensor data
    sensor_data = env.generate_sensor_data()

    print("High Fidelity Environment Created:")
    print(f"  Environment: {env.config['name']}")
    print(f"  Sensors: {[s.name for s in env.sensors]}")
    print(f"  Lighting: {env.lighting['time_of_day']} ({env.weather['type']})")

    return env, sensor_data

# Example usage
env, sensor_data = create_educational_high_fidelity_env()

# Display some sensor data properties
for sensor_name, data in sensor_data.items():
    print(f"\nSensor: {sensor_name}")
    if 'camera_params' in data:
        print(f"  Resolution: {data['image']['resolution']}")
        print(f"  Noise Level: {data['image']['effects']['noise_level']}")
    elif 'pointcloud' in data:
        print(f"  Points per second: {data['pointcloud']['num_points']}")
        print(f"  Range: {data['pointcloud']['range_min']}-{data['pointcloud']['range_max']}m")
```

These implementations demonstrate how different fidelity levels serve different educational purposes, from basic algorithmic development to advanced research applications that require photorealistic simulation.