#!/usr/bin/env python3

"""
Python Agent Controlling Robot Action Example

This example demonstrates a Python-based AI agent that controls a simulated robot
through ROS 2 communication patterns. The agent processes sensor data and makes
decisions to control robot movement.
"""

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class PythonRobotAgent(Node):
    """
    A Python-based AI agent that controls a robot through ROS 2.
    This agent demonstrates the bridge between Python AI/ML applications
    and ROS 2 robotic systems.
    """

    def __init__(self):
        super().__init__('python_robot_agent')

        # Publishers for robot control
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers for sensor data
        self.laser_subscriber = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # Publishers for agent status and metrics
        self.status_publisher = self.create_publisher(String, 'agent_status', 10)
        self.distance_publisher = self.create_publisher(Float64, 'agent_distance', 10)

        # Internal state
        self.last_laser_data = None
        self.robot_position = (0.0, 0.0)  # x, y
        self.robot_heading = 0.0  # radians
        self.target_position = (5.0, 5.0)  # x, y - target destination
        self.cumulative_distance = 0.0
        self.previous_position = (0.0, 0.0)

        # Agent parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.safe_distance = 0.5  # meters to obstacle
        self.arrival_threshold = 0.3  # meters to target for arrival

        # Timer for agent decision making
        self.agent_timer = self.create_timer(0.1, self.agent_control_loop)

        self.get_logger().info('Python Robot Agent initialized and ready to control robot')

    def laser_callback(self, msg):
        """Process laser scan data from the robot."""
        try:
            # Convert LaserScan message to numpy array
            ranges = np.array(msg.ranges)
            # Filter out invalid ranges (inf, nan)
            valid_ranges = ranges[np.isfinite(ranges)]

            if len(valid_ranges) > 0:
                self.last_laser_data = {
                    'ranges': valid_ranges,
                    'min_distance': np.min(valid_ranges),
                    'angle_increment': msg.angle_increment,
                    'range_min': msg.range_min,
                    'range_max': msg.range_max
                }

                # Log obstacle detection
                if self.last_laser_data['min_distance'] < self.safe_distance:
                    self.get_logger().warn(
                        f'Obstacle detected! Distance: {self.last_laser_data["min_distance"]:.2f}m'
                    )
            else:
                self.get_logger().warn('No valid laser ranges received')

        except Exception as e:
            self.get_logger().error(f'Error processing laser data: {e}')

    def odom_callback(self, msg):
        """Process odometry data to track robot position."""
        try:
            # Extract position from odometry message
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # Extract orientation (convert quaternion to euler)
            quat = msg.pose.pose.orientation
            self.robot_heading = self.quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

            self.previous_position = self.robot_position
            self.robot_position = (x, y)

            # Calculate distance traveled since last update
            dx = x - self.previous_position[0]
            dy = y - self.previous_position[1]
            distance_delta = math.sqrt(dx*dx + dy*dy)
            self.cumulative_distance += distance_delta

            # Publish distance traveled
            dist_msg = Float64()
            dist_msg.data = self.cumulative_distance
            self.distance_publisher.publish(dist_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing odometry: {e}')

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def agent_control_loop(self):
        """Main control loop where the AI agent makes decisions."""
        try:
            # Check if we have necessary data
            if self.last_laser_data is None:
                self.get_logger().debug('Waiting for sensor data...')
                return

            # Get control command from AI decision making
            control_cmd = self.make_ai_decision()

            # Publish control command
            cmd_msg = Twist()
            cmd_msg.linear.x = control_cmd['linear']
            cmd_msg.angular.z = control_cmd['angular']
            self.cmd_vel_publisher.publish(cmd_msg)

            # Publish agent status
            status_msg = String()
            status_msg.data = f"Linear: {control_cmd['linear']:.2f}, Angular: {control_cmd['angular']:.2f}"
            self.status_publisher.publish(status_msg)

            # Log control decision
            self.get_logger().info(
                f'AI Agent: v={control_cmd["linear"]:.2f}, w={control_cmd["angular"]:.2f}'
            )

        except Exception as e:
            self.get_logger().error(f'Error in agent control loop: {e}')

    def make_ai_decision(self):
        """
        AI decision-making function that determines robot control commands
        based on sensor data and goals.
        """
        # Calculate distance to target
        dx = self.target_position[0] - self.robot_position[0]
        dy = self.target_position[1] - self.robot_position[1]
        distance_to_target = math.sqrt(dx*dx + dy*dy)

        # Check if we've reached the target
        if distance_to_target < self.arrival_threshold:
            self.get_logger().info(f'Goal reached! Distance to target: {distance_to_target:.2f}m')
            return {'linear': 0.0, 'angular': 0.0}  # Stop

        # Calculate desired heading to target
        desired_heading = math.atan2(dy, dx)
        heading_error = desired_heading - self.robot_heading

        # Normalize heading error to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Simple proportional controller for angular velocity
        angular_cmd = max(-self.angular_speed, min(self.angular_speed, 2.0 * heading_error))

        # Check for obstacles before moving forward
        min_obstacle_distance = self.last_laser_data['min_distance']

        if min_obstacle_distance < self.safe_distance:
            # Emergency stop or turn to avoid obstacle
            if abs(heading_error) < 0.5:  # Facing obstacle
                return {'linear': 0.0, 'angular': self.angular_speed}  # Turn in place
            else:
                return {'linear': 0.0, 'angular': angular_cmd}  # Just turn toward goal
        else:
            # Move toward target with obstacle avoidance
            linear_cmd = self.linear_speed if abs(angular_cmd) < 0.2 else self.linear_speed * 0.5
            return {'linear': linear_cmd, 'angular': angular_cmd}

    def get_agent_status(self):
        """Get current status of the agent."""
        return {
            'position': self.robot_position,
            'heading': self.robot_heading,
            'target': self.target_position,
            'distance_to_target': math.sqrt(
                (self.target_position[0] - self.robot_position[0])**2 +
                (self.target_position[1] - self.robot_position[1])**2
            ),
            'cumulative_distance': self.cumulative_distance
        }


def main(args=None):
    """
    Main function to run the Python Robot Agent.

    This function initializes the ROS 2 context, creates the agent node,
    and starts the spinning loop to process callbacks and control the robot.
    """
    rclpy.init(args=args)

    agent = PythonRobotAgent()

    try:
        agent.get_logger().info('Python Robot Agent starting - controlling robot via ROS 2')
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Interrupt received, shutting down agent')
    except Exception as e:
        agent.get_logger().error(f'Unexpected error: {e}')
    finally:
        agent.get_logger().info('Python Robot Agent shutting down')
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()