#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
import time


class SimpleTestPublisher(Node):
    """
    Simple test node that publishes simulated rover data to /values topic.
    This simulates a rover starting off the reference path to test the correction algorithm.
    """

    def __init__(self):
        super().__init__('simple_test_publisher')

        self.publisher = self.create_publisher(Float64MultiArray, '/values', 10)
        self.timer = self.create_timer(0.1, self.publish_test_data)  # 10Hz

        # Test scenario parameters
        self.start_time = time.time()

        # Start rover off the path to trigger correction
        self.x = 1.0      # Start 1m along the path
        self.y = 0.5      # Start 0.5m off the path (should trigger correction)
        self.heading = 0.2  # Start with slight heading error (~11 degrees)

        # Simulation parameters
        self.velocity = 0.0  # Will be controlled by path follower
        self.last_cmd_time = time.time()
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

        # Subscribe to cmd_vel to simulate robot response
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("Simple test publisher started")
        self.get_logger().info(f"Initial position: x={self.x:.2f}, y={self.y:.2f}, heading={math.degrees(self.heading):.1f}°")
        self.get_logger().info("This should trigger path correction since CTE > threshold")

    def cmd_vel_callback(self, msg):
        """Simulate robot response to velocity commands."""
        self.current_linear_vel = msg.linear.x
        self.current_angular_vel = msg.angular.z
        self.last_cmd_time = time.time()

    def publish_test_data(self):
        """Publish simulated rover position and heading."""
        dt = 0.1  # 100ms
        current_time = time.time()

        # Simple kinematic simulation
        # If no command received recently, assume zero velocity
        if current_time - self.last_cmd_time > 0.5:  # 500ms timeout
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0

        # Update robot state based on current velocities
        self.heading += self.current_angular_vel * dt
        self.x += self.current_linear_vel * dt * math.cos(self.heading)
        self.y += self.current_linear_vel * dt * math.sin(self.heading)

        # Normalize heading
        while self.heading > math.pi:
            self.heading -= 2 * math.pi
        while self.heading < -math.pi:
            self.heading += 2 * math.pi

        # Create and publish message
        msg = Float64MultiArray()
        msg.data = [self.x, self.y, self.heading]
        self.publisher.publish(msg)

        # Log position periodically
        elapsed_time = current_time - self.start_time
        if int(elapsed_time * 10) % 50 == 0:  # Every 5 seconds
            cte = abs(self.y)  # Simple CTE calculation for straight path along x-axis
            self.get_logger().info(
                f"Position: ({self.x:.2f}, {self.y:.2f}), "
                f"Heading: {math.degrees(self.heading):.1f}°, "
                f"CTE: {cte:.3f}m, "
                f"Velocities: {self.current_linear_vel:.2f} m/s, {math.degrees(self.current_angular_vel):.1f}°/s"
            )


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        node = SimpleTestPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down test publisher...")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
