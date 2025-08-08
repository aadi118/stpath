#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class ValuesPublisher(Node):
    """
    Test node to publish simulated GPS+IMU+Odometry data on /values topic.
    This simulates a rover that starts off the path and needs correction.
    """

    def __init__(self):
        super().__init__('values_publisher')

        self.publisher = self.create_publisher(Float64MultiArray, '/values', 10)
        self.timer = self.create_timer(0.1, self.publish_values)  # 10Hz

        # Simulation state
        self.start_time = time.time()
        self.x = 0.0
        self.y = 2.0  # Start 2m off the path
        self.heading = 0.1  # Start with slight heading error
        self.velocity = 0.5  # Simulated forward velocity

        self.get_logger().info("Values publisher started - simulating rover off-path")

    def publish_values(self):
        """Publish simulated rover state."""
        current_time = time.time() - self.start_time

        # Simple simulation - rover moving forward with some drift
        self.x += self.velocity * 0.1 * math.cos(self.heading)
        self.y += self.velocity * 0.1 * math.sin(self.heading)

        # Add some noise to make it realistic
        self.heading += 0.01 * math.sin(current_time * 0.5)

        # Create message
        msg = Float64MultiArray()
        msg.data = [self.x, self.y, self.heading]

        self.publisher.publish(msg)

        if int(current_time) % 5 == 0 and int(current_time * 10) % 50 == 0:
            self.get_logger().info(f"Publishing: x={self.x:.2f}, y={self.y:.2f}, heading={math.degrees(self.heading):.1f}°")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ValuesPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
