#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
from typing import Tuple


class RoverPathFollower(Node):
    """
    ROS2 node for rover path following with stationary turning and smooth merging.

    The rover follows a straight reference path and merges smoothly when drifting away.
    Only turns when stationary for precise heading control.
    """

    def __init__(self):
        super().__init__('rover_path_follower')

        # Declare parameters
        self.declare_parameter('L_min', 2.0)  # Minimum look-ahead distance (m)
        self.declare_parameter('k', 1.5)  # Cross-track error multiplier
        self.declare_parameter('CTE_threshold', 0.3)  # When to start correction (m)
        self.declare_parameter('CTE_tolerance', 0.1)  # When correction is complete (m)
        self.declare_parameter('heading_tol', 0.05)  # Heading tolerance (rad)
        self.declare_parameter('forward_velocity', 1.0)  # Forward velocity (m/s)
        self.declare_parameter('angular_velocity', 0.5)  # Angular velocity for turns (rad/s)
        self.declare_parameter('control_frequency', 10.0)  # Control loop frequency (Hz)

        # Path parameters
        self.declare_parameter('path_start_x', 0.0)
        self.declare_parameter('path_start_y', 0.0)
        self.declare_parameter('path_end_x', 20.0)
        self.declare_parameter('path_end_y', 0.0)

        # Get parameters
        self.L_min = self.get_parameter('L_min').value
        self.k = self.get_parameter('k').value
        self.CTE_threshold = self.get_parameter('CTE_threshold').value
        self.CTE_tolerance = self.get_parameter('CTE_tolerance').value
        self.heading_tol = self.get_parameter('heading_tol').value
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        control_freq = self.get_parameter('control_frequency').value

        # Reference path setup
        path_start_x = self.get_parameter('path_start_x').value
        path_start_y = self.get_parameter('path_start_y').value
        path_end_x = self.get_parameter('path_end_x').value
        path_end_y = self.get_parameter('path_end_y').value

        self.path_start = np.array([path_start_x, path_start_y])
        self.path_end = np.array([path_end_x, path_end_y])

        # Calculate path direction and reference heading
        path_vector = self.path_end - self.path_start
        path_length = np.linalg.norm(path_vector)
        if path_length > 0:
            self.path_direction = path_vector / path_length  # Normalized direction vector
            self.reference_heading = math.atan2(path_vector[1], path_vector[0])
        else:
            self.path_direction = np.array([1.0, 0.0])
            self.reference_heading = 0.0

        # Robot state
        self.position = np.array([0.0, 0.0])
        self.heading = 0.0
        self.data_received = False

        # Control flags
        self.is_correcting = False
        self.is_turning_to_target = False
        self.is_merging = False
        self.is_aligning_to_reference = False
        self.target_heading = 0.0

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.values_sub = self.create_subscription(
            Float64MultiArray, '/values', self.values_callback, 10)

        # Control timer
        self.control_timer = self.create_timer(1.0 / control_freq, self.control_loop)

        self.get_logger().info(f"Rover Path Follower initialized")
        self.get_logger().info(f"Path: ({path_start_x:.1f},{path_start_y:.1f}) -> ({path_end_x:.1f},{path_end_y:.1f})")
        self.get_logger().info(f"Control frequency: {control_freq:.1f}Hz")

    def values_callback(self, msg):
        """Callback for fused GPS+IMU+Odometry data from /values topic."""
        if len(msg.data) >= 3:
            self.position[0] = msg.data[0]  # x position (m)
            self.position[1] = msg.data[1]  # y position (m)
            self.heading = msg.data[2]      # heading (radians)
            self.data_received = True

    def calculate_cross_track_error(self) -> Tuple[float, np.ndarray]:
        """
        Calculate the rover's current cross-track error (CTE) from the reference line.
        Project the rover's position onto the line to find the closest point (t0).

        Returns:
            Tuple[float, np.ndarray]: (cross_track_error, closest_point_t0)
        """
        # Vector from path start to current position
        pos_vector = self.position - self.path_start

        # Project position onto the path direction to find t0
        projection_length = np.dot(pos_vector, self.path_direction)

        # Clamp projection to path bounds
        path_length = np.linalg.norm(self.path_end - self.path_start)
        projection_length = max(0, min(projection_length, path_length))

        # Calculate closest point on path (t0)
        closest_point_t0 = self.path_start + projection_length * self.path_direction

        # Calculate cross-track error (perpendicular distance to path)
        error_vector = self.position - closest_point_t0
        cross_track_error = np.linalg.norm(error_vector)

        # Determine sign of error (left/right of path)
        cross_product = np.cross(self.path_direction, error_vector)
        if cross_product < 0:
            cross_track_error = -cross_track_error

        return cross_track_error, closest_point_t0

    def calculate_look_ahead_distance(self, cte: float) -> float:
        """
        Choose a look-ahead distance L, proportional to the CTE.
        L = max(L_min, k * abs(CTE))
        """
        return max(self.L_min, self.k * abs(cte))

    def calculate_target_point(self, closest_point_t0: np.ndarray, look_ahead_L: float) -> np.ndarray:
        """
        Compute a target point on the line at t0 + L.
        """
        # Move look-ahead distance along path from closest point
        target_point = closest_point_t0 + look_ahead_L * self.path_direction

        # Clamp to path bounds
        path_length = np.linalg.norm(self.path_end - self.path_start)
        projection_from_start = np.dot(target_point - self.path_start, self.path_direction)
        if projection_from_start > path_length:
            target_point = self.path_end
        elif projection_from_start < 0:
            target_point = self.path_start

        return target_point

    def calculate_heading_to_target(self, target_point: np.ndarray) -> float:
        """
        Calculate the heading from the rover's position to the target point.
        """
        direction_to_target = target_point - self.position
        if np.linalg.norm(direction_to_target) < 1e-6:
            return self.heading
        return math.atan2(direction_to_target[1], direction_to_target[0])

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def is_heading_aligned(self, target_heading: float) -> bool:
        """Check if current heading is aligned with target within tolerance."""
        heading_error = abs(self.normalize_angle(target_heading - self.heading))
        return heading_error < self.heading_tol

    def stop_robot(self):
        """Stop the robot completely."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def turn_in_place(self, target_heading: float) -> bool:
        """
        Turn in place until the rover's heading matches the target heading (within tolerance).

        Returns:
            bool: True if heading is aligned, False if still turning
        """
        cmd = Twist()
        cmd.linear.x = 0.0  # No forward movement during turning

        heading_error = self.normalize_angle(target_heading - self.heading)

        if abs(heading_error) < self.heading_tol:
            # Heading aligned within tolerance, stop turning
            self.stop_robot()
            return True
        else:
            # Turn towards target heading
            cmd.angular.z = self.angular_vel if heading_error > 0 else -self.angular_vel
            self.cmd_vel_pub.publish(cmd)
            return False

    def move_forward(self):
        """Move forward so the rover merges with the line at the target point."""
        cmd = Twist()
        cmd.linear.x = self.forward_vel
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def control_loop(self):
        """
        Main control loop implementing the path following algorithm.
        Runs continuously at the specified control frequency.
        """

        # Skip if no sensor data received yet
        if not self.data_received:
            return

        # Step 1: Calculate the rover's current cross-track error (CTE) from the reference line
        # Step 2: Project the rover's position onto the line to find the closest point (t0)
        cte, closest_point_t0 = self.calculate_cross_track_error()

        # Determine current status for logging
        status = "FOLLOW_PATH"
        if self.is_turning_to_target:
            status = "TURN_TO_TARGET"
        elif self.is_merging:
            status = "MERGE"
        elif self.is_aligning_to_reference:
            status = "ALIGN_TO_REF"

        # Log current state every 1 second (10 cycles at 10Hz)
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0

        if self.log_counter % 10 == 0:
            self.get_logger().info(
                f"Status: {status}, CTE: {cte:.3f}m, Heading: {math.degrees(self.heading):.1f}°")

        # Main control logic - no state machine, just continuous execution
        if not self.is_correcting:
            # Normal path following mode
            if abs(cte) > self.CTE_threshold:
                # CTE exceeds threshold - start correction procedure
                self.get_logger().info(f"CTE {cte:.3f}m > threshold {self.CTE_threshold:.3f}m - Starting correction")

                self.is_correcting = True
                self.is_turning_to_target = True

                # Step 3: Choose a look-ahead distance L, proportional to the CTE
                look_ahead_L = self.calculate_look_ahead_distance(cte)

                # Step 4: Compute a target point on the line at t0 + L
                target_point = self.calculate_target_point(closest_point_t0, look_ahead_L)

                # Step 5: Calculate the heading from the rover's position to this target point
                self.target_heading = self.calculate_heading_to_target(target_point)

                self.get_logger().info(f"Look-ahead: {look_ahead_L:.2f}m, Target heading: {math.degrees(self.target_heading):.1f}°")

                # Stop and prepare to turn
                self.stop_robot()
            else:
                # CTE within acceptable range - continue normal path following
                if not self.is_heading_aligned(self.reference_heading):
                    # Minor heading correction needed
                    self.stop_robot()
                    self.is_aligning_to_reference = True
                    self.target_heading = self.reference_heading
                else:
                    # Step 8: Continue forward along the reference path
                    self.move_forward()
        else:
            # Correction mode - handle the correction sequence
            if self.is_turning_to_target:
                # Step 6: Turn in place until the rover's heading matches target heading
                if self.turn_in_place(self.target_heading):
                    self.get_logger().info("Heading aligned with target - Starting merge")
                    self.is_turning_to_target = False
                    self.is_merging = True

            elif self.is_merging:
                # Step 7: Move forward so the rover merges with the line at the target point
                if abs(cte) <= self.CTE_tolerance:
                    # Step 8: Once the CTE is within a small tolerance, prepare to align with reference
                    self.get_logger().info(f"Merge complete - CTE {abs(cte):.3f}m <= tolerance {self.CTE_tolerance:.3f}m")
                    self.stop_robot()
                    self.is_merging = False
                    self.is_aligning_to_reference = True
                    self.target_heading = self.reference_heading
                else:
                    # Continue merging toward the path
                    self.move_forward()

            elif self.is_aligning_to_reference:
                # Step 8: Turn in place to match the line's original heading and continue forward
                if self.turn_in_place(self.reference_heading):
                    self.get_logger().info("Reference heading aligned - Resuming normal path following")
                    # Reset all correction flags
                    self.is_correcting = False
                    self.is_aligning_to_reference = False


def main(args=None):
    """Main function to initialize and run the rover path follower."""
    rclpy.init(args=args)

    try:
        node = RoverPathFollower()
        self.get_logger().info("Rover Path Follower node started. Waiting for /values data...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down rover path follower...")
    finally:
        # Clean shutdown
        if 'node' in locals():
            node.stop_robot()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
