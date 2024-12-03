#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
import time
import tf_transformations

class WorldToBaseBroadcaster(Node):
    def __init__(self):
        super().__init__('broadcaster')

        # Create a TransformBroadcaster object
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize parameters for movement
        self.frequency = 0.5  # Hz for oscillation
        self.amplitude = 1.0  # Meters for oscillation
        self.angular_speed = math.radians(30)  # Radians per second for rotation

        self.start_time = self.get_clock().now()

        # Timer to broadcast the transform periodically
        timer_period = 0.02  # seconds (50 Hz)
        self.timer = self.create_timer(timer_period, self.broadcast_transform)

        self.get_logger().info('World to Base Dynamic TF Broadcaster has been started.')

    def broadcast_transform(self):
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # seconds

        # Dynamic Translation: Oscillate along X-axis
        x = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time)
        y = 0.0
        z = 0.0

        # Create a TransformStamped message
        t = TransformStamped()

        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base'

        # Define the translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        # Define the rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast the transform
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    tf_broadcaster = WorldToBaseBroadcaster()

    try:
        rclpy.spin(tf_broadcaster)
    except KeyboardInterrupt:
        tf_broadcaster.get_logger().info('Shutting down TF Broadcaster.')

    tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()