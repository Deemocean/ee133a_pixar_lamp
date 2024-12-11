import numpy as np
import rclpy
import tf2_ros

from math import nan

from rclpy.node import Node
from visualization_msgs.msg import Marker  # Import Marker message
from rclpy.duration import Duration


class MarkerNode(Node):
    # Initialization.
    def __init__(self, name="marker_node", marker_pos=[0.0,0.0,0.0], rate=10.0):
        """
        Initialize the MarkerNode.

        :param name: Name of the node.
        :param namespace: Namespace for the node.
        :param rate: Publishing rate in Hz.
        :param trajectory: Optional trajectory data (not used for fixed marker).
        """
        super().__init__(name)

        # Initialize rate and timer interval
        self.rate = rate
        self.dt = 1.0 / rate  # Timer interval in seconds

        self.marker_pos = marker_pos

        # Create a publisher for Marker messages
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Initialize timer to call the update method periodically
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info(f"MarkerNode initialized with rate {self.rate} Hz (dt={self.dt} s)")


    def update(self):
        """
        Publish the marker.
        """
        # Create and populate the Marker message
        marker = Marker()
        marker.header.frame_id = "world"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lamp_visualization"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set marker position (fixed position)
        marker.pose.position.x = self.marker_pos[0]
        marker.pose.position.y = self.marker_pos[1]
        marker.pose.position.z = self.marker_pos[2]

        # Set marker orientation (no rotation)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set marker scale (size of the sphere)
        marker.scale.x = 0.2 # Diameter in meters
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Set marker color (RGBA)
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Set marker lifetime (0 means marker does not auto-delete)
        marker.lifetime = Duration(seconds=0).to_msg()

        # Publish the marker
        self.marker_pub.publish(marker)

        self.get_logger().debug(
            f"Published marker at ({marker.pose.position.x}, "
            f"{marker.pose.position.y}, {marker.pose.position.z})"
        )

    def shutdown(self):
        """
        Shutdown the node gracefully.
        """
        self.get_logger().info("Shutting down MarkerNode...")
        self.timer.cancel()
        self.destroy_node()

    def spin(self):
        """
        Spin the node until interrupted.
        """
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt received. Shutting down.")
        finally:
            self.shutdown()

def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Create an instance of MarkerNode
    marker_node = MarkerNode(
        name="marker_node",
        marker_pos=[-4.0, 2.0, 1.0],  
        rate=10.0,                         
    )
    try:
        # Spin the node. This will keep the node running and processing callbacks.
        rclpy.spin(marker_node)
    except KeyboardInterrupt:
        # Handle the shutdown gracefully on Ctrl+C
        marker_node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        # Shutdown the node and ROS.
        marker_node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()