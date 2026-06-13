#!/usr/bin/env python3

# ==========================================================
#                       IMPORTS
# ==========================================================

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point

from builtin_interfaces.msg import Duration


# ==========================================================
#          ESTIMATED STATE VISUALIZER NODE
# ==========================================================

class EstimatedStateVisualizer(Node):

    def __init__(self):

        super().__init__('estimated_state_visualizer')

        # ==================================================
        #                 SUBSCRIBER
        # ==================================================

        self.state_sub = self.create_subscription(

            Float32MultiArray,
            '/estimated_state',
            self.state_callback,
            10

        )

        # ==================================================
        #                  PUBLISHER
        # ==================================================

        self.marker_pub = self.create_publisher(

            Marker,
            '/estimated_trajectory_marker',
            10

        )

        # ==================================================
        #              TRAJECTORY STORAGE
        # ==================================================

        self.points = []

        self.get_logger().info(

            'Estimated State Visualizer Started'

        )

    # ======================================================
    #                  STATE CALLBACK
    # ======================================================

    def state_callback(self, msg):

        if len(msg.data) < 12:

            return

        x = msg.data[6]
        y = msg.data[7]
        z = msg.data[8]

        point = Point()

        point.x = float(x)
        point.y = float(y)
        point.z = float(z)

        self.points.append(point)

        if len(self.points) > 2000:

            self.points.pop(0)

        marker = Marker()

        marker.header.frame_id = 'world'

        marker.header.stamp = (

            self.get_clock().now().to_msg()

        )

        marker.ns = 'estimated_trajectory'

        marker.id = 2

        marker.type = Marker.LINE_STRIP

        marker.action = Marker.ADD

        marker.lifetime = Duration(

            sec=0,
            nanosec=0

        )

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2

        # ==============================================
        # BLUE
        # ==============================================

        marker.color.a = 1.0

        marker.color.r = 0.0

        marker.color.g = 0.0

        marker.color.b = 1.0

        marker.points = self.points

        self.marker_pub.publish(marker)

        self.get_logger().info(

            f'Estimated Position | '
            f'X={x:.2f} | '
            f'Y={y:.2f} | '
            f'Z={z:.2f} | '
            f'Points={len(self.points)}'

        )


def main(args=None):

    rclpy.init(args=args)

    node = EstimatedStateVisualizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()