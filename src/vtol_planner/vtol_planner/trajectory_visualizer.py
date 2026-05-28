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
#              TRAJECTORY VISUALIZER NODE
# ==========================================================

class TrajectoryVisualizer(Node):

    def __init__(self):

        super().__init__('trajectory_visualizer')

        # ==================================================
        #                 SUBSCRIBER
        # ==================================================

        self.reference_sub = self.create_subscription(

            Float32MultiArray,
            '/reference_states',
            self.reference_callback,
            10

        )

        # ==================================================
        #                  PUBLISHER
        # ==================================================

        self.marker_pub = self.create_publisher(

            Marker,
            '/trajectory_marker',
            10

        )

        # ==================================================
        #               TRAJECTORY STORAGE
        # ==================================================

        self.points = []

        self.get_logger().info(

            'Trajectory Visualizer Started'

        )

    # ======================================================
    #              REFERENCE CALLBACK
    # ======================================================

    def reference_callback(self, msg):

        # ==================================================
        #               EXTRACT REFERENCES
        # ==================================================

        x_d = msg.data[0]

        y_d = msg.data[1]

        z_d = msg.data[2]

        # ==================================================
        #                 CREATE POINT
        # ==================================================

        point = Point()

        point.x = float(x_d)

        point.y = float(y_d)

        point.z = float(z_d)

        # ==================================================
        #              STORE TRAJECTORY
        # ==================================================

        self.points.append(point)

        # ==================================================
        #           LIMIT MEMORY USAGE
        # ==================================================

        if len(self.points) > 2000:

            self.points.pop(0)

        # ==================================================
        #               CREATE MARKER
        # ==================================================

        marker = Marker()

        # ==================================================
        #                  HEADER
        # ==================================================

        marker.header.frame_id = 'map'

        marker.header.stamp = (

            self.get_clock().now().to_msg()

        )

        # ==================================================
        #                BASIC SETTINGS
        # ==================================================

        marker.ns = 'trajectory'

        marker.id = 0

        marker.type = Marker.LINE_STRIP

        marker.action = Marker.ADD

        # ==================================================
        #                  LIFETIME
        # ==================================================

        marker.lifetime = Duration(

            sec=0,
            nanosec=0

        )

        # ==================================================
        #                 ORIENTATION
        # ==================================================

        marker.pose.orientation.x = 0.0

        marker.pose.orientation.y = 0.0

        marker.pose.orientation.z = 0.0

        marker.pose.orientation.w = 1.0

        # ==================================================
        #                    SCALE
        # ==================================================

        marker.scale.x = 0.2

        # ==================================================
        #                    COLOR
        # ==================================================

        marker.color.a = 1.0

        marker.color.r = 1.0

        marker.color.g = 0.0

        marker.color.b = 0.0

        # ==================================================
        #                    POINTS
        # ==================================================

        marker.points = self.points

        # ==================================================
        #                   PUBLISH
        # ==================================================

        self.marker_pub.publish(marker)

        # ==================================================
        #                    DEBUG
        # ==================================================

        self.get_logger().info(

            f'Points={len(self.points)} | '
            f'X={x_d:.2f} | '
            f'Y={y_d:.2f} | '
            f'Z={z_d:.2f}'

        )

# ==========================================================
#                         MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryVisualizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

# ==========================================================
#                        ENTRY
# ==========================================================

if __name__ == '__main__':

    main()