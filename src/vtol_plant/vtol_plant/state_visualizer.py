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
#                STATE VISUALIZER NODE
# ==========================================================

class StateVisualizer(Node):

    def __init__(self):

        super().__init__('state_visualizer')

        # ==================================================
        #                 SUBSCRIBER
        # ==================================================

        self.state_sub = self.create_subscription(

            Float32MultiArray,
            '/vtol_states',
            self.state_callback,
            10

        )

        # ==================================================
        #                  PUBLISHER
        # ==================================================

        self.marker_pub = self.create_publisher(

            Marker,
            '/actual_trajectory_marker',
            10

        )

        # ==================================================
        #              TRAJECTORY STORAGE
        # ==================================================

        self.points = []

        self.get_logger().info(

            'State Visualizer Started'

        )

    # ======================================================
    #                  STATE CALLBACK
    # ======================================================

    def state_callback(self, msg):

        # ==================================================
        #              EXTRACT STATES
        # ==================================================
        #
        # Expected State Vector:
        #
        # [
        #   u, v, w,
        #   p, q, r,
        #   x, y, z,
        #   roll, pitch, yaw
        # ]
        #
        # ==================================================

        x = msg.data[6]

        y = msg.data[7]

        z = msg.data[8]

        # ==================================================
        #                 CREATE POINT
        # ==================================================

        point = Point()

        point.x = float(x)

        point.y = float(y)

        point.z = float(z)

        # ==================================================
        #               STORE TRAJECTORY
        # ==================================================

        self.points.append(point)

        # ==================================================
        #             LIMIT MEMORY USAGE
        # ==================================================

        if len(self.points) > 2000:

            self.points.pop(0)

        # ==================================================
        #                CREATE MARKER
        # ==================================================

        marker = Marker()

        # ==================================================
        #                    HEADER
        # ==================================================

        marker.header.frame_id = 'world'

        marker.header.stamp = (

            self.get_clock().now().to_msg()

        )

        # ==================================================
        #                BASIC SETTINGS
        # ==================================================

        marker.ns = 'actual_trajectory'

        marker.id = 1

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
        #                ORIENTATION
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
        #
        # GREEN = ACTUAL TRAJECTORY
        #
        # ==================================================

        marker.color.a = 1.0

        marker.color.r = 0.0

        marker.color.g = 1.0

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

            f'Actual Position | '
            f'X={x:.2f} | '
            f'Y={y:.2f} | '
            f'Z={z:.2f} | '
            f'Points={len(self.points)}'

        )


# ==========================================================
#                         MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = StateVisualizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


# ==========================================================
#                        ENTRY
# ==========================================================

if __name__ == '__main__':

    main()