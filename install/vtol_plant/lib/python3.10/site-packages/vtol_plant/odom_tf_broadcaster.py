#!/usr/bin/env python3

# ==========================================================
#                       IMPORTS
# ==========================================================

import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


# ==========================================================
#                ODOM TF BROADCASTER
# ==========================================================

class OdomTFBroadcaster(Node):

    def __init__(self):

        super().__init__(

            'odom_tf_broadcaster'

        )

        # ==================================================
        #              TF BROADCASTER
        # ==================================================

        self.tf_broadcaster = (

            TransformBroadcaster(

                self

            )

        )

        # ==================================================
        #               ODOM SUBSCRIBER
        # ==================================================

        self.odom_sub = (

            self.create_subscription(

                Odometry,

                '/vtol_odom',

                self.odom_callback,

                10

            )

        )

        self.get_logger().info(

            'Odom TF Broadcaster Started'

        )

    # ======================================================
    #                 ODOM CALLBACK
    # ======================================================

    def odom_callback(

        self,
        msg

    ):

        # ==================================================
        #              TF TRANSFORM
        # ==================================================

        transform = TransformStamped()

        transform.header.stamp = (

            self.get_clock().now().to_msg()

        )

        # ==================================================
        #               FRAME NAMES
        # ==================================================

        transform.header.frame_id = 'world'

        transform.child_frame_id = 'base_link'

        # ==================================================
        #                POSITION
        # ==================================================

        transform.transform.translation.x = (

            msg.pose.pose.position.x

        )

        transform.transform.translation.y = (

            msg.pose.pose.position.y

        )

        transform.transform.translation.z = (

            msg.pose.pose.position.z

        )

        # ==================================================
        #              ORIENTATION
        # ==================================================

        transform.transform.rotation = (

            msg.pose.pose.orientation

        )

        # ==================================================
        #                BROADCAST
        # ==================================================

        self.tf_broadcaster.sendTransform(

            transform

        )
        self.get_logger().info(
        f"Received odom z={msg.pose.pose.position.z:.2f}"
    )

# ==========================================================
#                         MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = OdomTFBroadcaster()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

# ==========================================================
#                        ENTRY
# ==========================================================

if __name__ == '__main__':

    main()