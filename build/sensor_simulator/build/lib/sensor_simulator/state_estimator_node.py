#!/usr/bin/env python3

# ==========================================================
#                 STATE ESTIMATOR NODE
# ==========================================================

import math

import rclpy

from rclpy.node import Node

from std_msgs.msg import (
    Float32MultiArray,
    Float64
)

from sensor_msgs.msg import Imu

from geometry_msgs.msg import PoseStamped

from tf_transformations import (
    euler_from_quaternion
)


# ==========================================================
#                STATE ESTIMATOR
# ==========================================================

class StateEstimator(Node):

    def __init__(self):

        super().__init__(

            'state_estimator_node'

        )

        # ==================================================
        # SENSOR STATES
        # ==================================================

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        # ==================================================
        # PREVIOUS VALUES
        # ==================================================

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0

        self.prev_time = (

            self.get_clock().now()

        )

        # ==================================================
        # SUBSCRIBERS
        # ==================================================

        self.create_subscription(

            Imu,

            '/imu/data',

            self.imu_callback,

            10

        )

        self.create_subscription(

            PoseStamped,

            '/gps/pose',

            self.gps_callback,

            10

        )

        self.create_subscription(

            Float64,

            '/barometer',

            self.barometer_callback,

            10

        )

        self.create_subscription(

            Float64,

            '/magnetometer',

            self.mag_callback,

            10

        )

        # ==================================================
        # PUBLISHER
        # ==================================================

        self.state_pub = self.create_publisher(

            Float32MultiArray,

            '/estimated_state',

            10

        )

        # ==================================================
        # TIMER
        # ==================================================

        self.create_timer(

            0.01,

            self.publish_estimate

        )

        self.get_logger().info(

            'State Estimator Started'

        )

    # ======================================================
    # IMU CALLBACK
    # ======================================================

    def imu_callback(

        self,

        msg

    ):

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        roll, pitch, yaw = (

            euler_from_quaternion(

                [qx, qy, qz, qw]

            )

        )

        self.phi = roll
        self.theta = pitch
        self.psi = yaw

        self.p = (

            msg.angular_velocity.x

        )

        self.q = (

            msg.angular_velocity.y

        )

        self.r = (

            msg.angular_velocity.z

        )

    # ======================================================
    # GPS CALLBACK
    # ======================================================

    def gps_callback(

        self,

        msg

    ):

        current_time = (

            self.get_clock().now()

        )

        dt = (

            current_time

            -

            self.prev_time

        ).nanoseconds / 1e9

        if dt > 1e-6:

            self.vx = (

                msg.pose.position.x

                -

                self.prev_x

            ) / dt

            self.vy = (

                msg.pose.position.y

                -

                self.prev_y

            ) / dt

        self.x = (

            msg.pose.position.x

        )

        self.y = (

            msg.pose.position.y

        )

        self.prev_x = self.x
        self.prev_y = self.y

        self.prev_time = current_time

    # ======================================================
    # BAROMETER CALLBACK
    # ======================================================

    def barometer_callback(

        self,

        msg

    ):

        current_time = (

            self.get_clock().now()

        )

        dt = (

            current_time

            -

            self.prev_time

        ).nanoseconds / 1e9

        if dt > 1e-6:

            self.vz = (

                msg.data

                -

                self.prev_z

            ) / dt

        self.z = msg.data

        self.prev_z = self.z

    # ======================================================
    # MAGNETOMETER CALLBACK
    # ======================================================

    def mag_callback(

        self,

        msg

    ):

        # Future yaw correction

        pass

    # ======================================================
    # PUBLISH ESTIMATE
    # ======================================================

    def publish_estimate(

        self

    ):

        msg = Float32MultiArray()

        msg.data = [

            # Position

            self.x,
            self.y,
            self.z,

            # Velocity

            self.vx,
            self.vy,
            self.vz,

            # Attitude

            self.phi,
            self.theta,
            self.psi,

            # Angular Rates

            self.p,
            self.q,
            self.r

        ]

        self.state_pub.publish(

            msg

        )

    # ======================================================
    # DESTROY
    # ======================================================

    def destroy_node(

        self

    ):

        super().destroy_node()


# ==========================================================
# MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = StateEstimator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()