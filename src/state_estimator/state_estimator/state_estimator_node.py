#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

from tf_transformations import euler_from_quaternion

class StateEstimator(Node):


    def __init__(self):

        super().__init__('state_estimator_node')

    # ==========================================
    # Estimated States
    # ==========================================

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.u = 0.0
        self.v = 0.0
        self.w = 0.0

        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

    # ==========================================
    # Previous Values
    # ==========================================

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0

        self.prev_gps_time = self.get_clock().now()
        self.prev_baro_time = self.get_clock().now()

    # ==========================================
    # Subscribers
    # ==========================================

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

    # ==========================================
    # Publisher
    # ==========================================

        self.state_pub = self.create_publisher(
        Float32MultiArray,
        '/estimated_state',
        10
    )

    # ==========================================
    # Timer
    # ==========================================

        self.create_timer(
        0.01,
        self.publish_estimate
    )

        self.get_logger().info(
        'State Estimator Started'
    )

# ==============================================
# IMU CALLBACK
# ==============================================

    def imu_callback(self, msg):

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        roll, pitch, yaw = euler_from_quaternion(
        [qx, qy, qz, qw]
    )

        self.phi = roll
        self.theta = pitch
        self.psi = yaw

        self.p = msg.angular_velocity.x
        self.q = msg.angular_velocity.y
        self.r = msg.angular_velocity.z

# ==============================================
# GPS CALLBACK
# ==============================================

    def gps_callback(self, msg):

        current_time = self.get_clock().now()

        dt = (
        current_time -
        self.prev_gps_time
        ).nanoseconds / 1e9

        if dt > 1e-3:

            vx = (
            msg.pose.position.x -
            self.prev_x
        ) / dt

            vy = (
            msg.pose.position.y -
            self.prev_y
        ) / dt

            self.u = vx
            self.v = vy

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        self.prev_x = self.x
        self.prev_y = self.y

        self.prev_gps_time = current_time

# ==============================================
# BAROMETER CALLBACK
# ==============================================

    def barometer_callback(self, msg):

        current_time = self.get_clock().now()

        dt = (
        current_time -
        self.prev_baro_time
        ).nanoseconds / 1e9

        if dt > 1e-3:

            vz = (
            msg.data -
            self.prev_z
            ) / dt

            self.w = vz

        self.z = msg.data

        self.prev_z = self.z

        self.prev_baro_time = current_time

    # ==============================================
    # MAGNETOMETER CALLBACK
    # ==============================================

    def mag_callback(self, msg):

        self.psi = msg.data

# ==============================================
# PUBLISH ESTIMATE
# ==============================================

    def publish_estimate(self):

        msg = Float32MultiArray()

        msg.data = [

        # Body Velocities

        self.u,
        self.v,
        self.w,

        # Angular Rates

        self.p,
        self.q,
        self.r,

        # Position

        self.x,
        self.y,
        self.z,

        # Attitude

        self.phi,
        self.theta,
        self.psi

    ]

        self.state_pub.publish(msg)

# ==============================================
# DESTROY
# ==============================================

    def destroy_node(self):

        super().destroy_node()

def main(args=None):


    rclpy.init(args=args)

    node = StateEstimator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

    if __name__ == '__main__':
        main()

