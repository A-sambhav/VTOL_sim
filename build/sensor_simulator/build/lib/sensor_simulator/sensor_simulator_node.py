#!/usr/bin/env python3

# ==========================================================
#                 SENSOR SIMULATOR NODE
# ==========================================================

import math

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

from sensor_msgs.msg import Imu

from geometry_msgs.msg import PoseStamped

from tf_transformations import quaternion_from_euler


# ==========================================================
#                 SENSOR SIMULATOR
# ==========================================================

class SensorSimulator(Node):

    def __init__(self):

        super().__init__(

            'sensor_simulator_node'

        )
        # ==================================================
        # PREVIOUS STATES FOR MPU6500 MODEL
        # ==================================================

        self.prev_u = 0.0
        self.prev_v = 0.0
        self.prev_w = 0.0

        self.prev_time = self.get_clock().now()

        # ==================================================
        # SUBSCRIBER
        # ==================================================

        self.create_subscription(

            Float32MultiArray,

            '/vtol_states',

            self.state_callback,

            10

        )

        # ==================================================
        # IMU PUBLISHER
        # ==================================================

        self.imu_pub = self.create_publisher(

            Imu,

            '/imu/data',

            10

        )

        # ==================================================
        # GPS PUBLISHER
        # ==================================================

        self.gps_pub = self.create_publisher(

            PoseStamped,

            '/gps/pose',

            10

        )

        # ==================================================
        # BAROMETER PUBLISHER
        # ==================================================

        self.baro_pub = self.create_publisher(

            Float64,

            '/barometer',

            10

        )

        # ==================================================
        # MAGNETOMETER PUBLISHER
        # ==================================================

        self.mag_pub = self.create_publisher(

            Float64,

            '/magnetometer',

            10

        )

        self.get_logger().info(

            'Sensor Simulator Started'

        )

    # ======================================================
    # STATE CALLBACK
    # ======================================================

    def state_callback(

        self,

        msg

    ):

        if len(msg.data) < 12:

            return

        # ==================================================
        # STATES FROM VTOL PLANT
        # ==================================================
        #
        # X =
        #
        # [u,v,w,p,q,r,x,y,z,phi,theta,psi]
        #
        # ==================================================

        u = msg.data[0]
        v = msg.data[1]
        w = msg.data[2]

        p = msg.data[3]
        q = msg.data[4]
        r = msg.data[5]

        x = msg.data[6]
        y = msg.data[7]
        z = msg.data[8]

        phi = msg.data[9]
        theta = msg.data[10]
        psi = msg.data[11]

        # ==================================================
        # IMU MESSAGE
        # ==================================================

        imu_msg = Imu()

        imu_msg.header.stamp = (

            self.get_clock().now().to_msg()

        )

        imu_msg.header.frame_id = 'imu_link'

 # ==================================================
# MPU6500 STYLE OUTPUT
# ==================================================

# MPU6500 does not provide orientation

        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

# ==================================================
# GYROSCOPE
# ==================================================

        imu_msg.angular_velocity.x = p
        imu_msg.angular_velocity.y = q
        imu_msg.angular_velocity.z = r

# ==================================================
# ACCELEROMETER MODEL
# ==================================================

        current_time = self.get_clock().now()

        dt = (
    current_time -
    self.prev_time
        ).nanoseconds / 1e9

        if dt > 1e-4:

            u_dot = (
        u - self.prev_u
    ) / dt

            v_dot = (
        v - self.prev_v
    ) / dt

            w_dot = (
        w - self.prev_w
    ) / dt

        else:

            u_dot = 0.0
            v_dot = 0.0
            w_dot = 0.0

        g = 9.81

# Specific force measured by MPU6500

        ax = (
    u_dot
    + q * w
    - r * v
    + g * math.sin(theta)
)

        ay = (
    v_dot
    + r * u
    - p * w
    - g * math.cos(theta) * math.sin(phi)
)

        az = (
    w_dot
    + p * v
    - q * u
    + g * math.cos(theta) * math.cos(phi)
)

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

# ==================================================
# STORE PREVIOUS VALUES
# ==================================================

        self.prev_u = u
        self.prev_v = v
        self.prev_w = w

        self.prev_time = current_time

# ==================================================
# PUBLISH IMU
# ==================================================

        self.imu_pub.publish(
        imu_msg
)

        # ==================================================
        # GPS POSE
        # ==================================================

        gps_msg = PoseStamped()

        gps_msg.header.stamp = (

            self.get_clock().now().to_msg()

        )

        gps_msg.header.frame_id = 'world'

        gps_msg.pose.position.x = x
        gps_msg.pose.position.y = y
        gps_msg.pose.position.z = z

        gps_msg.pose.orientation.w = 1.0

        self.gps_pub.publish(

            gps_msg

        )

        # ==================================================
        # BAROMETER
        # ==================================================

        baro_msg = Float64()

        baro_msg.data = z

        self.baro_pub.publish(

            baro_msg

        )

        # ==================================================
        # MAGNETOMETER
        # ==================================================

        mag_msg = Float64()

        mag_msg.data = psi

        self.mag_pub.publish(

            mag_msg

        )


# ==========================================================
# MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = SensorSimulator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()