#!/usr/bin/env python3

# ==========================================================
#                  VTOL DYNAMICS NODE
# ==========================================================
#
# X CONFIGURATION QUADROTOR DYNAMICS
#
# FEATURES:
#
# ✅ Vertical Dynamics
# ✅ Roll Dynamics
# ✅ Pitch Dynamics
# ✅ Yaw Dynamics
# ✅ X-Configuration Torque Model
# ✅ TF Publishing
# ✅ Odometry Publishing
# ✅ Basic Ground Collision
#
# ==========================================================

import math

from click import command

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

from tf_transformations import quaternion_from_euler
import subprocess


# ==========================================================
#                    VTOL DYNAMICS
# ==========================================================

class VTOLDynamics(Node):

    def __init__(self):

        super().__init__('vtol_dynamics')

        # ==================================================
        #                  PARAMETERS
        # ==================================================

        self.mass = 1.5

        self.gravity = 9.81

        self.k_thrust = 0.00005

        self.k_drag = 0.000001

        self.arm_length = 0.25

        self.dt = 0.01

        # ==================================================
        #               MOMENTS OF INERTIA
        # ==================================================

        self.Ix = 0.02

        self.Iy = 0.02

        self.Iz = 0.04

        # ==================================================
        #               POSITION STATES
        # ==================================================

        self.x = 0.0

        self.y = 0.0

        self.z = 0.0

        # ==================================================
        #               LINEAR VELOCITIES
        # ==================================================

        self.x_dot = 0.0

        self.y_dot = 0.0

        self.z_dot = 0.0

        # ==================================================
        #               LINEAR ACCELERATION
        # ==================================================

        self.x_ddot = 0.0

        self.y_ddot = 0.0

        self.z_ddot = 0.0

        # ==================================================
        #               ATTITUDE STATES
        # ==================================================

        self.roll = 0.0

        self.pitch = 0.0

        self.yaw = 0.0

        # ==================================================
        #                ANGULAR RATES
        # ==================================================

        self.p = 0.0

        self.q = 0.0

        self.r = 0.0

        # ==================================================
        #              MOTOR VELOCITIES
        # ==================================================

        self.motor_omega = [

            0.0,
            0.0,
            0.0,
            0.0

        ]

        # ==================================================
        #               ROS2 SUBSCRIBERS
        # ==================================================

        self.motor_subscriber = self.create_subscription(

            Float64MultiArray,
            '/motor_velocity_controller/commands',
            self.motor_callback,
            10

        )

        # ==================================================
        #               ROS2 PUBLISHERS
        # ==================================================

        self.odom_publisher = self.create_publisher(

            Odometry,
            '/odom',
            10

        )

        # ==================================================
        #                TF BROADCASTER
        # ==================================================

        self.tf_broadcaster = TransformBroadcaster(self)

        # ==================================================
        #                    TIMER
        # ==================================================

        self.timer = self.create_timer(

            self.dt,
            self.update_dynamics

        )

        self.get_logger().info(

            'VTOL Dynamics Node Started'

        )

    # ======================================================
    #                MOTOR CALLBACK
    # ======================================================

    def motor_callback(self, msg):

        if len(msg.data) >= 4:

            self.motor_omega = [

                msg.data[0],
                msg.data[1],
                msg.data[2],
                msg.data[3]

            ]
    
    
    # ======================================================
    #              UPDATE GAZEBO POSE
    # ======================================================

    def update_gazebo_pose(self):

        command = f"""
        ign service -s /world/default/set_pose \
        --reqtype ignition.msgs.Pose \
        --reptype ignition.msgs.Boolean \
        --timeout 1000 \
        --req '
        name: "vtol"

        position {{
         x: {self.x}
        y: {self.y}
        z: {self.z}
        }}

        orientation {{
        x: 0
        y: 0
        z: 0
        w: 1
        }}'
        """

        subprocess.run(

            command,

            shell=True,

            stdout=subprocess.DEVNULL,

            stderr=subprocess.DEVNULL

    )
    # ======================================================
    #                UPDATE DYNAMICS
    # ======================================================

    def update_dynamics(self):

        # ==================================================
        #               INDIVIDUAL THRUSTS
        # ==================================================

        F1 = self.k_thrust * self.motor_omega[0]**2

        F2 = self.k_thrust * self.motor_omega[1]**2

        F3 = self.k_thrust * self.motor_omega[2]**2

        F4 = self.k_thrust * self.motor_omega[3]**2

        # ==================================================
        #                TOTAL THRUST
        # ==================================================

        total_thrust = (

            F1 +
            F2 +
            F3 +
            F4

        )

        # ==================================================
        #          X CONFIGURATION TORQUE MODEL
        # ==================================================

        effective_arm = (

            self.arm_length /
            math.sqrt(2.0)

        )

        # ==================================================
        #                 ROLL TORQUE
        # ==================================================

        tau_roll = effective_arm * (

            F1 +
            F2 -
            F3 -
            F4

        )

        # ==================================================
        #                PITCH TORQUE
        # ==================================================

        tau_pitch = effective_arm * (

            F2 +
            F3 -
            F1 -
            F4

        )

        # ==================================================
        #                 YAW TORQUE
        # ==================================================

        tau_yaw = self.k_drag * (

            self.motor_omega[0]**2
            - self.motor_omega[1]**2
            + self.motor_omega[2]**2
            - self.motor_omega[3]**2

        )

        # ==================================================
        #             ANGULAR ACCELERATION
        # ==================================================

        p_dot = tau_roll / self.Ix

        q_dot = tau_pitch / self.Iy

        r_dot = tau_yaw / self.Iz

        # ==================================================
        #             UPDATE ANGULAR RATES
        # ==================================================

        self.p += p_dot * self.dt

        self.q += q_dot * self.dt

        self.r += r_dot * self.dt

        # ==================================================
        #             UPDATE ORIENTATION
        # ==================================================

        self.roll += self.p * self.dt

        self.pitch += self.q * self.dt

        self.yaw += self.r * self.dt

        # ==================================================
        #               WORLD FRAME THRUST
        # ==================================================

        thrust_x = (

            total_thrust *
            math.sin(self.pitch)

        )

        thrust_y = (

            -total_thrust *
            math.sin(self.roll)

        )

        thrust_z = (

            total_thrust *
            math.cos(self.roll) *
            math.cos(self.pitch)

        )

        # ==================================================
        #                 NET FORCES
        # ==================================================

        Fx = thrust_x

        Fy = thrust_y

        Fz = thrust_z - (

            self.mass *
            self.gravity

        )

        # ==================================================
        #             LINEAR ACCELERATION
        # ==================================================

        self.x_ddot = Fx / self.mass

        self.y_ddot = Fy / self.mass

        self.z_ddot = Fz / self.mass

        # ==================================================
        #              UPDATE VELOCITIES
        # ==================================================

        self.x_dot += self.x_ddot * self.dt

        self.y_dot += self.y_ddot * self.dt

        self.z_dot += self.z_ddot * self.dt

        # ==================================================
        #               UPDATE POSITION
        # ==================================================

        self.x += self.x_dot * self.dt

        self.y += self.y_dot * self.dt

        self.z += self.z_dot * self.dt

        # ==================================================
        #              GROUND COLLISION
        # ==================================================

        if self.z < 0.0:

            self.z = 0.0

            self.z_dot = 0.0

        # ==================================================
        #             PUBLISH ODOMETRY
        # ==================================================

        self.publish_odometry()

        # ==================================================
        #               DEBUG OUTPUT
        # ==================================================

        self.get_logger().info(

            f'POS -> '
            f'X:{self.x:.2f} '
            f'Y:{self.y:.2f} '
            f'Z:{self.z:.2f} | '

            f'ATT -> '
            f'R:{math.degrees(self.roll):.2f} '
            f'P:{math.degrees(self.pitch):.2f} '
            f'Y:{math.degrees(self.yaw):.2f}'

        )

    # ======================================================
    #               PUBLISH ODOMETRY
    # ======================================================

    def publish_odometry(self):

        odom = Odometry()

        odom.header.stamp = (

            self.get_clock().now().to_msg()

        )

        odom.header.frame_id = 'world'

        odom.child_frame_id = 'base_link'

        # ==================================================
        #                   POSITION
        # ==================================================

        odom.pose.pose.position.x = self.x

        odom.pose.pose.position.y = self.y

        odom.pose.pose.position.z = self.z

        # ==================================================
        #                 ORIENTATION
        # ==================================================

        q = quaternion_from_euler(

            self.roll,
            self.pitch,
            self.yaw

        )

        odom.pose.pose.orientation.x = q[0]

        odom.pose.pose.orientation.y = q[1]

        odom.pose.pose.orientation.z = q[2]

        odom.pose.pose.orientation.w = q[3]

        # ==================================================
        #                  VELOCITIES
        # ==================================================

        odom.twist.twist.linear.x = self.x_dot

        odom.twist.twist.linear.y = self.y_dot

        odom.twist.twist.linear.z = self.z_dot

        odom.twist.twist.angular.x = self.p

        odom.twist.twist.angular.y = self.q

        odom.twist.twist.angular.z = self.r

        # ==================================================
        #                 PUBLISH ODOM
        # ==================================================

        self.odom_publisher.publish(odom)

        # ==================================================
        #                TF TRANSFORM
        # ==================================================

        transform = TransformStamped()

        transform.header.stamp = (

            self.get_clock().now().to_msg()

        )

        transform.header.frame_id = 'world'

        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x

        transform.transform.translation.y = self.y

        transform.transform.translation.z = self.z

        transform.transform.rotation.x = q[0]

        transform.transform.rotation.y = q[1]

        transform.transform.rotation.z = q[2]

        transform.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(transform)


# ==========================================================
#                        MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = VTOLDynamics()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


# ==========================================================
#                        ENTRY
# ==========================================================

if __name__ == '__main__':

    main()