#!/usr/bin/env python3

# ==========================================================
#                       IMPORTS
# ==========================================================

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Quaternion

from tf_transformations import quaternion_from_euler

import numpy as np

# ==========================================================
#                  INTERNAL MODULES
# ==========================================================

from vtol_plant.parameter import VTOLParameters

from vtol_plant.dynamics import VTOLDynamics

from vtol_plant.integrator import RK4Integrator


# ==========================================================
#                    VTOL PLANT NODE
# ==========================================================

class VTOLPlantNode(Node):

    def __init__(self):

        super().__init__('vtol_plant_node')

        # ==================================================
        #                  PARAMETERS
        # ==================================================

        self.params = VTOLParameters()

        # ==================================================
        #                   DYNAMICS
        # ==================================================

        self.dynamics = VTOLDynamics(

            self.params

        )

        # ==================================================
        #                  INTEGRATOR
        # ==================================================

        self.integrator = RK4Integrator(

            self.dynamics,
            self.params.dt

        )

        # ==================================================
        #                  INITIAL STATE
        # ==================================================
        #
        # X =
        #
        # [u,v,w,p,q,r,x,y,z,phi,theta,psi]
        #
        # ==================================================

        self.X = np.zeros(12)

        # ==================================================
        #                CONTROL INPUTS
        # ==================================================
        #
        # U =
        #
        # [U1,U2,U3,U4]
        #
        # ==================================================

        self.U = np.zeros(4)

        # ==================================================
        #                 SUBSCRIBERS
        # ==================================================

        self.control_sub = self.create_subscription(

            Float32MultiArray,
            '/control_inputs',
            self.control_callback,
            10

        )

        # ==================================================
        #                  PUBLISHERS
        # ==================================================

        self.odom_pub = self.create_publisher(

            Odometry,
            '/vtol_odom',
            10

        )

        self.state_pub = self.create_publisher(

            Float32MultiArray,
            '/vtol_states',
            10

        )

        # ==================================================
        #                    TIMER
        # ==================================================

        self.timer = self.create_timer(

            self.params.dt,
            self.update_plant

        )

        self.get_logger().info(

            'VTOL Plant Node Started'

        )

    # ======================================================
    #                CONTROL CALLBACK
    # ======================================================

    def control_callback(self, msg):

        if len(msg.data) >= 4:

            self.U[0] = msg.data[0]

            self.U[1] = msg.data[1]

            self.U[2] = msg.data[2]

            self.U[3] = msg.data[3]

    # ======================================================
    #                  UPDATE PLANT
    # ======================================================

    def update_plant(self):

        # ==================================================
        #              RK4 STATE UPDATE
        # ==================================================

        self.X = self.integrator.step(

            self.X,
            self.U

        )

        # ==================================================
        #              PUBLISH STATES
        # ==================================================

        self.publish_states()

        # ==================================================
        #             PUBLISH ODOMETRY
        # ==================================================

        self.publish_odometry()

        # ==================================================
        #                DEBUG OUTPUT
        # ==================================================

        self.get_logger().info(

            f'X={self.X[6]:.2f} | '
            f'Y={self.X[7]:.2f} | '
            f'Z={self.X[8]:.2f} | '

            f'Roll={np.degrees(self.X[9]):.2f} | '
            f'Pitch={np.degrees(self.X[10]):.2f} | '
            f'Yaw={np.degrees(self.X[11]):.2f}'

        )

    # ======================================================
    #                 PUBLISH STATES
    # ======================================================

    def publish_states(self):

        msg = Float32MultiArray()

        msg.data = self.X.tolist()

        self.state_pub.publish(msg)

    # ======================================================
    #                PUBLISH ODOMETRY
    # ======================================================

    def publish_odometry(self):

        odom = Odometry()

        # ==================================================
        #                    HEADER
        # ==================================================

        odom.header.stamp = (

            self.get_clock().now().to_msg()

        )

        odom.header.frame_id = 'world'

        odom.child_frame_id = 'base_link'

        # ==================================================
        #                   POSITION
        # ==================================================

        odom.pose.pose.position.x = self.X[6]

        odom.pose.pose.position.y = self.X[7]

        odom.pose.pose.position.z = self.X[8]

        # ==================================================
        #                ORIENTATION
        # ==================================================

        q = quaternion_from_euler(

            self.X[9],
            self.X[10],
            self.X[11]

        )

        odom.pose.pose.orientation = Quaternion(

            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3]

        )

        # ==================================================
        #              LINEAR VELOCITIES
        # ==================================================

        odom.twist.twist.linear.x = self.X[0]

        odom.twist.twist.linear.y = self.X[1]

        odom.twist.twist.linear.z = self.X[2]

        # ==================================================
        #             ANGULAR VELOCITIES
        # ==================================================

        odom.twist.twist.angular.x = self.X[3]

        odom.twist.twist.angular.y = self.X[4]

        odom.twist.twist.angular.z = self.X[5]

        # ==================================================
        #                PUBLISH ODOM
        # ==================================================

        self.odom_pub.publish(odom)


# ==========================================================
#                         MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = VTOLPlantNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


# ==========================================================
#                        ENTRY
# ==========================================================

if __name__ == '__main__':

    main()