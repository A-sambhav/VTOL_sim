#!/usr/bin/env python3

# ==========================================================
#                       IMPORTS
# ==========================================================

import math

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from vtol_controller.controller_parameters import (
    ControllerParameters
)

from vtol_controller.feedback_linearization import (
    FeedbackLinearization
)

from vtol_controller.lqr_controller import (
    LQRController
)

# ==========================================================
#                  CONTROLLER NODE
# ==========================================================

class ControllerNode(Node):

    def __init__(self):

        super().__init__('controller_node')

        # ==================================================
        #                  PARAMETERS
        # ==================================================

        self.params = ControllerParameters()

        # ==================================================
        #                CONTROLLERS
        # ==================================================

        self.feedback_linearization = (

            FeedbackLinearization()

        )

        self.lqr = LQRController()

        # ==================================================
        #              REFERENCE STATES
        # ==================================================

        self.reference = [0.0] * 10

        # ==================================================
        #                PLANT STATES
        # ==================================================

        self.states = [0.0] * 12

        # ==================================================
        #                 SUBSCRIBERS
        # ==================================================

        self.reference_sub = self.create_subscription(

            Float32MultiArray,

            '/reference_states',

            self.reference_callback,

            10

        )

        self.state_sub = self.create_subscription(

            Float32MultiArray,

            '/vtol_states',

            self.state_callback,

            10

        )

        # ==================================================
        #                 PUBLISHER
        # ==================================================

        self.control_pub = self.create_publisher(

            Float32MultiArray,

            '/control_inputs',

            10

        )

        # ==================================================
        #                    TIMER
        # ==================================================

        self.dt = 0.01

        self.timer = self.create_timer(

            self.dt,

            self.control_loop

        )

        self.get_logger().info(

            'Controller Node Started'

        )

    # ======================================================
    #             REFERENCE CALLBACK
    # ======================================================

    def reference_callback(self, msg):

        self.reference = msg.data

    # ======================================================
    #                STATE CALLBACK
    # ======================================================

    def state_callback(self, msg):

        self.states = msg.data

    # ======================================================
    #                 CONTROL LOOP
    # ======================================================

    def control_loop(self):
    
        if len(self.reference) < 10:
            return

        if len(self.states) < 12:
            return

    # ==================================================
    #           UNPACK REFERENCES
    # ==================================================

        xr = self.reference[0]
        yr = self.reference[1]
        zr = self.reference[2]

        vxr = self.reference[3]
        vyr = self.reference[4]
        vzr = self.reference[5]

        psi_r = self.reference[9]

    # ==================================================
    #      PLANNER CURRENTLY DOES NOT PROVIDE
    #      DESIRED ACCELERATIONS
    # ==================================================

        axr = 0.0
        ayr = 0.0
        azr = 0.0

    # ==================================================
    #             UNPACK STATES
    # ==================================================

        u = self.states[0]
        v = self.states[1]
        w = self.states[2]

        p = self.states[3]
        q = self.states[4]
        r = self.states[5]

        x = self.states[6]
        y = self.states[7]
        z = self.states[8]

        phi = self.states[9]
        theta = self.states[10]
        psi = self.states[11]
        # ==================================================
        #      BODY → INERTIAL VELOCITY TRANSFORMATION
        # ==================================================

        cphi = math.cos(phi)
        sphi = math.sin(phi)

        cth = math.cos(theta)
        sth = math.sin(theta)

        cpsi = math.cos(psi)
        spsi = math.sin(psi)

        vx = (

    cth * cpsi * u

    +

    (sphi * sth * cpsi - cphi * spsi) * v

    +

    (cphi * sth * cpsi + sphi * spsi) * w

)

        vy = (

    cth * spsi * u

    +

    (sphi * sth * spsi + cphi * cpsi) * v

    +

    (cphi * sth * spsi - sphi * cpsi) * w

)

        vz = (

    -sth * u

    +

    sphi * cth * v

    +

    cphi * cth * w

)

    # ==================================================
    #       FEEDBACK LINEARIZATION
    # ==================================================

        reference = [

        xr,
        yr,
        zr,

        vxr,
        vyr,
        vzr,

        psi_r

    ]

        state = [

    x,
    y,
    z,

    vx,
    vy,
    vz,

    phi,
    theta

]

        fl_output = self.feedback_linearization.compute(

    reference,
    state

)

        U1 = fl_output['U1']

        phi_r = fl_output['phi_ref']

        theta_r = fl_output['theta_ref']

        psi_r = fl_output['psi_ref']
    # ==================================================
    #              LQR CONTROLLER
    # ==================================================

        control = self.lqr.compute_control(

            phi,
            p,

            theta,
            q,

            psi,
            r,

            phi_r,
            theta_r,
            psi_r

    )

        U2 = control['U2']
        U3 = control['U3']
        U4 = control['U4']

    # ==================================================
    #             CONTROL VECTOR
    # ==================================================

        control_msg = Float32MultiArray()

        control_msg.data = [

        U1,
        U2,
        U3,
        U4

    ]

        self.control_pub.publish(

        control_msg

    )

    # ==================================================
    #                    DEBUG
    # ==================================================

        self.get_logger().info(

        f'U1={U1:.2f} | '
        f'U2={U2:.2f} | '
        f'U3={U3:.2f} | '
        f'U4={U4:.2f}'

    )

# ==========================================================
#                         MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = ControllerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

# ==========================================================
#                        ENTRY
# ==========================================================

if __name__ == '__main__':

    main()