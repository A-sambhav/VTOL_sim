#!/usr/bin/env python3

# ==========================================================
#               FEEDBACK LINEARIZATION
# ==========================================================

import math

from vtol_controller.controller_parameters import ControllerParameters


# ==========================================================
#          FEEDBACK LINEARIZATION CONTROLLER
# ==========================================================

class FeedbackLinearization:

    def __init__(self):

        self.params = ControllerParameters()

        self.m = self.params.mass

        self.g = self.params.gravity

        # ==================================================
        # X AXIS GAINS
        # ==================================================

        self.k1x = self.params.k1x
        self.k2x = self.params.k2x

        # ==================================================
        # Y AXIS GAINS
        # ==================================================

        self.k1y = self.params.k1y
        self.k2y = self.params.k2y

        # ==================================================
        # Z AXIS GAINS
        # ==================================================

        self.k1z = self.params.k1z
        self.k2z = self.params.k2z

    # ======================================================
    #                 MAIN CONTROLLER
    # ======================================================

    def compute(

        self,

        reference,
        state

    ):

        # ==================================================
        #           REFERENCE STATES
        # ==================================================

        xr = reference[0]
        yr = reference[1]
        zr = reference[2]

        vxr = reference[3]
        vyr = reference[4]
        vzr = reference[5]

        psi_r = reference[6]

        # ==================================================
        #             CURRENT STATES
        # ==================================================

        x = state[0]
        y = state[1]
        z = state[2]

        vx = state[3]
        vy = state[4]
        vz = state[5]

        phi = state[6]
        theta = state[7]

        # ==================================================
        #               POSITION ERRORS
        # ==================================================

        ex = xr - x
        ey = yr - y
        ez = zr - z

        # ==================================================
        #               VELOCITY ERRORS
        # ==================================================

        ex_dot = vxr - vx
        ey_dot = vyr - vy
        ez_dot = vzr - vz

        # ==================================================
        #            VIRTUAL CONTROL INPUTS
        # ==================================================

        ux = (

            self.k1x * ex

            +

            self.k2x * ex_dot

        )

        uy = (

            self.k1y * ey

            +

            self.k2y * ey_dot

        )

        uz = (

            self.k1z * ez

            +

            self.k2z * ez_dot

        )

        # ==================================================
        #             THRUST COMMAND U1
        # ==================================================

        cos_phi = math.cos(phi)

        cos_theta = math.cos(theta)

        denominator = (

            cos_phi
            *
            cos_theta

        )

        if abs(denominator) < 0.05:

            denominator = 0.05

        U1 = (

            self.m

            *

            (uz + self.g)

            /

            denominator

        )

        # ==================================================
        #        INTERMEDIATE VARIABLES
        # ==================================================

        if abs(U1) < 0.01:

            U1 = 0.01

        a = (

            self.m
            *
            ux

            /

            U1

        )

        b = (

            self.m
            *
            uy

            /

            U1

        )

        # ==================================================
        #             DESIRED PITCH
        # ==================================================

        theta_ref = math.atan(

            (

                a * math.cos(psi_r)

                +

                b * math.sin(psi_r)

            )

            /

            max(

                math.cos(phi),

                0.05

            )

        )

        # ==================================================
        #             DESIRED ROLL
        # ==================================================

        phi_ref = math.atan(

            (

                a * math.sin(psi_r)

                -

                b * math.cos(psi_r)

            )

            /

            max(

                math.cos(theta_ref),

                0.05

            )

        )

        # ==================================================
        #              OUTPUT VECTOR
        # ==================================================

        return {

            'U1': U1,

            'phi_ref': phi_ref,

            'theta_ref': theta_ref,

            'psi_ref': psi_r,

            'ux': ux,
            'uy': uy,
            'uz': uz,

            'ex': ex,
            'ey': ey,
            'ez': ez

        }