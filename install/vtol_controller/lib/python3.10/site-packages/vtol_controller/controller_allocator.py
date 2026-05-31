#!/usr/bin/env python3

# ==========================================================
#                CONTROLLER ALLOCATOR
# ==========================================================

import math

from vtol_controller.controller_parameters import ControllerParameters


# ==========================================================
#                CONTROLLER ALLOCATOR
# ==========================================================

class ControllerAllocator:

    def __init__(self):

        self.params = ControllerParameters()

    # ======================================================
    #                 LIMIT FUNCTION
    # ======================================================

    def constrain(

        self,
        value,
        minimum,
        maximum

    ):

        return max(

            minimum,

            min(

                value,
                maximum

            )

        )

    # ======================================================
    #              BUILD MPC COMMAND
    # ======================================================

    def allocate(

        self,

        U1,
        phi_ref,
        theta_ref,
        psi_ref

    ):

        # ==================================================
        #              SAFETY LIMITS
        # ==================================================

        U1 = self.constrain(

            U1,

            self.params.min_thrust,

            self.params.max_thrust

        )

        phi_ref = self.constrain(

            phi_ref,

            -self.params.max_roll,

            self.params.max_roll

        )

        theta_ref = self.constrain(

            theta_ref,

            -self.params.max_pitch,

            self.params.max_pitch

        )

        # ==================================================
        #               MPC COMMAND
        # ==================================================

        command = {

            'U1': U1,

            'phi_ref': phi_ref,

            'theta_ref': theta_ref,

            'psi_ref': psi_ref

        }

        return command

    # ======================================================
    #             VECTOR FORMAT (OPTIONAL)
    # ======================================================

    def allocate_vector(

        self,

        U1,
        phi_ref,
        theta_ref,
        psi_ref

    ):

        command = self.allocate(

            U1,
            phi_ref,
            theta_ref,
            psi_ref

        )

        return [

            command['U1'],

            command['phi_ref'],

            command['theta_ref'],

            command['psi_ref']

        ]