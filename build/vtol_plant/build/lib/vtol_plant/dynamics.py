# ==========================================================
#                     IMPORTS
# ==========================================================

import numpy as np

import math

from vtol_plant.allocator import ControlAllocator


# ==========================================================
#                   VTOL DYNAMICS
# ==========================================================

class VTOLDynamics:

    def __init__(self, params):

        # ==================================================
        #                  PARAMETERS
        # ==================================================

        self.params = params

        # ==================================================
        #                 ALLOCATOR
        # ==================================================

        self.allocator = ControlAllocator(

            self.params

        )

    # ======================================================
    #                STATE DERIVATIVE
    # ======================================================
    #
    # X =
    #
    # [u,v,w,p,q,r,x,y,z,phi,theta,psi]
    #
    # U =
    #
    # [U1,U2,U3,U4]
    #
    # ======================================================

    def state_derivative(

        self,
        X,
        U

    ):

        # ==================================================
        #                 EXTRACT STATES
        # ==================================================

        u = X[0]
        v = X[1]
        w = X[2]

        p = X[3]
        q = X[4]
        r = X[5]

        x = X[6]
        y = X[7]
        z = X[8]

        phi = X[9]
        theta = X[10]
        psi = X[11]

        # ==================================================
        #              EXTRACT CONTROL INPUTS
        # ==================================================

        U1 = U[0]

        U2 = U[1]

        U3 = U[2]

        U4 = U[3]

        # ==================================================
        #            COMPUTE MOTOR VELOCITIES
        # ==================================================

        omega = self.allocator.allocate(

            U1,
            U2,
            U3,
            U4

        )

        omega1 = omega[0]

        omega2 = omega[1]

        omega3 = omega[2]

        omega4 = omega[3]

        # ==================================================
        #         TOTAL PROPELLER ANGULAR SPEED
        # ==================================================
        #
        # CW  : Motor 1, 3
        # CCW : Motor 2, 4
        #
        # ==================================================

        Omega = (

            -omega1
            + omega2
            - omega3
            + omega4

        )

        # ==================================================
        #           TRANSLATIONAL DYNAMICS
        # ==================================================

        Fx = 0.0
        Fy = 0.0
        Fz = U1

        u_dot = (

            Fx / self.params.mass

            - q*w   

            + r*v

            - self.params.gravity
            * math.sin(theta)

)

        v_dot = (

            Fy / self.params.mass

            - r*u

            + p*w

            + self.params.gravity
            * math.cos(theta)
            * math.sin(phi)

)

        w_dot = (

    Fz / self.params.mass

    - p*v

    + q*u

    - self.params.gravity
      * math.cos(theta)
      * math.cos(phi)

)     
        D = (

    self.params.Ixx
    * self.params.Izz

    - self.params.Ixz**2

)

        Mx = U2
        My = U3
        Mz = U4

        # ==================================================
        #             ROTATIONAL DYNAMICS
        # ==================================================

        p_dot = (

        Mx * self.params.Izz

        + Mz * self.params.Ixz

        + (

        self.params.Ixx
        * self.params.Ixz

        - self.params.Iyy
        * self.params.Ixz

        + self.params.Izz
        * self.params.Ixz

         ) * p * q

        + (

        self.params.Iyy
        * self.params.Izz

        - self.params.Ixz**2

        - self.params.Izz**2

           ) * q * r

           ) / D

        q_dot = (

            My

            + (

        self.params.Izz
        - self.params.Ixx

            ) * p * r

            - self.params.Ixz
            * (

            p**2
            - r**2

            )

            ) / self.params.Iyy
        r_dot = (

        My * self.params.Izz

        + Mx * self.params.Ixz

        + (

        self.params.Ixz**2

        + self.params.Ixx**2

        - self.params.Ixx
          * self.params.Iyy

        ) * p * q

        + (

        self.params.Ixz
        * self.params.Iyy

        - self.params.Ixz
          * self.params.Izz

        - self.params.Ixz
          * self.params.Ixx

        ) * q * r

        ) / D

        # ==================================================
        #              EULER KINEMATICS
        # ==================================================

        phi_dot = (

            p

            + q * math.sin(phi)
            * math.tan(theta)

            + r * math.cos(phi)
            * math.tan(theta)

        )

        theta_dot = (

            q * math.cos(phi)

            - r * math.sin(phi)

        )

        psi_dot = (

            (

                q * math.sin(phi)

                + r * math.cos(phi)

            )

            / math.cos(theta)

        )

        # ==================================================
        #          BODY TO INERTIAL TRANSFORM
        # ==================================================

        x_dot = (

            (

                math.cos(theta)
                * math.cos(psi)

            ) * u

            +

            (

                math.sin(phi)
                * math.sin(theta)
                * math.cos(psi)

                - math.cos(phi)
                * math.sin(psi)

            ) * v

            +

            (

                math.cos(phi)
                * math.sin(theta)
                * math.cos(psi)

                + math.sin(phi)
                * math.sin(psi)

            ) * w

        )

        y_dot = (

            (

                math.cos(theta)
                * math.sin(psi)

            ) * u

            +

            (

                math.sin(phi)
                * math.sin(theta)
                * math.sin(psi)

                + math.cos(phi)
                * math.cos(psi)

            ) * v

            +

            (

                math.cos(phi)
                * math.sin(theta)
                * math.sin(psi)

                - math.sin(phi)
                * math.cos(psi)

            ) * w

        )

        z_dot = (

            (

                -math.sin(theta)

            ) * u

            +

            (

                math.sin(phi)
                * math.cos(theta)

            ) * v

            +

            (

                math.cos(phi)
                * math.cos(theta)

            ) * w

        )

        # ==================================================
        #             RETURN STATE DERIVATIVE
        # ==================================================

        X_dot = np.array([

            u_dot,
            v_dot,
            w_dot,

            p_dot,
            q_dot,
            r_dot,

            x_dot,
            y_dot,
            z_dot,

            phi_dot,
            theta_dot,
            psi_dot

        ])

        return X_dot