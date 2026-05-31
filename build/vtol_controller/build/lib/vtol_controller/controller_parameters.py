#!/usr/bin/env python3

# ==========================================================
#              CONTROLLER PARAMETERS
# ==========================================================

class ControllerParameters:

    def __init__(self):

        # ==================================================
        #               VEHICLE PARAMETERS
        # ==================================================

        self.mass = 2.0          # kg

        self.gravity = 9.81      # m/s²
        # ==================================================
        #           INERTIA PARAMETERS
        # ==================================================

        self.Ixx = 0.525

        self.Iyy = 0.459

        self.Izz = 0.974

        self.Ixz = 0.02
        # ==================================================
        #         FEEDBACK LINEARIZATION GAINS
        # ==================================================
        #
        # Error Dynamics:
        #
        # ë = K1 e + K2 ė
        #
        # Characteristic Equation:
        #
        # λ² - K2 λ - K1 = 0
        #
        # Desired Poles:
        #
        # λ1 = -1
        # λ2 = -2
        #
        # => λ² + 3λ + 2 = 0
        #
        # Therefore:
        #
        # K1 = -2
        # K2 = -3
        #
        # ==================================================

        self.k1x = 2.0
        self.k2x = 3.0

        self.k1y = 2.0
        self.k2y = 3.0

        self.k1z = 2.0
        self.k2z = 3.0

        # ==================================================
        #          SAFETY LIMITS
        # ==================================================

        self.max_roll = 0.785398      # 45 deg

        self.max_pitch = 0.785398     # 45 deg

        self.max_yaw_rate = 1.5       # rad/s

        self.max_thrust = 100.0       # N

        self.min_thrust = 0.0         # N

        # ==================================================
        #              MPC PARAMETERS
        # ==================================================
        #
        # Will be used later
        #
        # ==================================================

        self.prediction_horizon = 20

        self.control_horizon = 10

        self.controller_dt = 0.01

        # ==================================================
        #              ATTITUDE WEIGHTS
        # ==================================================

        self.q_phi = 10.0

        self.q_theta = 10.0

        self.q_psi = 5.0

        self.q_p = 1.0

        self.q_q = 1.0

        self.q_r = 1.0

        # ==================================================
        #              INPUT WEIGHTS
        # ==================================================

        self.r_u2 = 0.1

        self.r_u3 = 0.1

        self.r_u4 = 0.1