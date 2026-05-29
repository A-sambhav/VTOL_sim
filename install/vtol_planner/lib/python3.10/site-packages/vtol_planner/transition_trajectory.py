# ==========================================================
#             VTOL TRANSITION TRAJECTORY
# ==========================================================

import math


# ==========================================================
#           VTOL TRANSITION TRAJECTORY CLASS
# ==========================================================

class TransitionTrajectory:

    def __init__(self):

        # ==================================================
        #                 MISSION PARAMETERS
        # ==================================================

        self.takeoff_altitude = 10.0

        self.cruise_distance = 50.0

        self.forward_velocity = 5.0

        self.transition_time = 5.0

        self.hover_time = 3.0

        self.descent_rate = 1.0

    # ======================================================
    #              GENERATE TRAJECTORY
    # ======================================================

    def generate(self, t):
        # ==================================================
        #             PHASE 1 : TAKEOFF
        # ==================================================

        if t < 10.0:

            x_d = 0.0
            y_d = 0.0

            z_d = (

                self.takeoff_altitude
                * (t / 10.0)

            )

            vx_d = 0.0
            vy_d = 0.0
            vz_d = 1.0

            ax_d = 0.0
            ay_d = 0.0
            az_d = 0.0

            psi_d = 0.0

            phase = 'TAKEOFF'

        # ==================================================
        #             PHASE 2 : HOVER
        # ==================================================

        elif t < 13.0:

            x_d = 0.0
            y_d = 0.0
            z_d = self.takeoff_altitude

            vx_d = 0.0
            vy_d = 0.0
            vz_d = 0.0

            ax_d = 0.0
            ay_d = 0.0
            az_d = 0.0

            psi_d = 0.0

            phase = 'HOVER'

        # ==================================================
        #         PHASE 3 : TRANSITION FORWARD
        # ==================================================

        elif t < 18.0:

            tau = (

                (t - 13.0)

                / self.transition_time

            )

            x_d = (

                0.5
                * self.forward_velocity
                * tau**2
                * self.transition_time

            )

            y_d = 0.0

            z_d = self.takeoff_altitude

            vx_d = (

                self.forward_velocity
                * tau

            )

            vy_d = 0.0
            vz_d = 0.0

            ax_d = (

                self.forward_velocity
                / self.transition_time

            )

            ay_d = 0.0
            az_d = 0.0

            psi_d = 0.0

            phase = 'TRANSITION_FORWARD'

        # ==================================================
        #             PHASE 4 : CRUISE
        # ==================================================

        elif t < 28.0:

            cruise_time = t - 18.0

            x_d = (

                self.forward_velocity
                * cruise_time

            )

            y_d = 0.0

            z_d = self.takeoff_altitude

            vx_d = self.forward_velocity

            vy_d = 0.0
            vz_d = 0.0

            ax_d = 0.0
            ay_d = 0.0
            az_d = 0.0

            psi_d = 0.0

            phase = 'CRUISE'

        # ==================================================
        #         PHASE 5 : TRANSITION BACK
        # ==================================================

        elif t < 33.0:

            tau = (

                (t - 28.0)

                / self.transition_time

            )

            remaining_velocity = (

                self.forward_velocity
                * (1.0 - tau)

            )

            x_d = (

                self.forward_velocity * 10.0

                +

                self.forward_velocity
                * (t - 28.0)

                -

                0.5
                * self.forward_velocity
                * tau**2
                * self.transition_time

            )

            y_d = 0.0

            z_d = self.takeoff_altitude

            vx_d = remaining_velocity

            vy_d = 0.0
            vz_d = 0.0

            ax_d = (

                -self.forward_velocity
                / self.transition_time

            )

            ay_d = 0.0
            az_d = 0.0

            psi_d = 0.0

            phase = 'TRANSITION_BACK'

        # ==================================================
        #             PHASE 6 : LANDING
        # ==================================================

        else:

            descent_time = t - 33.0

            x_d = (

                self.forward_velocity
                * 10.0

                +

                self.forward_velocity
                * self.transition_time

            )

            y_d = 0.0

            z_d = max(

                0.0,

                self.takeoff_altitude

                -

                self.descent_rate
                * descent_time

            )

            vx_d = 0.0
            vy_d = 0.0

            vz_d = -self.descent_rate

            ax_d = 0.0
            ay_d = 0.0
            az_d = 0.0

            psi_d = 0.0

            phase = 'LANDING'

        # ==================================================
        #             RETURN REFERENCE VECTOR
        # ==================================================

        return {

            'x_d': x_d,
            'y_d': y_d,
            'z_d': z_d,

            'vx_d': vx_d,
            'vy_d': vy_d,
            'vz_d': vz_d,

            'ax_d': ax_d,
            'ay_d': ay_d,
            'az_d': az_d,

            'psi_d': psi_d,

            'phase': phase

        }