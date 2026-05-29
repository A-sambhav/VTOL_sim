# ==========================================================
#                CIRCLE TRAJECTORY GENERATOR
# ==========================================================

import math


# ==========================================================
#                CIRCLE TRAJECTORY CLASS
# ==========================================================

class CircleTrajectory:

    def __init__(

        self,
        radius=5.0,
        altitude=5.0,
        angular_velocity=0.2,
        center_x=0.0,
        center_y=0.0

    ):

        # ==================================================
        #                TRAJECTORY PARAMETERS
        # ==================================================

        self.radius = radius

        self.altitude = altitude

        self.angular_velocity = angular_velocity

        self.center_x = center_x

        self.center_y = center_y

    # ======================================================
    #              GENERATE REFERENCE STATES
    # ======================================================

    def generate(self, t):

        omega = self.angular_velocity

        # ==================================================
        #                 POSITION REFERENCES
        # ==================================================

        x_d = (

            self.center_x

            + self.radius

            * math.cos(

                omega * t

            )

        )

        y_d = (

            self.center_y

            + self.radius

            * math.sin(

                omega * t

            )

        )

        z_d = self.altitude

        # ==================================================
        #                 VELOCITY REFERENCES
        # ==================================================

        vx_d = (

            -self.radius

            * omega

            * math.sin(

                omega * t

            )

        )

        vy_d = (

            self.radius

            * omega

            * math.cos(

                omega * t

            )

        )

        vz_d = 0.0

        # ==================================================
        #              ACCELERATION REFERENCES
        # ==================================================

        ax_d = (

            -self.radius

            * omega**2

            * math.cos(

                omega * t

            )

        )

        ay_d = (

            -self.radius

            * omega**2

            * math.sin(

                omega * t

            )

        )

        az_d = 0.0

        # ==================================================
        #                   YAW REFERENCE
        # ==================================================

        psi_d = math.atan2(

            vy_d,

            vx_d

        )

        # ==================================================
        #             RETURN REFERENCE VECTOR
        # ==================================================
        #
        # [
        #   x_d,
        #   y_d,
        #   z_d,
        #
        #   vx_d,
        #   vy_d,
        #   vz_d,
        #
        #   ax_d,
        #   ay_d,
        #   az_d,
        #
        #   psi_d
        # ]
        #
        # ==================================================

        return [

            x_d,
            y_d,
            z_d,

            vx_d,
            vy_d,
            vz_d,

            ax_d,
            ay_d,
            az_d,

            psi_d

        ]