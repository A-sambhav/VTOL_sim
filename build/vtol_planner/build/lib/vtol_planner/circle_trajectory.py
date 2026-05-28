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

        # ==================================================
        #                 POSITION REFERENCES
        # ==================================================

        x_d = (

            self.center_x

            + self.radius

            * math.cos(

                self.angular_velocity * t

            )

        )

        y_d = (

            self.center_y

            + self.radius

            * math.sin(

                self.angular_velocity * t

            )

        )

        z_d = self.altitude

        # ==================================================
        #                 VELOCITY REFERENCES
        # ==================================================

        vx_d = (

            -self.radius
            * self.angular_velocity

            * math.sin(

                self.angular_velocity * t

            )

        )

        vy_d = (

            self.radius
            * self.angular_velocity

            * math.cos(

                self.angular_velocity * t

            )

        )

        vz_d = 0.0

        # ==================================================
        #                   YAW REFERENCE
        # ==================================================
        #
        # Tangent direction of trajectory
        #
        # ==================================================

        psi_d = math.atan2(

            vy_d,
            vx_d

        )

        # ==================================================
        #             RETURN REFERENCE VECTOR
        # ==================================================
        #
        # [x_d,
        #  y_d,
        #  z_d,
        #  vx_d,
        #  vy_d,
        #  vz_d,
        #  psi_d]
        #
        # ==================================================

        return [

            x_d,
            y_d,
            z_d,

            vx_d,
            vy_d,
            vz_d,

            psi_d

        ]