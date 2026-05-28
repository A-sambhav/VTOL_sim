#!/usr/bin/env python3

# ==========================================================
#                       IMPORTS
# ==========================================================

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import time

import math

# ==========================================================
#              TRAJECTORY IMPORTS
# ==========================================================

from vtol_planner.circle_trajectory import CircleTrajectory

from vtol_planner.transition_trajectory import TransitionTrajectory


# ==========================================================
#                    PLANNER NODE
# ==========================================================

class PlannerNode(Node):

    def __init__(self):

        super().__init__('planner_node')

        # ==================================================
        #                  PUBLISHER
        # ==================================================

        self.reference_pub = self.create_publisher(

            Float32MultiArray,
            '/reference_states',
            10

        )

        # ==================================================
        #                 SAMPLE TIME
        # ==================================================

        self.dt = 0.01

        # ==================================================
        #                  TIMER
        # ==================================================

        self.timer = self.create_timer(

            self.dt,
            self.publish_reference

        )

        # ==================================================
        #                  START TIME
        # ==================================================

        self.start_time = time.time()

        # ==================================================
        #               CURRENT MODE
        # ==================================================
        #
        # OPTIONS:
        #
        # hover
        # circle
        # line
        # transition
        #
        # ==================================================

        self.mode = 'transition'

        # ==================================================
        #             TRAJECTORY OBJECTS
        # ==================================================

        self.circle_trajectory = CircleTrajectory(

            radius=5.0,
            altitude=5.0,
            angular_velocity=0.2

        )

        self.transition_trajectory = (

            TransitionTrajectory()

        )

        # ==================================================
        #                   LOGGER
        # ==================================================

        self.get_logger().info(

            f'Planner Node Started | Mode: {self.mode}'

        )

    # ======================================================
    #             PUBLISH REFERENCE STATES
    # ======================================================

    def publish_reference(self):

        # ==================================================
        #                     TIME
        # ==================================================

        t = time.time() - self.start_time

        # ==================================================
        #               HOVER TRAJECTORY
        # ==================================================

        if self.mode == 'hover':

            x_d = 0.0

            y_d = 0.0

            z_d = 5.0

            vx_d = 0.0

            vy_d = 0.0

            vz_d = 0.0

            psi_d = 0.0

            phase = 'HOVER'

        # ==================================================
        #              CIRCLE TRAJECTORY
        # ==================================================

        elif self.mode == 'circle':

            ref = self.circle_trajectory.generate(t)

            x_d = ref[0]

            y_d = ref[1]

            z_d = ref[2]

            vx_d = ref[3]

            vy_d = ref[4]

            vz_d = ref[5]

            psi_d = ref[6]

            phase = 'CIRCLE'

        # ==================================================
        #               LINE TRAJECTORY
        # ==================================================

        elif self.mode == 'line':

            velocity = 2.0

            x_d = velocity * t

            y_d = 0.0

            z_d = 5.0

            vx_d = velocity

            vy_d = 0.0

            vz_d = 0.0

            psi_d = 0.0

            phase = 'LINE'

        # ==================================================
        #           TRANSITION TRAJECTORY
        # ==================================================

        elif self.mode == 'transition':

            ref = self.transition_trajectory.generate(t)

            x_d = ref['x_d']

            y_d = ref['y_d']

            z_d = ref['z_d']

            vx_d = ref['vx_d']

            vy_d = ref['vy_d']

            vz_d = ref['vz_d']

            psi_d = ref['psi_d']

            phase = ref['phase']

        # ==================================================
        #                   DEFAULT
        # ==================================================

        else:

            x_d = 0.0

            y_d = 0.0

            z_d = 0.0

            vx_d = 0.0

            vy_d = 0.0

            vz_d = 0.0

            psi_d = 0.0

            phase = 'DEFAULT'

        # ==================================================
        #             CREATE REFERENCE MSG
        # ==================================================

        msg = Float32MultiArray()

        # ==================================================
        #              REFERENCE VECTOR
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
        #   psi_d
        # ]
        #
        # ==================================================

        msg.data = [

            x_d,
            y_d,
            z_d,

            vx_d,
            vy_d,
            vz_d,

            psi_d

        ]

        # ==================================================
        #                  PUBLISH
        # ==================================================

        self.reference_pub.publish(msg)

        # ==================================================
        #                    DEBUG
        # ==================================================

        self.get_logger().info(

            f'[{phase}] | '
            f'X={x_d:.2f} | '
            f'Y={y_d:.2f} | '
            f'Z={z_d:.2f} | '
            f'Vx={vx_d:.2f} | '
            f'Vy={vy_d:.2f} | '
            f'Vz={vz_d:.2f} | '
            f'Yaw={math.degrees(psi_d):.2f}'

        )


# ==========================================================
#                         MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = PlannerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


# ==========================================================
#                        ENTRY
# ==========================================================

if __name__ == '__main__':

    main()