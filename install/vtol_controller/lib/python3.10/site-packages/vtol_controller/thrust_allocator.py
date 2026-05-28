#!/usr/bin/env python3

# ==========================================================
#                 PHYSICAL THRUST ALLOCATOR
# ==========================================================

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import JointState

from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.boolean_pb2 import Boolean

from gz.transport13 import Node as GzNode


class ThrustAllocator(Node):

    def __init__(self):

        super().__init__('thrust_allocator')

        # ==================================================
        #              THRUST COEFFICIENT
        # ==================================================

        self.k_thrust = 0.0005

        # ==================================================
        #                 MOTOR LINKS
        # ==================================================

        self.motor_map = {

            'motor1_joint': 'motor1',
            'motor2_joint': 'motor2',
            'motor3_joint': 'motor3',
            'motor4_joint': 'motor4',

        }

        # ==================================================
        #             GAZEBO TRANSPORT NODE
        # ==================================================

        self.gz_node = GzNode()

        # ==================================================
        #              JOINT STATE SUBSCRIBER
        # ==================================================

        self.subscription = self.create_subscription(

            JointState,
            '/joint_states',
            self.joint_state_callback,
            10

        )

        self.get_logger().info(

            'Physical Thrust Allocator Started'

        )

    # ======================================================
    #                 GAZEBO CALLBACK
    # ======================================================

    def gz_callback(self, response):

        pass

    # ======================================================
    #                 APPLY FORCE FUNCTION
    # ======================================================

    def apply_force(self, link_name, thrust):

        msg = EntityWrench()

        # ==================================================
        #                   ENTITY NAME
        # ==================================================

        msg.entity.name = link_name

        # ==================================================
        #                  FORCE VECTOR
        # ==================================================

        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = thrust

        # ==================================================
        #                APPLY LINK WRENCH
        # ==================================================

        try:

            result = self.gz_node.request(

                '/world/default/apply_link_wrench',

                msg,

                Boolean,

                self.gz_callback,

                1000

            )

            self.get_logger().info(

                f'Applied {thrust:.2f} N to {link_name}'

            )

        except Exception as e:

            self.get_logger().error(

                f'Failed to apply force: {str(e)}'

            )

    # ======================================================
    #               JOINT STATE CALLBACK
    # ======================================================

    def joint_state_callback(self, msg):

        joint_velocity_map = {}

        for i in range(len(msg.name)):

            joint_velocity_map[msg.name[i]] = msg.velocity[i]

        # ==================================================
        #                APPLY THRUST
        # ==================================================

        for joint_name, link_name in self.motor_map.items():

            if joint_name in joint_velocity_map:

                omega = joint_velocity_map[joint_name]

                # ==================================================
                #                THRUST EQUATION
                # ==================================================

                thrust = self.k_thrust * (omega ** 2)

                # ==================================================
                #              APPLY PHYSICAL FORCE
                # ==================================================

                self.apply_force(link_name, thrust)

                self.get_logger().info(

                    f'{joint_name} | '
                    f'omega = {omega:.2f} | '
                    f'thrust = {thrust:.2f} N'

                )


# ==========================================================
#                         MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = ThrustAllocator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()