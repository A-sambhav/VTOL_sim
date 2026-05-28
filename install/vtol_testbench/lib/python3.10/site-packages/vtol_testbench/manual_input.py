#!/usr/bin/env python3

# ==========================================================
#                       IMPORTS
# ==========================================================

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


# ==========================================================
#                  MANUAL INPUT NODE
# ==========================================================

class ManualInputNode(Node):

    def __init__(self):

        super().__init__('manual_input_node')

        # ==================================================
        #                  PUBLISHER
        # ==================================================

        self.control_pub = self.create_publisher(

            Float32MultiArray,
            '/control_inputs',
            10

        )

        self.get_logger().info(

            'Manual Input Node Started'

        )

        # ==================================================
        #                   MAIN LOOP
        # ==================================================

        self.run()

    # ======================================================
    #                     MAIN LOOP
    # ======================================================

    def run(self):

        while rclpy.ok():

            print('\n===================================')
            print('      VTOL MANUAL INPUT')
            print('===================================')

            try:

                U1 = float(

                    input('U1 (Total Thrust): ')

                )

                U2 = float(

                    input('U2 (Roll Torque): ')

                )

                U3 = float(

                    input('U3 (Pitch Torque): ')

                )

                U4 = float(

                    input('U4 (Yaw Torque): ')

                )

                # ==========================================
                #             CREATE MESSAGE
                # ==========================================

                msg = Float32MultiArray()

                msg.data = [

                    U1,
                    U2,
                    U3,
                    U4

                ]

                # ==========================================
                #               PUBLISH
                # ==========================================

                self.control_pub.publish(msg)

                self.get_logger().info(

                    f'Published: '
                    f'U1={U1:.2f}, '
                    f'U2={U2:.2f}, '
                    f'U3={U3:.2f}, '
                    f'U4={U4:.2f}'

                )

            except Exception as e:

                print(f'Error: {e}')


# ==========================================================
#                         MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = ManualInputNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


# ==========================================================
#                        ENTRY
# ==========================================================

if __name__ == '__main__':

    main()