from launch import LaunchDescription

from launch_ros.actions import Node

import os

from ament_index_python.packages import (

    get_package_share_directory

)


# ==========================================================
#                 PLANNER LAUNCH FILE
# ==========================================================

def generate_launch_description():

    # ======================================================
    #                 RVIZ CONFIG PATH
    # ======================================================

    rviz_config = os.path.join(

        get_package_share_directory(

            'vtol_planner'

        ),

        'rviz',

        'planner.rviz'

    )

    return LaunchDescription([

        # ==================================================
        #                  PLANNER NODE
        # ==================================================

        Node(

            package='vtol_planner',

            executable='planner_node',

            name='planner_node',

            output='screen'

        ),

        # ==================================================
        #            TRAJECTORY VISUALIZER
        # ==================================================

        Node(

            package='vtol_planner',

            executable='trajectory_visualizer',

            name='trajectory_visualizer',

            output='screen'

        ),

        # ==================================================
        #              STATIC TF PUBLISHER
        # ==================================================

        Node(

            package='tf2_ros',

            executable='static_transform_publisher',

            name='map_to_world_tf',

            arguments=[

                '0', '0', '0',

                '0', '0', '0',

                'map',

                'world'

            ],

            output='screen'

        ),
        # ==================================================
        #              STATE VISUALIZER
        # ==================================================

        Node(

            package='vtol_plant',

            executable='state_visualizer',

            name='state_visualizer',

            output='screen'

        ),

        # ==================================================
        #                     RVIZ2
        # ==================================================

        Node(

            package='rviz2',

            executable='rviz2',

            name='rviz2',

            arguments=[

                '-d',

                rviz_config

            ],

            output='screen'

        )

    ])