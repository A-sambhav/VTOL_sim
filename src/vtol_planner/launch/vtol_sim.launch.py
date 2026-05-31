from launch import LaunchDescription

from launch_ros.actions import Node

import os

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    rviz_config = os.path.join(

        get_package_share_directory(
            'vtol_planner'
        ),

        'rviz',

        'planner.rviz'

    )

    return LaunchDescription([

        # ==============================================
        # Planner
        # ==============================================

        Node(
            package='vtol_planner',
            executable='planner_node',
            output='screen'
        ),

        # ==============================================
        # Controller
        # ==============================================

        Node(
            package='vtol_controller',
            executable='controller_node',
            output='screen'
        ),

        # ==============================================
        # Plant
        # ==============================================

        Node(
            package='vtol_plant',
            executable='plant_node',
            output='screen'
        ),

        # ==============================================
        # Trajectory Visualizer
        # ==============================================

        Node(
            package='vtol_planner',
            executable='trajectory_visualizer',
            output='screen'
        ),

        # ==============================================
        # State Visualizer
        # ==============================================

        Node(
            package='vtol_plant',
            executable='state_visualizer',
            output='screen'
        ),

        # ==============================================
        # TF
        # ==============================================

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0','0','0',
                '0','0','0',
                'map',
                'world'
            ]
        ),

        # ==============================================
        # RViz
        # ==============================================

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                rviz_config
            ],
            output='screen'
        )

    ])