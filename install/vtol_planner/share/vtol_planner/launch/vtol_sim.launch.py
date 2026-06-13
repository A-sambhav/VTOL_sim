from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)


from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)

import os


def generate_launch_description():

    display_launch = os.path.join(

        get_package_share_directory(
            'vtol_description'
        ),

        'launch',

        'display.launch.py'

    )

    return LaunchDescription([

        # ==============================================
        # VTOL DESCRIPTION
        # ==============================================

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                display_launch

            )

        ),

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
        # Sensor Simulator
        # ==============================================

        Node(
            package='sensor_simulator',
            executable='sensor_simulator_node',
            output='screen'
        ),

        # ==============================================
        # State Estimator
        # ==============================================

        Node(
            package='state_estimator',
            executable='state_estimator_node',
            output='screen'
        ),

        # ==============================================
        # ODOM -> TF
        # ==============================================

        Node(
            package='vtol_plant',
            executable='odom_tf_broadcaster',
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
        # Estimated State Visualizer
        # ==============================================

        Node(
            package='vtol_plant',
            executable='estimated_state_visualizer',
            output='screen'
        ),
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
        '0', '0', '0',
        '0', '0', '0',
        'map',
        'world'
    ]
    ),


    ])