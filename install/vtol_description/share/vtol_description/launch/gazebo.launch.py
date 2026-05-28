import os
from os import pathsep
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)

from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # =========================================================
    # PACKAGE PATH
    # =========================================================

    vtol_description = get_package_share_directory(
        "vtol_description"
    )

    # =========================================================
    # MODEL ARGUMENT
    # =========================================================

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            vtol_description,
            "urdf",
            "vtol_description.urdf.xacro"
        ),
        description="Absolute path to robot urdf file"
    )

    # =========================================================
    # WORLD ARGUMENT
    # =========================================================

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty"
    )

    # =========================================================
    # WORLD PATH
    # =========================================================

    world_path = PathJoinSubstitution([
        vtol_description,
        "worlds",
        PythonExpression([
            "'",
            LaunchConfiguration("world_name"),
            "' + '.sdf'"
        ])
    ])

    # =========================================================
    # GAZEBO RESOURCE PATH
    # =========================================================

    model_path = str(
        Path(vtol_description).parent.resolve()
    )

    model_path += pathsep + os.path.join(
        vtol_description,
        "meshes"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=model_path
    )

    # =========================================================
    # ROS DISTRO CHECK
    # =========================================================

    ros_distro = os.environ["ROS_DISTRO"]

    is_ignition = "True" if ros_distro == "humble" else "False"

    # =========================================================
    # ROBOT DESCRIPTION
    # =========================================================

    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    # =========================================================
    # ROBOT STATE PUBLISHER
    # =========================================================

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ]
    )

    # =========================================================
    # GAZEBO SIMULATION
    # =========================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(
                    "ros_gz_sim"
                ),
                "launch"
            ),
            "/gz_sim.launch.py"
        ]),

        launch_arguments={
            "gz_args": PythonExpression([
                "'",
                world_path,
                " -v 4 -r'"
            ])
        }.items()
    )

    # =========================================================
    # SPAWN VTOL
    # =========================================================

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",

            "-name",
            "vtol",

            "-z",
            "10"
        ],
    )

    # =========================================================
    # ROS-GAZEBO BRIDGE
    # =========================================================

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",

        arguments=[

            # Clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            # IMU
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],

        remappings=[
            ("/imu", "/imu/out"),
        ]
    )

    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================

    return LaunchDescription([

        model_arg,

        world_name_arg,

        gazebo_resource_path,

        robot_state_publisher_node,

        gazebo,

        gz_spawn_entity,

        gz_ros2_bridge

    ])