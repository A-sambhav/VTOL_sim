import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Package Directory
    vtol_description_dir = get_package_share_directory(
        "vtol_description"
    )

    # URDF/XACRO Path
    default_model_path = os.path.join(
        vtol_description_dir,
        "urdf",
        "vtol_description.urdf.xacro"
    )

    # RViz Config Path
    default_rviz_config_path = os.path.join(
        vtol_description_dir,
        "rviz",
        "display.rviz"
    )

    # Launch Argument
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Absolute path to robot urdf file"
    )

    # Robot Description
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model")
        ]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description
        }],
        output="screen"
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            default_rviz_config_path
        ],
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])