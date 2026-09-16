"""Bring up ros2_control for Pebble with a selectable hardware backend.

    ros2 launch rocky_control control.launch.py hw:=mock     # kinematic echo
    ros2 launch rocky_control control.launch.py hw:=topic    # Python driver bridge
    ros2 launch rocky_control control.launch.py hw:=serial   # C++ plugin (later)

hw:=topic additionally starts rocky_driver/hw_bridge_node.py, which owns the
real Feetech bus via the tested pure-Python driver (driver/rocky_driver).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hw = LaunchConfiguration("hw")
    desc_pkg = get_package_share_directory("rocky_description")
    ctrl_pkg = get_package_share_directory("rocky_control")
    xacro_file = os.path.join(desc_pkg, "urdf", "pebble.urdf.xacro")
    controllers = os.path.join(ctrl_pkg, "config", "controllers.yaml")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file,
                 " hw:=", hw]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("hw", default_value="mock",
                              choices=["mock", "topic", "serial"]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[{"robot_description": robot_description}, controllers],
             output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["leg_position_controller"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["claw_position_controller"]),
        Node(package="rocky_driver", executable="hw_bridge_node.py",
             condition=LaunchConfigurationEquals("hw", "topic"),
             output="screen"),
        Node(package="rocky_gait", executable="gait_node", output="screen"),
    ])
