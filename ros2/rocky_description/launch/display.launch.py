"""RViz sanity view:  ros2 launch rocky_description display.launch.py"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("rocky_description")
    xacro_file = os.path.join(pkg, "urdf", "pebble.urdf.xacro")
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file, " hw:=mock"]),
        value_type=str)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher_gui",
             executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2"),
    ])
