#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo


def generate_launch_description():
    controller_params = PathJoinSubstitution(
        [
            FindPackageShare("cart_pole_drake"),
            "config",
            "lqr_with_swing_up_controller.yaml",
        ]
    )

    pendulum_params = PathJoinSubstitution(
        [
            FindPackageShare("cart_pole_drake"),
            "config",
            "pendulum_parameters.yaml",
        ]
    )

    cart_pole_controller_node = Node(
        package="cart_pole_drake",
        executable="lqr_with_swing_up_controller_node",
        parameters=[controller_params, pendulum_params],
    )

    actions = [
        cart_pole_controller_node,
    ]

    return LaunchDescription(actions)
