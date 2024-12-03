#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, RegisterEventHandler


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )
    )

    
    controller_params = PathJoinSubstitution(
        [
            FindPackageShare("cart_pole_lypunov_lqr"),
            "config",
            "controller.yaml",
        ]
    )

    cart_pole_lypunov_lqr_node = Node(
        package="cart_pole_lypunov_lqr",
        executable="controller",
        parameters=[controller_params, {'use_sim_time': True}],
        #parameters=[{'use_sim_time': True}], 
    )

    actions = [
        cart_pole_lypunov_lqr_node,
    ]

    return LaunchDescription(declared_arguments + actions)
