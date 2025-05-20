#!/usr/bin/env python3
"""
******************************************************************************
*  @file    biotac_launch.py
*  @author  Pedro Machado (pedro.machado@ntu.ac.uk)
*  @brief   ROS 2 launch file for starting the BioTac sensor interface and
*           visualizer nodes.
*
*  @details This launch file initializes the ROS 2 nodes for interfacing with
*           Syntouch's BioTac sensors via the Cheetah SPI interface and launches
*           a visualization node for the sensor data.
*
*  @version 0.1.0
*  @date    May 2025
*
*  @copyright BSD
*
*  SPDX-License-Identifier: BSD-3-Clause
******************************************************************************
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='biotac_sp_ros2',
            executable='visualise_biotac',
            name='visualise_biotac',
            output='screen',
        ),
    ])
