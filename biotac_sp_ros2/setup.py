#!/usr/bin/env python3
"""
******************************************************************************
*  @file    setup.py
*  @author  Pedro Machado (pedro.machado@ntu.ac.uk)
*  @brief   Setup file for the biotac_sp_ros2 package.
*
*  @details This setup script installs the biotac_sensors package, which wraps
*           Syntouch's BioTac / Cheetah C libraries for use with ROS 2.
*
*  @version 0.1.0
*  @date    May 2025
*
*  @copyright GPLv3
*
*  SPDX-License-Identifier: GPLv3
******************************************************************************
"""

from setuptools import setup
import os
from glob import glob

package_name = 'biotac_sp_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pedro Machado',
    maintainer_email='pedro.machado@ntu.ac.uk',
    description='The biotac_sensors package wraps Syntouch\'s BioTac / Cheetah C libraries for use with ROS2.',
    license='GPLv3',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    entry_points={
        'console_scripts': [
            'biotac_driver = biotac_sp_ros2.biotac_sp_ros2:main',  # if you have it
            'visualise_biotac = biotac_sp_ros2.visualise_biotac:main',
        ],
    },
)
