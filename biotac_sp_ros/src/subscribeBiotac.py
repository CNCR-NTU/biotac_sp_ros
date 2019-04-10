#!/usr/bin/env python
from __future__ import print_function

# -*- coding: utf-8 -*-
"""
:ABSTRACT:
This script is part of the of the enhanced grasp project

:REQUIRES:

:
:AUTHOR:  Pedro Machado
:ORGANIZATION: Nottingham Trent University
:CONTACT: pedro.baptistamachado@ntu.ac.uk
:SINCE: 10/04/2019
:VERSION: 0.1

This file is part of <project> project.
the Robot 2 Robot interaction project can not be copied and/or distributed without the express
permission of Prof. Martin McGinnity <martin.mcginnity@ntu.ac.uk>

Copyright (C) 2019 All rights reserved, Nottingham Trent University
Computational Neuroscience and Cognitive Robotics Laboratory
email:  pedro.baptistamachado@ntu.ac.uk
website: https://www.ntu.ac.uk/research/groups-and-centres/groups/computational-neuroscience-and-cognitive-robotics-laboratory


"""
# ===============================================================================
# PROGRAM METADATA
# ===============================================================================
__author__ = 'Pedro Machado'
__contact__ = 'pedro.baptistamachado@ntu.ac.uk'
__copyright__ = 'Enhanced grasping project can not be copied and/or distributed \
without the express permission of Prof. Martin McGinnity <martin.mcginnity@ntu.ac.uk'
__license__ = '2019 (C) CNCR@NTU, All rights reserved'
__date__ = '13/02/2019'
__version__ = '0.1'
__file_name__ = 'subscribeBiotac.py'
__description__ = 'Subscribe Biotac snesors'
__compatibility__ = "Python 2 and Python 3"
__platforms__ = "Sawyer and AR10 hand"

#===============================================================================
# IMPORT STATEMENTS
#===============================================================================
import rospy
from std_msgs.msg import String
import numpy as np

#===============================================================================
# GLOBAL VARIABLES DECLARATIONS
#===============================================================================

#===============================================================================
# METHODS
#===============================================================================

#===============================================================================
#  TESTING AREA
#===============================================================================
def callback_biotac(data):
    buffer = data.data
    buffer = buffer.split(',')
    mat = np.asarray(buffer)

    mat = np.asarray(mat)
    fields_name = ["E1", "PAC", "E2", "PAC", "E3", "PAC", "E4", "PAC", "E5", "PAC", "E6", "PAC", "E7", "PAC", "E8",\
                   "PAC", "E9", "PAC", "E10", "PAC", "E11", "PAC", "E12", "PAC", "E13", "PAC", "E14", "PAC", "E15",\
                   "PAC", "E16", "PAC", "E17", "PAC", "E18", "PAC", "E19", "PAC", "E20", "PAC", "E21", "PAC", "E22",\
                   "PAC", "E23", "PAC", "E24", "PAC", "PDC", "PAC", "TAC", "PAC", "TDC", "PAC"]
    print("ROS time:", mat[0])
    for sensor in range(0, 3):
        print("********* Sensor", sensor, "*********\n", fields_name, "\n",
              mat[1 + sensor * len(fields_name):(sensor + 1) * len(fields_name)])

def listener():
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("/biotac_sp_ros", String,callback_biotac)
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down the Biotac subscriber!")
        except IOError:
            print(IOError)
            print("Shuting down the Biotac subscriber!")

#===============================================================================
# MAIN METHOD
#===============================================================================
if __name__ == '__main__':
    print("[Initializing Sawyer ROS node...]\n")
    rospy.init_node('Move_position', anonymous=True)
    # if not flag:
    #     main()
    #     flag = True
    listener()
