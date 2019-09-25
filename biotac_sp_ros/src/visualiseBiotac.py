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

2019 (c) GPLv3, Nottingham Trent University
Computational Neuroscience and Cognitive Robotics Laboratory
email:  pedro.baptistamachado@ntu.ac.uk
website: https://www.ntu.ac.uk/research/groups-and-centres/groups/computational-neuroscience-and-cognitive-robotics-laboratory


"""
# ===============================================================================
# PROGRAM METADATA
# ===============================================================================
__author__ = 'Pedro Machado'
__contact__ = 'pedro.baptistamachado@ntu.ac.uk'
__copyright__ = '2019 (C) GPLv3, CNCR@NTU, Prof. Martin McGinnity martin.mcginnity@ntu.ac.uk'
__license__ = 'GPLv3'
__date__ = '12/07/2019'
__version__ = '1.0'
__file_name__ = 'visualiseBiotac.py'
__description__ = 'Subscribe the Biotac sensors raw data and display the data per sensor'
__compatibility__ = "Python 2 and Python 3"
__platforms__ = "i386, x86_64, arm32 and arm64"
__diff__= "GPLv3 , new lauch file and publication in 3 topics"

#===============================================================================
# IMPORT STATEMENTS
#===============================================================================
import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import os
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

#===============================================================================
# GLOBAL VARIABLES DECLARATIONS
#===============================================================================
PATH=os.path.dirname(os.path.realpath(__file__))
P=0.98
visualisationFlag = True# False #
global fsr
#===============================================================================
# METHODS
#===============================================================================
def callback_biotac(data,publishers):
    global flag, prev_mat, fsr
    buffer = data.data
    buffer = buffer.split(',')
    mat = np.asarray(buffer)

    mat = np.asarray(mat)
    fields_name = ["E1", "PAC", "E2", "PAC", "E3", "PAC", "E4", "PAC", "E5", "PAC", "E6", "PAC", "E7", "PAC", "E8", \
                   "PAC", "E9", "PAC", "E10", "PAC", "E11", "PAC", "E12", "PAC", "E13", "PAC", "E14", "PAC", "E15", \
                   "PAC", "E16", "PAC", "E17", "PAC", "E18", "PAC", "E19", "PAC", "E20", "PAC", "E21", "PAC", "E22", \
                   "PAC", "E23", "PAC", "E24", "PAC", "PDC", "PAC", "TAC", "PAC", "TDC", "PAC"]

    # Dictionary to adress the respective sensors
    fn = {
        "E1": 1, "PAC": 2, "E2": 3, "E3": 5, "E4": 7, "E5": 9, "E6": 11, "E7": 13, "E8": 15, \
        "E9": 17, "E10": 19, "E11": 21, "E12": 23, "E13": 25, "E14": 27, "E15": 29, \
        "E16": 31, "E17": 33, "E18": 35, "E19": 37, "E20": 39, "E21": 41, "E22": 43, \
        "E23": 45, "E24": 47, "PDC": 49, "TAC": 51, "TDC": 53
    }

    vis_mat=[]
    if flag:
        prev_mat=mat[1:len(mat)]
        flag=False
    else:
        for i in range(1, len(mat)):
            prev_mat[i-1]=int((1-P)*float(mat[i])+P*float(prev_mat[i-1]))
        for i in range(1, len(mat)):
            mat[i] = np.abs(int(mat[i])-int(prev_mat[i - 1]))
    for sensor in range(0, 3):
        pac=0
        for i in range(2,len(fields_name)+1,2):
            pac+=int(mat[i+ sensor * len(fields_name)])
        pac=int(pac/int(len(fields_name)))
        vis_mat.append(np.asarray([[0,int(mat[fn["E11"]+ sensor * len(fields_name)]),0,0,0,int(mat[fn["E1"]+ sensor * len(fields_name)]),0],
                            [0,0,int(mat[fn["E12"]+ sensor * len(fields_name)]),0,int(mat[fn["E2"]+ sensor * len(fields_name)]),0,0],
                            [int(mat[fn["TAC"]+ sensor * len(fields_name)]),0,0,int(mat[fn["E21"]+ sensor * len(fields_name)]),0,0,pac],
                            [0,0,int(mat[fn["E23"]+ sensor * len(fields_name)]),0, int(mat[fn["E22"]+ sensor * len(fields_name)]),0,0],
                            [int(mat[fn["E13"]+ sensor * len(fields_name)]),int(mat[fn["E14"]+ sensor * len(fields_name)]),0,int(mat[fn["E24"]+ sensor * len(fields_name)]),0,int(mat[fn["E4"]+ sensor * len(fields_name)]),int(mat[fn["E3"]+ sensor * len(fields_name)])],
                            [0,0,int(mat[fn["E15"]+ sensor * len(fields_name)]),0,int(mat[fn["E5"]+ sensor * len(fields_name)]),0,0],
                            [int(mat[fn["E16"]+ sensor * len(fields_name)]),0,0,0,0,0,int(mat[fn["E6"]+ sensor * len(fields_name)])],
                            [0,int(mat[fn["E17"]+ sensor * len(fields_name)]),0,0,0,int(mat[fn["E7"]+ sensor * len(fields_name)]),0],
                            [int(mat[fn["TDC"]+ sensor * len(fields_name)]),0,int(mat[fn["E18"]+ sensor * len(fields_name)]),0,int(mat[fn["E8"]+ sensor * len(fields_name)]),0,int(mat[fn["PDC"]+ sensor * len(fields_name)])],
                            [0,int(mat[fn["E19"]+ sensor * len(fields_name)]),0,0,0,int(mat[fn["E9"]+ sensor * len(fields_name)]),0],
                            [int(mat[fn["E20"]+ sensor * len(fields_name)]),0,0,0,0,0,int(mat[fn["E10"]+ sensor * len(fields_name)])]]))

        publishers[sensor].publish(np.asarray(vis_mat[sensor], dtype=np.float32).flatten('F'))
    for sensor in range(0, 3):
        aux=np.array(vis_mat[sensor],dtype=np.uint8)
        if visualisationFlag:
            scale_percent = 4000  # percent of original size
            width = int(aux.shape[1] * scale_percent / 100)
            height = int(aux.shape[0] * scale_percent / 100)
            dim = (width, height)
            # resize image
            aux = cv2.resize(aux, dim, interpolation=cv2.INTER_AREA)
            im_color = (cv2.applyColorMap(aux, cv2.COLORMAP_HOT))
            cv2.imshow("Sensor " + str(sensor), im_color)

    if visualisationFlag and cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown('Quit')
        cv2.destroyAllWindows()


def listener():
    global flag
    while not rospy.is_shutdown():
        try:
            pub0 = rospy.Publisher('sensors/biotac/0', numpy_msg(Floats),queue_size=10)
            pub1 = rospy.Publisher('sensors/biotac/1', numpy_msg(Floats),queue_size=10)
            pub2 = rospy.Publisher('sensors/biotac/2', numpy_msg(Floats),queue_size=10)
            # Calls callback_biotec each time a new message on the topic arrives
            # Hands over the publishers (pub0,pub1,pub2) of this script to the callback function as well
            rospy.Subscriber("/biotac_sp_ros", String, callback_biotac, ([pub0, pub1, pub2]))
            print("Sensor 0 published in topic: /sensors/biotac/0.")
            print("Sensor 1 published in topic: /sensors/biotac/1.")
            print("Sensor 2 published in topic: /sensors/biotac/2.")
            flag=True
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down the Biotac subscriber!")
        except IOError:
            print(IOError)
            print("Shuting down the Biotac subscriber!")
#===============================================================================
#  TESTING AREA
#===============================================================================

#===============================================================================
# MAIN METHOD
#===============================================================================
if __name__ == '__main__':
    print("[Initialising biotac visualisation...]\n")
    rospy.init_node('visualise_biotac', anonymous=True)
    fsr=0
    # if not flag:
    #     main()
    #     flag = True
    listener()