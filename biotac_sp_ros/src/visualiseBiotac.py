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
__file_name__ = 'visualiseBiotac.py'
__description__ = 'Subscribe Biotac snesors'
__compatibility__ = "Python 2 and Python 3"
__platforms__ = "Sawyer and AR10 hand"

#===============================================================================
# IMPORT STATEMENTS
#===============================================================================
import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import os

#===============================================================================
# GLOBAL VARIABLES DECLARATIONS
#===============================================================================
PATH=os.path.dirname(os.path.realpath(__file__))
P=0.98
visualisationFlag = True# False #
#===============================================================================
# METHODS
#===============================================================================
def callback_biotac(data,pub):
    global flag, prev_mat, out
    buffer = data.data
    buffer = buffer.split(',')
    mat = np.asarray(buffer)

    mat = np.asarray(mat)
    fields_name = ["E1", "PAC", "E2", "PAC", "E3", "PAC", "E4", "PAC", "E5", "PAC", "E6", "PAC", "E7", "PAC", "E8", \
                   "PAC", "E9", "PAC", "E10", "PAC", "E11", "PAC", "E12", "PAC", "E13", "PAC", "E14", "PAC", "E15", \
                   "PAC", "E16", "PAC", "E17", "PAC", "E18", "PAC", "E19", "PAC", "E20", "PAC", "E21", "PAC", "E22", \
                   "PAC", "E23", "PAC", "E24", "PAC", "PDC", "PAC", "TAC", "PAC", "TDC", "PAC"]
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
        vis_mat.append(np.asarray([[0,int(mat[21+ sensor * len(fields_name)]),0,0,0,int(mat[1+ sensor * len(fields_name)]),0],
                            [0,0,int(mat[23+ sensor * len(fields_name)]),0,int(mat[3+ sensor * len(fields_name)]),0,0],
                            [int(mat[51+ sensor * len(fields_name)]),0,0,int(mat[41+ sensor * len(fields_name)]),0,0,pac],
                            [0,0,int(mat[45+ sensor * len(fields_name)]),0, int(mat[43+ sensor * len(fields_name)]),0,0],
                            [int(mat[25+ sensor * len(fields_name)]),int(mat[27+ sensor * len(fields_name)]),0,int(mat[47+ sensor * len(fields_name)]),0,int(mat[7+ sensor * len(fields_name)]),int(mat[5+ sensor * len(fields_name)])],
                            [0,0,int(mat[29+ sensor * len(fields_name)]),0,int(mat[9+ sensor * len(fields_name)]),0,0],
                            [int(mat[31+ sensor * len(fields_name)]),0,0,0,0,0,int(mat[11+ sensor * len(fields_name)])],
                            [0,int(mat[33+ sensor * len(fields_name)]),0,0,0,int(mat[13+ sensor * len(fields_name)]),0],
                            [int(mat[53+ sensor * len(fields_name)]),0,int(mat[35+ sensor * len(fields_name)]),0,int(mat[15+ sensor * len(fields_name)]),0,int(mat[49+ sensor * len(fields_name)])],
                            [0,int(mat[37+ sensor * len(fields_name)]),0,0,0,int(mat[17+ sensor * len(fields_name)]),0],
                            [int(mat[39+ sensor * len(fields_name)]),0,0,0,0,0,int(mat[19+ sensor * len(fields_name)])]]))

    #print("ROS time:", mat[0])
    message=[0,0,0]
    for sensor in range(0, 3):
        aux=np.array(vis_mat[sensor],dtype=np.uint8)
        ct=0
        aux1=aux.reshape((aux.shape[0]*aux.shape[1]))
        for i in range(0,len(aux1)):
            if aux1[i]>100:
                ct+=1
                if ct>2:
                    message[sensor]=1
                    break
        if visualisationFlag:
            scale_percent = 6000  # percent of original size
            width = int(aux.shape[1] * scale_percent / 100)
            height = int(aux.shape[0] * scale_percent / 100)
            dim = (width, height)
            # resize image
            aux = cv2.resize(aux, dim, interpolation=cv2.INTER_AREA)
            im_color=(cv2.applyColorMap(aux, cv2.COLORMAP_HOT))
            cv2.imshow("Sensor "+str(sensor), im_color)
    msg2pub=""
    for sensor in range(0, 3):
        msg2pub+=str(sensor)+","+str(message[sensor])
        if sensor<2:
            msg2pub+=","
    pub.publish(msg2pub)


    if visualisationFlag and cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown('Quit')
        cv2.destroyAllWindows()


def listener():
    global flag, out
    while not rospy.is_shutdown():
        try:
            pub = rospy.Publisher('biotac/contact', String, queue_size=1)
            rospy.Subscriber("/biotac_sp_ros", String, callback_biotac, (pub))
            print("Contact detection published in topic: biotac/contact.")
            flag=True
            rospy.spin()
        except rospy.ROSInterruptException:
            for i in range(0, 3):
                out[i].release()
            print("Shuting down the Biotac subscriber!")
        except IOError:
            for i in range(0, 3):
                out[i].release()
            print(IOError)
            print("Shuting down the Biotac subscriber!")
#===============================================================================
#  TESTING AREA
#===============================================================================

#===============================================================================
# MAIN METHOD
#===============================================================================
if __name__ == '__main__':
    print("[Initialising contact detector...]\n")
    rospy.init_node('Move_position', anonymous=True)
    # if not flag:
    #     main()
    #     flag = True
    listener()
