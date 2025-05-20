#!/usr/bin/env python3

# -*- coding: utf-8 -*-
"""
:ABSTRACT:
This script is part of the enhanced grasp project

:REQUIRES:

:
:AUTHOR:  Pedro Machado
:ORGANIZATION: Nottingham Trent University
:CONTACT: pedro.machado@ntu.ac.uk
:SINCE: 09/05/2025
:VERSION: 0.3

email:  pedro.machado@ntu.ac.uk
website: https://www.ntu.ac.uk/research/groups-and-centres/groups/computational-neuroscience-and-cognitive-robotics-laboratory
"""

# ===============================================================================
# PROGRAM METADATA
# ===============================================================================
__author__ = 'Pedro Machado'
__email__ = 'pedro.machado@ntu.ac.uk'
__copyright__ = 'GPLv3'
__license__ = 'GPLv3'
__date__ = '09/05/2025'
__version__ = '2.0'
__file_name__ = 'visualise_biotac.py'
__description__ = 'Subscribe the Biotac sensors raw data and display the data per sensor'
__compatibility__ = "Python 3"
__platforms__ = "i386, x86_64, arm32 and arm64"
__diff__= "GPLv3 , new launch file and publication in 3 topics"

#===============================================================================
# IMPORT STATEMENTS
#===============================================================================
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import cv2
import os
from std_msgs.msg import Float32MultiArray

#===============================================================================
# GLOBAL VARIABLES DECLARATIONS
#===============================================================================
PATH = os.path.dirname(os.path.realpath(__file__))
P = 0.98
visualisationFlag = True
FINGERTIPS = ["Thumb", "Index", "Ring"]

#===============================================================================
# MAIN NODE CLASS
#===============================================================================
class BiotacVisualizer(Node):
    def __init__(self):
        super().__init__('visualise_biotac')
        self.get_logger().info("[Initialising biotac visualisation...]")

        # Publishers for each sensor
        self.pub0 = self.create_publisher(Float32MultiArray, 'sensors/biotac/sensor_0', 10)
        self.pub1 = self.create_publisher(Float32MultiArray, 'sensors/biotac/sensor_1', 10)
        self.pub2 = self.create_publisher(Float32MultiArray, 'sensors/biotac/sensor_2', 10)

        # Subscriber for Biotac data
        self.subscription = self.create_subscription(
            String,
            '/biotac_sp_ros2',
            lambda msg: self.callback_biotac(msg, [self.pub0, self.pub1, self.pub2]),
            10)

        self.flag = True
        self.prev_mat = None
        self.fsr = 0

        self.get_logger().info("Sensor 0 published in topic: /sensors/biotac/sensor_0.")
        self.get_logger().info("Sensor 1 published in topic: /sensors/biotac/sensor_1.")
        self.get_logger().info("Sensor 2 published in topic: /sensors/biotac/sensor_2.")

    def callback_biotac(self, data, publishers):
        buffer = data.data
        buffer = buffer.split(',')
        mat = np.asarray(buffer)

        fields_name = ["E1", "PAC", "E2", "PAC", "E3", "PAC", "E4", "PAC", "E5", "PAC", "E6", "PAC", "E7", "PAC", "E8", \
                     "PAC", "E9", "PAC", "E10", "PAC", "E11", "PAC", "E12", "PAC", "E13", "PAC", "E14", "PAC", "E15", \
                     "PAC", "E16", "PAC", "E17", "PAC", "E18", "PAC", "E19", "PAC", "E20", "PAC", "E21", "PAC", "E22", \
                     "PAC", "E23", "PAC", "E24", "PAC", "PDC", "PAC", "TAC", "PAC", "TDC", "PAC"]

        vis_mat = []

        if self.flag:
            self.prev_mat = mat[1:len(mat)]
            self.flag = False
        else:
            for i in range(1, len(mat)):
                self.prev_mat[i-1] = int((1-P)*float(mat[i]) + P*float(self.prev_mat[i-1]))
            for i in range(1, len(mat)):
                mat[i] = np.abs(int(mat[i]) - int(self.prev_mat[i - 1]))

        for sensor in range(0, 3):
            pac = 0
            for i in range(2, len(fields_name)+1, 2):
                pac += int(mat[i + sensor * len(fields_name)])
            pac = int(pac / int(len(fields_name)/2))

            sensor_mat = np.asarray([
                [0, int(mat[21 + sensor * len(fields_name)]), 0, 0, 0, int(mat[1 + sensor * len(fields_name)]), 0],
                [0, 0, int(mat[23 + sensor * len(fields_name)]), 0, int(mat[3 + sensor * len(fields_name)]), 0, 0],
                [int(mat[51 + sensor * len(fields_name)]), 0, 0, int(mat[41 + sensor * len(fields_name)]), 0, 0, pac],
                [0, 0, int(mat[45 + sensor * len(fields_name)]), 0, int(mat[43 + sensor * len(fields_name)]), 0, 0],
                [int(mat[25 + sensor * len(fields_name)]), int(mat[27 + sensor * len(fields_name)]), 0, int(mat[47 + sensor * len(fields_name)]), 0, int(mat[7 + sensor * len(fields_name)]), int(mat[5 + sensor * len(fields_name)])],
                [0, 0, int(mat[29 + sensor * len(fields_name)]), 0, int(mat[9 + sensor * len(fields_name)]), 0, 0],
                [int(mat[31 + sensor * len(fields_name)]), 0, 0, 0, 0, 0, int(mat[11 + sensor * len(fields_name)])],
                [0, int(mat[33 + sensor * len(fields_name)]), 0, 0, 0, int(mat[13 + sensor * len(fields_name)]), 0],
                [int(mat[53 + sensor * len(fields_name)]), 0, int(mat[35 + sensor * len(fields_name)]), 0, int(mat[15 + sensor * len(fields_name)]), 0, int(mat[49 + sensor * len(fields_name)])],
                [0, int(mat[37 + sensor * len(fields_name)]), 0, 0, 0, int(mat[17 + sensor * len(fields_name)]), 0],
                [int(mat[39 + sensor * len(fields_name)]), 0, 0, 0, 0, 0, int(mat[19 + sensor * len(fields_name)])]
            ])

            vis_mat.append(sensor_mat)

            # Publish the data
            msg = Float32MultiArray()
            msg.data = sensor_mat.flatten('F').tolist()
            publishers[sensor].publish(msg)

        # Visualization
        for sensor in range(0, 3):
            aux = np.array(vis_mat[sensor], dtype=np.uint8)
            if visualisationFlag:
                scale_percent = 4000  # percent of original size
                width = int(aux.shape[1] * scale_percent / 100)
                height = int(aux.shape[0] * scale_percent / 100)
                dim = (width, height)
                # resize image
                aux = cv2.resize(aux, dim, interpolation=cv2.INTER_AREA)
                im_color = (cv2.applyColorMap(aux, cv2.COLORMAP_HOT))
                cv2.imshow(FINGERTIPS[sensor], im_color)
                cv2.namedWindow(FINGERTIPS[sensor],cv2.WND_PROP_FULLSCREEN)
                cv2.setWindowProperty(FINGERTIPS[sensor], cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        if visualisationFlag and cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down the Biotac visualizer!")
            cv2.destroyAllWindows()
            rclpy.shutdown()

#===============================================================================
# MAIN METHOD
#===============================================================================
def main(args=None):
    rclpy.init(args=args)
    biotac_visualizer = BiotacVisualizer()

    try:
        rclpy.spin(biotac_visualizer)
    except KeyboardInterrupt:
        biotac_visualizer.get_logger().info("Shutting down the Biotac visualizer!")
    except Exception as e:
        biotac_visualizer.get_logger().error(f"Error: {str(e)}")
    finally:
        biotac_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
