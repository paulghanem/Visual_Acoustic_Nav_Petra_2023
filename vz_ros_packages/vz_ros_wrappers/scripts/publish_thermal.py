#! /usr/bin/python3
'''
Purpose: Capture images from FLIR Lepton dev kit and republish
'''

# Future proofing python 2
from __future__ import nested_scopes
from __future__ import generators
from __future__ import division
from __future__ import absolute_import
from __future__ import with_statement
from __future__ import print_function

# Standard package imports
import rospy
import tf2_ros
import sys
import os
import cv2
import roslib
import math
import traceback
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

UPDATE_RATE = 111111111.111 # nanoseconds

class FLIR_LEPTON:
    def __init__(self):

        # Intialize empty stores for current image
        self.thermal_image = None

        # the following depends on usb port we plug into
        self.thermal_cap = cv2.VideoCapture('/dev/video6')

        # CV Bridge Instance
        self.bridge = CvBridge()

        # ROS Publishers
        self.thermal_pub = rospy.Publisher("/Lepton/image_raw", Image, queue_size=100)

        # ROS TImer to periodically publish thermal images
        self.timer = rospy.Timer(rospy.Duration(0, UPDATE_RATE), self.publish_image)

        # Keep reading thermal camera inputs
        while not rospy.is_shutdown():
            ret, self.thermal_image = self.thermal_cap.read()


    def publish_image(self, timer):
        try:
            if self.thermal_image is None:
                return
                
            img_msg = self.bridge.cv2_to_imgmsg(self.thermal_image, encoding="passthrough")
            img_msg.header.frame_id = 'flir_3_5_near_realsense'
            img_msg.header.stamp = rospy.Time.now()
            self.thermal_pub.publish(img_msg)
            
        except Exception as e:
            print(traceback.print_exc())

if __name__ == '__main__':
    try:
        rospy.init_node('FLIRLepton', anonymous=True)
        processor = FLIR_LEPTON()
    except rospy.ROSInterruptException:
        print('Thermal Image processing node failed!')
    cv2.destroyAllWindows()