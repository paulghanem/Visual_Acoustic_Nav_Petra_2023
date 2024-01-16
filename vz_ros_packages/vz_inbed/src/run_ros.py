#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import sys

import os
path_to_modules = os.environ.get('PATH_TO_MODULES')
path_to_ined = str(path_to_modules) + '/in_bed_pose_estimation'
path_to_torch2trt = str(path_to_modules) + '/torch2trt'
sys.path.insert(1,path_to_ined)

from data.SLP_RD import SLP_RD
from data.SLP_FD import SLP_FD
from data.SLP_RD_realtime import SLP_RD_realtime
from data.SLP_FD_realtime import SLP_FD_realtime
import utils.vis as vis
import utils.utils as ut
import numpy as np
import opt
import torch
import json
from os import path
import os
import rospy

from utils.logger import Colorlogger
from utils.utils_tch import get_model_summary
from core.loss import JointsMSELoss
from torch.utils.data import DataLoader
from torch.optim import Adam
import time
from utils.utils_ds import accuracy, flip_back
from utils.visualizer import Visualizer

# CPU vs GPU uploads
inbed_computer = rospy.get_param("/inbed_computer")
if(inbed_computer == "cpu"):
    from modeltest_ros import ConnectToROS # without CUDA calls
    print("CPU")
else:
    #from modeltest_ros_cuda_v2 import ConnectToROS # without TensorRT calls but improvised version
    from convtrt import ConnectToROS # with TensorRT calls    
    print("GPU")


#from modeltest_ros_cuda import ConnectToROS # without TensorRT calls

import matplotlib.image as mping

import tifffile as tiff


import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

# def preprocess_tiff(img):
#     # img = tiff.imread('/home/mehrshad/catkin_ws/src/vz_inbed/src/datasets/Uncovered_pose-2_2.tiff')
#     img = img.astype(np.float64)
#     img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
#     im_min, im_max = img.min(), img.max()
#     img -= im_min
#     img /= (im_max - im_min)
#     img *= 255
#     img = img.astype(np.uint8)
#     return img

class Inbed_wrapper():
    def __init__(self, model):
        rospy.init_node('vz_inbed_listener', anonymous=True)
        
        rospy.Subscriber("/Lepton/image_raw", Image, self.input_callback, queue_size=1)
        # self.output_pub = rospy.Publisher("/vz_inbed/pose", Float32MultiArray)
        self.vis_pub = rospy.Publisher("/vz_inbed/vis", Image, queue_size=1)

        self.model = model
        self.bridge = CvBridge()
        self.img_msg = None

    def input_callback(self, data):
        # print("Inbed xavier")
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.encoding)  
        self.img_msg = data      
        s=time.time()
        self.predict()
        # print("time for one image: ",time.time()- s)

    def spin(self):
        rospy.spin()
        # rate = rospy.Rate(1) # 1hz
        # while not rospy.is_shutdown():
           
        #     rate.sleep()

    def predict(self):
        if self.img_msg == None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.img_msg, "8UC3")
            img_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            img_gray = cv2.rotate(img_gray, cv2.ROTATE_90_COUNTERCLOCKWISE)
            img_gray = cv2.resize(img_gray, (120, 160), interpolation= cv2.INTER_LINEAR)

            # print(img_gray.shape)
            self.model.dataload(img_gray)
            rst_test = self.model.test()
            # print(rst_test)
            # new_jmessage.data = rst_test['pred_ori']
            # self.output_pub.publish(new_jmessage) # 14x2 array
            
            out_im = cv2.rotate(rst_test['out_im'], cv2.ROTATE_90_CLOCKWISE)
            out_im = self.bridge.cv2_to_imgmsg(out_im)
            self.vis_pub.publish(out_im)

        except CvBridgeError as e:
            print(e)


def main():
    
    model1 = ConnectToROS()
    inbed_wrapper = Inbed_wrapper(model1)
    inbed_wrapper.spin()
    # model1.saveresult(rst_test)
    

if __name__ == '__main__':
   
    main()
    
