#! /usr/bin/python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
import sys
import cv2

# Python imports
from cv_bridge import CvBridge
import os

# Custom imports
modules_path = os.environ.get('PATH_TO_MODULES')
NIR_path = str(modules_path) + '/optical_remote_vital_sensing/NIR_Video_Submodule/Classes_and_Functions'
sys.path.insert(0, NIR_path)
from ORVS_Signal_Processing import ORVS_Signal_Processing as OSP
from vz_optical_remote_vital_sensing.msg import VZ_BpmData
from vz_face_recognition.msg import HSE_Object


class Struct_Core:
    def __init__(self):
        # Intialize empty stores for heart rate or breathes per min
        self.bpm = None

        # Initialize classes
        self.signal_processing = OSP()

        self.bridge = CvBridge()

        self.fps = 10
        self.seconds = rospy.get_param("rec_secs")

        self.num_frames = self.fps* self.seconds
        self.buffer=100
        
        self.y_dim = 912
        self.x_dim = 1216  

        self.frame_stack = np.empty((self.x_dim, self.y_dim, self.num_frames)) #[None]*(self.num_frames)
        self.time_stack = [None]*(self.num_frames)
        self.ROI = [260, 678, 388, 256] # Hardcoded ROI
        
        self.index = 0
        self.t_zero = 0
         
        self.num_cycles = 10 # how many points do we want for testing... (WHAT DO THE POINTS MEAN?? -> for point cloud?)                               
        
        self.thermal_sub = rospy.Subscriber("/sc/infrared_left/image", Image, self.ir_cb, queue_size=100)
        self.roi_pub = rospy.Subscriber("/orvs_ir_bb", HSE_Object, self.roi_cb, queue_size=10)
        self.hr_pub = rospy.Publisher("/orvs/heart_rate", VZ_BpmData, queue_size=10)


        self.b = self.signal_processing.FIR1_Design() # Filter Design

        self.msg = VZ_BpmData()


    # Store each IR image into some data structure for signal processing, e.g. buffer
    def ir_cb(self, msg_data):
        img_msg = self.bridge.imgmsg_to_cv2(msg_data, desired_encoding="passthrough") # array of array of uint16
        rot_img_msg = cv2.rotate(img_msg,cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.frame_stack[:, :, self.index] = rot_img_msg
        self.time_stack[self.index] = self.t_zero
        self.index += 1
        self.t_zero += 0.03333
        if (self.index == self.num_frames):
            # Send to Rahul's function
            frame_list = self.frame_stack
            time_list = self.time_stack
            self.time_stack = self.time_stack[:(self.num_frames-self.buffer)]+ [None]*(self.buffer)
            self.index = self.num_frames - self.buffer # reset index
            self.loop(frame_list, time_list)


    def roi_cb(self, msg_data):
        roi = [msg_data.tl.x, msg_data.tl.y, msg_data.width, msg_data.height]
        self.ROI = roi
        self.ROI = np.around(self.ROI).astype(int)
        
        
    def loop(self, frame_stack, time_stack):
        faces_rects = self.signal_processing.duplicate_Rectangle_ROI(self.ROI, self.num_frames)
        cropped_img_dict = self.signal_processing.Crop_Face(faces_rects,frame_stack)
        [masked_img_dict, Avg_img_vec] = self.signal_processing.Mask_Face(cropped_img_dict)
        [Splined_time,Splined_img_vec] = self.signal_processing.Spline_Signal(time_stack,Avg_img_vec,self.signal_processing.fs)

        [imf_1,imf_2,imf,EMD_recon_vec] = self.signal_processing.EMD_Decomposition(Splined_img_vec,self.signal_processing.EMD_comp)
        Filtered_signal = self.signal_processing.filter_data(EMD_recon_vec,self.b)
        [fourier_freqs,PSD,max_PSD_idx,self.msg.data] = self.signal_processing.fourier_representation(self.signal_processing.fs,Splined_time,Filtered_signal)
        # print("Heart Rate: ", self.msg.data)
        self.msg.header.stamp = rospy.Time.now()
        self.hr_pub.publish(self.msg)

            

if __name__== '__main__':
    try:
        rospy.init_node("occ_capture")
        processor = Struct_Core()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('BPM extraction node failed!')