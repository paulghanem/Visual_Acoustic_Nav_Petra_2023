#! /usr/bin/python3

# ROS imports
import rospy
from sensor_msgs.msg import Image


# Python imports
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
import os

# Custom imports
from vz_face_recognition.msg import HSE_Object
from vz_optical_remote_vital_sensing.msg import VZ_BpmData
modules_path = os.environ.get('PATH_TO_MODULES')
thermal_path = str(modules_path) + '/optical_remote_vital_sensing/Thermal_Video_Submodule'
sys.path.insert(0, thermal_path)
from Thermal_Camera_PostProcessing_Demo import AVI


class FLIR_LEPTON:
    def __init__(self):
        # Intialize empty stores for heart rate or breathes per min
        self.bpm = None

        self.bridge = CvBridge()
        self.video_writer = None

        self.fps = 9
        self.seconds = rospy.get_param("rec_secs")
        self.num_frames = self.fps * self.seconds
        self.frame_arr = [None]*(self.num_frames)
        self.time_arr = [None]*(self.num_frames)
        
        self.index = 0
        self.t_zero = 0

        self.roi_sub = rospy.Subscriber("/orvs_thermal_bb", HSE_Object, self.roi_cb, queue_size=10)

        self.thermal_sub = rospy.Subscriber("/Lepton/image_raw", Image, self.thermal_cb)
        self.bpm_pub = rospy.Publisher("/orvs/resp_rate", VZ_BpmData, queue_size=10)

        self.ROI = [39,20,23,34]

        self.msg = VZ_BpmData()

    def thermal_cb(self, msg_data):
        cv_image = self.bridge.imgmsg_to_cv2(msg_data, desired_encoding='passthrough')
        t = msg_data.header.stamp
        if self.index == 0:
            self.t_zero = t.to_sec()
        if self.index < self.num_frames:
            self.frame_arr[self.index] = cv_image
            deltat = t.to_sec() - self.t_zero
            self.time_arr[self.index] = deltat
            self.index += 1
        else: # when list is full
            self.index = 0 # reset index
            vid_arr = np.asarray(self.frame_arr) # make into np array 
            self.ROI = np.around(self.ROI).astype(int)
            AVIObj = AVI()
            self.msg.data = AVIObj.get_bpm(self.ROI,vid_arr,self.time_arr) # perform bpm measure
            self.msg.header.stamp = rospy.Time.now()
            self.bpm_pub.publish(self.msg)


    def roi_cb(self, msg_data):
        roi = [msg_data.tl.x, msg_data.tl.y, msg_data.width, msg_data.height]
        self.ROI = roi 

    def cleanup(self):
        if self.video_writer is not None:
            self.video_writer.release()


if __name__== '__main__':
    try:
        rospy.init_node("bpm_capture")
        processor = FLIR_LEPTON()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('BPM extraction node failed!')
        rospy.on_shutdown(processor.cleanup)