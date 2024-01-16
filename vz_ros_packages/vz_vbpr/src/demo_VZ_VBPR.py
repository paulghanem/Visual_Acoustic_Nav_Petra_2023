#!/bin/env python3
# "exec" "`dirname $0`/env/bin/python" "$0" "$@"

import os
import cv2
import sys

sys.path.insert(1,str(os.environ.get('PATH_TO_MODULES')) + '/visual_body_pose_recognition')
sys.path.insert(1,str(os.environ.get('PATH_TO_MODULES')) + '/torch2trt')

from VZ_VBPR_TRT import ConnectToROS
import numpy as np
import time

import rospy
from cv_bridge import CvBridge, CvBridgeError
import message_filters

from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from vz_face_recognition.msg import  HSE_Object

class_nums = [15,8,18,30,19,9,41,6,24,14,43]
class_names = ['others', 'drink water', 'eat meal', 'brush teeth', 'brush hair', 'drop', 'pick up', 'throw', 'sit down', 'stand up', 'clapping', 'reading', 'writing', 'tear up paper', 'put on jacket', 'take off jacket', 'put on a shoe', 'take off a shoe', 'put on glasses', 'take off glasses', 'put on a hat/cap', 'take off a hat/cap', 'cheer up', 'hand waving', 'kicking', 'reach into pocket', 'hopping', 'jump up', 'phone call', 'play with phone/tablet', 'typing', 'point to something', 'taking a selfie', 'check time (from watch)', 'rub two hands', 'nod head/bow', 'shake head', 'wipe face', 'salute', 'put palms together', 'cross hands in front', 'sneeze/cough', 'staggering', 'falling down', 'headache', 'chest pain', 'back pain', 'neck pain', 'nausea/vomiting', 'fan self', 'punch/slap', 'kicking', 'pushing', 'pat on back', 'point finger', 'hugging', 'giving object', 'touch pocket', 'shaking hands', 'walking towards', 'walking apart', 'not enough frames']

class Vbpr_wrapper():
    def __init__(self, demo):
        rospy.init_node('vz_vbpr_listener', anonymous=True)
        # uncomment for detectron
        # rospy.Subscriber("/D435i/image_raw_rot", Image, self.input_detectron_callback, queue_size=10)

        # comment for detectron
        # rospy.Subscriber("/D435i/image_raw_rot", Image, self.image_callback, queue_size=1) #window size 15
        # rospy.Subscriber("/HSE/person_object", HSE_Object, self.bbox_callback, queue_size=1) # window size 10
        # self.output_pub = rospy.Publisher("/vz_vbpr/action", Int32)

        # Set up ROS Publishers
        self.pose_pub = rospy.Publisher("/vz_vbpr/pose", Image)
        self.output_pub = rospy.Publisher("/vz_vbpr/action", String)

        # Set up ROS Subscribers to sync up
        self.raw_img_sub= message_filters.Subscriber("D435i/image_raw_rot",Image)
        self.hse_person_obj_sub= message_filters.Subscriber("/ip_vbpr_pub",HSE_Object)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.raw_img_sub,self.hse_person_obj_sub], queue_size=10,slop=1)
        self.synchronizer.registerCallback(self.convert_callback)
      
        self.demo = demo
        self.bridge = CvBridge()
        self.cv_image = None
        self.bbox = None
        self.video_pred_uvd_jts = None
        self.loop = 1

    # # For detectron
    # def input_detectron_callback(self, data):
    #     # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.encoding)        
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, "8UC3")
    #         bbox = self.demo.Detectron2_tracking(cv_image)
    #         video_pred_uvd_jts = self.demo.HybrIK_3dpose(cv_image, bbox)
    #         print(video_pred_uvd_jts.shape)
    #         label_50_before = self.demo.MS_G3D_actions(video_pred_uvd_jts)
    #         print(label_50_before)
    #         self.output_pub.publish(int(label_50_before))
    #     except CvBridgeError as e:
    #         print(e)

    # def image_callback(self, data):
    #     # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.encoding)        
    #     try:
    #         self.cv_image = self.bridge.imgmsg_to_cv2(data, "8UC3")
    #     except CvBridgeError as e:
    #         print(e)

    # def bbox_callback(self, datas):
    #     data = None
    #     for object in datas.objects:
    #         data = object

    #     # rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.cv_image)
    #     if self.cv_image is not None and data is not None:
    #         tl = data.tl
    #         bbox = np.array([[tl.x, tl.y, tl.x + data.width, tl.y + data.height]])
    #         video_pred_uvd_jts, bbox_img = self.demo.HybrIK_3dpose(self.cv_image, bbox)
    #         # print(video_pred_uvd_jts.shape)
    #         label_50_before = self.demo.MS_G3D_actions(video_pred_uvd_jts)
    #         action_str = class_names[int(label_50_before)]
    #         # print(label_50_before)
    #         # self.output_pub.publish(int(label_50_before))
    #         pose_msg = self.bridge.cv2_to_imgmsg(bbox_img)
    #         self.pose_pub.publish(pose_msg)
    #         self.output_pub.publish(action_str)

    def convert_callback(self,raw_img_data, person_ob_data):
        #print("syncedcbinsidevbpr")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_img_data, "8UC3")
        except CvBridgeError as e:
            print(e)

        bad_data = person_ob_data.tl.x == 0.0 and person_ob_data.tl.y == 0.0 and  person_ob_data.width == 0.0 and person_ob_data.height == 0.0
        if self.cv_image is not None and person_ob_data is not None and not bad_data:
            tl = person_ob_data.tl
            self.bbox = np.array([[tl.x, tl.y, tl.x + person_ob_data.width, tl.y + person_ob_data.height]])

        # Now that we have raw image and person bounding box synced up, lets predict
        self.predict()            

    def predict(self):
        if self.cv_image is not None and self.bbox is not None:
            start_hybrik=time.time()
            self.video_pred_uvd_jts, self.bbox_img = self.demo.HybrIK_3dpose(self.cv_image, self.bbox)
            # print("total time for hybrik: ",time.time()-start_hybrik)
            if self.loop >= 10:
                start_action=time.time()
                label_50_before = self.demo.MS_G3D_actions(self.video_pred_uvd_jts)
                action_str = class_names[int(label_50_before)]
                print("Predicted action:", action_str)
                self.output_pub.publish(action_str)
                # print("total time action: ",time.time()-start_action)
                self.loop=0

            pose_msg = self.bridge.cv2_to_imgmsg(self.bbox_img)
            self.pose_pub.publish(pose_msg)
            self.loop = self.loop + 1

    def spin(self):
        rospy.spin()

        # rate = rospy.Rate(1) # 1hz
        # while not rospy.is_shutdown():
        #     start_time = time.time()
        #     self.predict()
        #     print("total time: ",time.time()-start_time)
        #     rate.sleep()


def main():
    print(rospy.get_published_topics())
    demo = ConnectToROS()
    vbpr_wrapper = Vbpr_wrapper(demo)
    vbpr_wrapper.spin()


if __name__ == '__main__':
    main()
