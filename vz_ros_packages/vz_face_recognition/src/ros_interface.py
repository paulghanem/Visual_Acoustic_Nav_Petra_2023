#!/usr/bin/python3

# ROS imports
from regex import B
from sympy import Point2D
import rospy
import numpy as np
from sensor_msgs.msg import Image


# Python imports
import sys
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
from time import perf_counter
from argparse import ArgumentParser
import pickle 

# Custom imports
from vz_face_recognition.msg import HSE_Object, HSE_Objects

path_to_modules = os.environ.get('PATH_TO_MODULES')
path_to_face_recognition=str(path_to_modules) + '/face_recognition'
path_to_common = str(path_to_face_recognition) + '/common/python'
sys.path.insert(0, path_to_common)
from openvino.model_zoo.model_api.models.utils import OutputTransform
from openvino.model_zoo.model_api.performance_metrics import PerformanceMetrics
import monitors

path_to_face_recognition_demo = str(path_to_face_recognition) + '/face_recognition_demo/python'
sys.path.insert(0, path_to_face_recognition_demo)
import face_recognition_exe as fre


class Face_Recognition:
    def __init__(self):

        self.bridge = CvBridge()

        self.registration = rospy.get_param("face_registration")
        self.registration_num_frames  = rospy.get_param("face_registration_num_frames")

        self.msg = HSE_Objects()

        self.frame_num = 0
        self.metrics = PerformanceMetrics()
        self.presenter = None
        self.output_transform = None
        self.input_crop = None

        self.initial_condition= fre.initialize(self.registration)
        
        self.frame_processor = fre.FrameProcessor(self.initial_condition)

        if self.initial_condition.crop_size[0] > 0 and self.initial_condition.crop_size[1] > 0:
            self.input_crop = np.array(self.initial_condition.crop_size)
        elif not (self.initial_condition.crop_size[0] == 0 and self.initial_condition.crop_size[1] == 0):
            raise ValueError('Both crop height and width should be positive')

        self.rgb_sub = rospy.Subscriber("/D435i/image_raw_rot", Image, self.img_cb, queue_size=1) 
        self.faces_pub = rospy.Publisher("/face/object", HSE_Objects, queue_size=1)

        self.face_ts = 0

        self.home_path = os.environ.get('HOME')
        self.reg_path = os.environ.get('VZ_REG_DIR')


    def img_cb(self, msg_data):
        face_ts = msg_data.header.stamp
        img = self.bridge.imgmsg_to_cv2(msg_data, desired_encoding="passthrough") # uint8
        self.run(img, face_ts)
        

    def run(self, img, ts):
        start_time = perf_counter()
        frame = img
        if frame is None:
            if self.frame_num == 0:
                raise ValueError("Can't read an image from the input")
        if self.input_crop != None:
            frame = fre.center_crop(frame, self.input_crop)
        if self.frame_num == 0:
            self.output_transform = OutputTransform(frame.shape[:2], self.initial_condition.output_resolution)
            if self.initial_condition.output_resolution:
                self.initial_condition.output_resolution = self.output_transform.new_resolution
            else:
                self.initial_condition.output_resolution = (frame.shape[1], frame.shape[0])
            # print(self.initial_condition.new_resolution)
            self.presenter = monitors.Presenter(self.initial_condition.utilization_monitors, 55,(round(self.initial_condition.output_resolution[0] / 4), round(self.initial_condition.output_resolution[1] / 8)))
            # self.presenter = monitors.Presenter(self.initial_condition.utilization_monitors, 55,(2, 2))

        detections = self.frame_processor.process(frame)

        self.msg.objects =  []
        # Publish rois and face identities
        if(detections[0]): # Check if empty
            for i in range(len(detections[0])):
                obj = HSE_Object()

                # get label
                obj.label = self.frame_processor.face_identifier.get_identity_label(detections[2][(i)].id)

                # get roi
                obj.tl.x = int(detections[0][i].position[0])
                obj.tl.y = int(detections[0][i].position[1])
                obj.width = int(detections[0][i].size[0])
                obj.height = int(detections[0][i].size[1])

                # Append the list of objects
                self.msg.objects.append(obj)

        self.presenter.drawGraphs(frame)
        frame = fre.draw_detections(frame, self.frame_processor, detections, self.output_transform)
        
        # Publish marked up image and list of HSE objects
        self.msg.image = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
        self.msg.header.frame_id = 'face_object'
        self.msg.header.stamp = ts  
        self.msg.image.header.stamp = ts
        self.faces_pub.publish(self.msg)

        self.metrics.update(start_time, frame)

        self.frame_num += 1

        faces_database_path = str(self.reg_path) + '/face_recognition_reg/face_database.pkl'

        if (self.frame_num >= self.registration_num_frames and self.registration == True):
            with open(faces_database_path, 'wb') as outp:
    
                pickle.dump(self.frame_processor.faces_database, outp, pickle.HIGHEST_PROTOCOL)
                print("Face registration complete! File created: {}".format(faces_database_path))
            
            self.registration = False
            rospy.set_param("face_registration", False)
            self.initial_condition = fre.initialize(self.registration)
            self.frame_processor = fre.FrameProcessor(self.initial_condition)
            self.frame_num = 0
            


if __name__ == '__main__':
    try:
        rospy.init_node('Face_Recognition', anonymous=True)
        processor = Face_Recognition()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Face Recognition node failed!')
        pass
    cv2.destroyAllWindows()
