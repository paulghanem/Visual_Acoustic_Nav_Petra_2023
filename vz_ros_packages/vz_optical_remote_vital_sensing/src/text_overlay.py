#! /usr/bin/python3

# ROS imports
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

# Python imports
from cv_bridge import CvBridge
import cv2

# Custom imports
from vz_optical_remote_vital_sensing.msg import VZ_BpmData

class Overlay:
    def __init__(self):

        self.bridge = CvBridge()

        # Intialize empty stores image and text
        self.text_rr = "calculating..."
        self.text_hr = "calculating..."
        self.img = np.zeros((480,640,3), np.uint8)
        
        self.hr_sub = rospy.Subscriber("/orvs/heart_rate", VZ_BpmData, self.hr_cb, queue_size=10)
        self.img_sub = rospy.Subscriber("/D435i/image_raw_rot", Image, self.img_cb, queue_size=10)
        self.rr_sub = rospy.Subscriber("/orvs/resp_rate", VZ_BpmData, self.rr_cb, queue_size=10) 

        self.overlay_pub = rospy.Publisher("/ORVS/overlay_img_topic", Image)      

        # Text Parameters
        self.font                   = cv2.FONT_HERSHEY_SIMPLEX
        self.dist_from_left         = 10
        self.dist_from_top          = 1100
        # self.bottomLeftCornerOfText = (10,1050)
        self.fontScale              = 1.75
        self.fontColor              = (0,0,255)
        self.thickness              = 3
        self.lineType               = 2

        self.img_with_text = np.zeros((480,640,3), np.uint8)

        self.taken = False


    # Store each IR image into some data structure for signal processing, e.g. buffer
    def hr_cb(self, msg_data):
        cv2.destroyAllWindows()
        text_hr = "{:.2f}".format(msg_data.data)
        self.text_hr = text_hr + " beats per min"

    def rr_cb(self, msg_data):
        cv2.destroyAllWindows()
        text_rr = "{:.2f}".format(msg_data.data)
        self.text_rr = text_rr + " breathes per min"
        
    def img_cb(self, msg_data):
        self.img = self.bridge.imgmsg_to_cv2(msg_data, desired_encoding="passthrough") # uint8
        self.update_img()

    def update_img(self):
        self.img_with_text = cv2.putText(self.img, "Heart rate: ", # Need to find the units for this
                    (self.dist_from_left, self.dist_from_top),
                    self.font,
                    self.fontScale,
                    self.fontColor,
                    self.thickness,
                    self.lineType)
        
        self.img_with_text = cv2.putText(self.img, "{}".format(self.text_hr), # Need to find the units for this
                   (self.dist_from_left, self.dist_from_top + 50),
                    self.font,
                    self.fontScale,
                    self.fontColor,
                    self.thickness,
                    self.lineType)

        self.img_with_text = cv2.putText(self.img_with_text, "Respiratory Rate: ", # Need to find the units for this
                    (self.dist_from_left, self.dist_from_top + 120),
                    self.font,
                    self.fontScale,
                    (255,0,100),
                    self.thickness,
                    self.lineType)

        self.img_with_text = cv2.putText(self.img_with_text, "{}".format(self.text_rr), # Need to find the units for this
                    (self.dist_from_left, self.dist_from_top + 170),
                    self.font,
                    self.fontScale,
                    (255,0,100),
                    self.thickness,
                    self.lineType)
                    
        # Publish Image to be viewed in RVIZ
        img = self.bridge.cv2_to_imgmsg(self.img_with_text, encoding="rgb8")
        self.overlay_pub.publish(img)


if __name__== '__main__':
    try:
        rospy.init_node("text_overlay")
        rospy.Rate(10)
        processor = Overlay()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Overlay node failed!')