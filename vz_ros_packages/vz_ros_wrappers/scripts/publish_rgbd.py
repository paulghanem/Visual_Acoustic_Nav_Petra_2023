#! /usr/bin/python3
'''
Purpose: Publish RGBD images with proper orientations
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
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class D435i:
    def __init__(self): 
        
        # Intialize empty stores for current image
        self.D435i_image = None

        # Instance of CV Bridge
        self.bridge = CvBridge()

        # ROS Publishers
        self.orig_rgb_pub = rospy.Publisher("/D435i/image_raw", Image, queue_size=1)
        self.rot_rgb_pub = rospy.Publisher("/D435i/image_raw_rot", Image, queue_size=1)
        self.rot_depth_pub = rospy.Publisher("/D435i/rot_depth_img", Image, queue_size=1)

        # ROS Subscribers
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,callback=self.publish_depth, queue_size=1)
        rospy.Subscriber("/camera/color/image_raw", Image,callback=self.publish_rgb, queue_size=1)


    def publish_depth(self,raw_depth_img):
        
        # Publish rotated depth image       
        cv_img_orig_depth = self.bridge.imgmsg_to_cv2(raw_depth_img , desired_encoding="passthrough")
        cv_img_rot_depth = cv2.rotate(cv_img_orig_depth,cv2.ROTATE_90_CLOCKWISE)
        depth_msg_rot = self.bridge.cv2_to_imgmsg(cv_img_rot_depth,encoding="passthrough")
        
        depth_msg_rot.header.frame_id = 'depth_rot'
        depth_msg_rot.header.stamp = raw_depth_img.header.stamp
        self.rot_depth_pub.publish(depth_msg_rot)

    def publish_rgb(self,raw_img_msg):

        # Publish the original rgb image
        cv_img_orig_rgb = self.bridge.imgmsg_to_cv2(raw_img_msg , desired_encoding="rgb8")
        rgb_msg = self.bridge.cv2_to_imgmsg(cv_img_orig_rgb,encoding="rgb8")
        self.orig_rgb_pub.publish(rgb_msg)
        
        # Publish the rotated rgb image
        cv_img_rot_rgb = cv2.rotate(cv_img_orig_rgb,cv2.ROTATE_90_CLOCKWISE)
        rgb_msg_rot = self.bridge.cv2_to_imgmsg(cv_img_rot_rgb,encoding="rgb8")
        rgb_msg_rot.header.frame_id = 'D435i_rot'
        rgb_msg_rot.header.stamp = raw_img_msg.header.stamp
        self.rot_rgb_pub.publish(rgb_msg_rot)        
        
if __name__ == '__main__':
    try:
        rospy.init_node('D435i', anonymous=True)
        processor = D435i()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('RGB and Depth processing node failed!')
        pass
    cv2.destroyAllWindows()
