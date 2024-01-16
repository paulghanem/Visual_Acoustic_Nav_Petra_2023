#! /usr/bin/python3

# Purpose of node : Subscribe : Realsense rotated image, thermal raw image, ir image and the bounding box in the real sense image 
# Purpose of node : Publish   : 3 HSEObjects msg with the bounding box for thermal and IR and rgb bb

# General Imports

from xml.etree.ElementTree import TreeBuilder
import os
from cv_bridge import CvBridge
import cv2

# Imports ROS Specific
import rospy
import message_filters
import numpy as np

# Custom Imports
from vz_face_recognition.msg import HSE_Objects, HSE_Object
from vz_optical_remote_vital_sensing.msg import VZ_BpmData
from sensor_msgs.msg import Image


class BoundingBoxesORVS:
    def __init__(self):

            #### Set up the publishers of all the bounding boxes
            self.thermal_bb_pub = rospy.Publisher('/orvs_thermal_bb',HSE_Object,queue_size=10)
            self.ir_bb_pub = rospy.Publisher('/orvs_ir_bb',HSE_Object,queue_size=10)
            self.rs_bb_pub = rospy.Publisher('/orvs_rs_bb',HSE_Object,queue_size=10)

            self.thermal_bb_img_pub = rospy.Publisher('/orvs_thermal_img_bb',Image,queue_size=10)
            self.ir_bb_img_pub = rospy.Publisher('/orvs_ir_img_bb',Image,queue_size=10)
            self.rs_bb_img_pub = rospy.Publisher('/orvs_rs_img_bb',Image,queue_size=10)
            #####

            ##### Set up the subscribers order is thermal, ir, rgb raw, rgb bb
            self.orvs_topics = []

            # Set up subscriber for Lepton Image Raw
            self.lepton_thermal_callback= message_filters.Subscriber("/Lepton/image_raw", Image)
            self.orvs_topics.append(self.lepton_thermal_callback)
            
            # Set up subscriber for Structure Core Infrared Image Raw
            self.structure_core_infrared_callback = message_filters.Subscriber("/sc/infrared_left/image", Image)
            self.orvs_topics.append(self.structure_core_infrared_callback)

            # Set up subscriber for RealSense Image Raw
            self.realsense_rgb_callback = message_filters.Subscriber("/D435i/image_raw_rot", Image)
            self.orvs_topics.append(self.realsense_rgb_callback)

            self.realsense_face_boundingbox_callback = message_filters.Subscriber("/face/object", HSE_Objects)
            self.orvs_topics.append(self.realsense_face_boundingbox_callback)

            # Time Sync
            self.orvs_ats = message_filters.ApproximateTimeSynchronizer(self.orvs_topics,queue_size=5,slop=10.0)

            # Synced callback
            self.orvs_ats.registerCallback(self.orvs_synced_topic_callback)
            #####

            # For conversions b/w ROS and OpenCV
            self.bridge = CvBridge()


    def RectangletoXY(self,rec_pos):
        XY_Coord = [[0,0], [0,0], [0,0], [0,0]]

        #top left
        XY_Coord[0][0] = rec_pos[1]
        XY_Coord[0][1] = rec_pos[0]

        #bottom left
        XY_Coord[1][0] = rec_pos[1] + rec_pos[3]
        XY_Coord[1][1] = rec_pos[0]

        #top right
        XY_Coord[2][0] = rec_pos[1]
        XY_Coord[2][1] = rec_pos[0] + rec_pos[2]

        #bottom right
        XY_Coord[3][0] = rec_pos[1]+rec_pos[3]
        XY_Coord[3][1] = rec_pos[0]+rec_pos[2]

        #returns 4x2 matrix
        return XY_Coord

    def RGBtoSR(self,RGBBoundingBox_matrix):
        
        RGB2scIR_constants = [[0.524054983,	1.081300813],
                              [0.888888889,	1.081300813],
                              [0.524054983,	1.221445221],
                              [0.888888889,	1.221445221]]

        result = np.around(np.multiply(RGBBoundingBox_matrix, RGB2scIR_constants)).astype(int)

        #returns 4x2 matrix
        return result

    def RGBtoThermal(self,RGBBoundingBox_matrix):

        RGB2thermal_constants = [[0.073883162,	0.162601626],
                                  [0.109803922,	0.162601626],
                                  [0.073883162,	0.181818182],
                                  [0.109803922,	0.181818182]]
        result = np.around(np.multiply(RGBBoundingBox_matrix, RGB2thermal_constants)).astype(int)
        
        #returns 4x2 matrix
        return result

    def XYtoRectangle(self,XY_Coord):
        #initialize list
        rec_pos = [0,0,0,0]

        rec_pos[0] = XY_Coord[0,1]
        rec_pos[1] = XY_Coord[0,0]
        rec_pos[2] = XY_Coord[2,1] - XY_Coord[0,1]
        rec_pos[3] = XY_Coord[3, 0] - XY_Coord[0, 0]

        #returns 1x4 matrix
        return rec_pos

    def subROI_Calcuation(self,bbox_xy, bbox_rec, w_s=0, w_e=100, h_s=10, h_e=70):
        w_s_pix = ((w_s / 100) * bbox_rec[2])
        w_e_pix = ((w_e / 100) * bbox_rec[2])
        h_s_pix = ((h_s / 100) * bbox_rec[3])
        h_e_pix = ((h_e / 100) * bbox_rec[3])

        subROI_xy = bbox_xy

        subROI_xy[0,1] = subROI_xy[0,1] + w_s_pix
        subROI_xy[1,1] = subROI_xy[1,1] + w_s_pix
        subROI_xy[2,1] = bbox_xy[0,1] + w_e_pix
        subROI_xy[3,1] = bbox_xy[0,1] + w_e_pix

        subROI_xy[0,0] = subROI_xy[0,0] + h_s_pix
        subROI_xy[1,0] = subROI_xy[1,0] + h_s_pix
        subROI_xy[2,0] = bbox_xy[0,0] + h_e_pix
        subROI_xy[3,0] = bbox_xy[0,0] + h_e_pix

        subROI = self.XYtoRectangle(subROI_xy)

        #returns 1x4 matrix
        return subROI
    
    def publish_hse_object(self,bounding_box):
        hse_obs_rgb        = HSE_Object()
        hse_obs_rgb.tl.x   = bounding_box[0]
        hse_obs_rgb.tl.y   = bounding_box[1]
        hse_obs_rgb.width  = bounding_box[2]
        hse_obs_rgb.height = bounding_box[3]
        return hse_obs_rgb

    def publish_images_wih_bounding_box(self,bounding_box, raw_cv_img,rgb = False):

        start_point = (int(bounding_box[0]), int(bounding_box[1]))
        end_point = (int(bounding_box[0]+bounding_box[2]) , int(bounding_box[1]+bounding_box[3]))

        

        cv_image_with_bb = cv2.rectangle(raw_cv_img, start_point, end_point,(0, 220, 0), 2)

        if rgb == False:
            image_message = self.bridge.cv2_to_imgmsg(cv_image_with_bb, encoding="passthrough")
        else:
            image_message = self.bridge.cv2_to_imgmsg(cv_image_with_bb, encoding="rgb8")

        return image_message

    # Function to set up orvs subscriber callback
    def orvs_synced_topic_callback(self, thermal_msg, sc_ir,  rs_rgb, face_objs):
        
        #Assume the person of interest is in the frame only
        if(len(face_objs.objects) == 0):
            return

        face_obj = face_objs.objects[0]
        
        # Step 1 : Grab the raw bounding box [topleft_x,topleft_y,width,height]
        output_face_bb_rgb_1_by_4 = [face_obj.tl.x, face_obj.tl.y, face_obj.width, face_obj.height] #Result bb RGB

        # Step 2 : Convert the realsense bounding box to 4*2 matrix
        face_bb_rgb_4_by_2 = self.RectangletoXY(output_face_bb_rgb_1_by_4)

        # Step 3 : For both the thermal and IR convert the 4*2 matrix RGB BB to 4*2 matrix thermal BB and 4*2 IR BB
        # 3a
        thermal_4_by_2 = self.RGBtoThermal(face_bb_rgb_4_by_2)
        # 3b
        sc_ir_4_by_2 = self.RGBtoSR(face_bb_rgb_4_by_2)

        # Step 4 : Convert the 4*2 matrices found in step 3 to 1*4 matrices (tl,tr,w,h) for thermal and iR
        # 4a
        thermal_bounding_box_1_by_4 = self.XYtoRectangle(thermal_4_by_2) #topleft_x,topleft_y,width,height
        # 4b
        output_sc_bb_1_by_4 = self.XYtoRectangle(sc_ir_4_by_2) #topleft_x,topleft_y,width,height # Result bb ir
        # output_sc_bb_1_by_4 = [output_sc_bb_1_by_4[1],output_sc_bb_1_by_4[0],output_sc_bb_1_by_4[3],output_sc_bb_1_by_4[2]]

        # Step 5 : Add an additonal thermal sub ROI 
        output_thermal_sub_roi_1_by_4 = self.subROI_Calcuation(thermal_4_by_2,thermal_bounding_box_1_by_4) # Result bb thermal (note the sub roi is used )

        ########### Start publishing the outputs
        # Publish the HSE Object
        self.rs_bb_pub.publish(self.publish_hse_object(output_face_bb_rgb_1_by_4))
        self.ir_bb_pub.publish(self.publish_hse_object(output_sc_bb_1_by_4))
        self.thermal_bb_pub.publish(self.publish_hse_object(output_thermal_sub_roi_1_by_4))

        # Publish the Image with bounding box
        cv_image_rgb_raw = self.bridge.imgmsg_to_cv2(rs_rgb, desired_encoding='passthrough')
        self.rs_bb_img_pub.publish(self.publish_images_wih_bounding_box(output_face_bb_rgb_1_by_4,cv_image_rgb_raw,True))

        # flip th sc ir and publish synced image
        cv_image_ir = self.bridge.imgmsg_to_cv2(sc_ir, desired_encoding='passthrough')
        cv_image_ir_rot_raw = cv2.rotate(cv_image_ir,cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.ir_bb_img_pub.publish(self.publish_images_wih_bounding_box(output_sc_bb_1_by_4,cv_image_ir_rot_raw))

        cv_image_thermal_raw = self.bridge.imgmsg_to_cv2(thermal_msg, desired_encoding='passthrough')
        self.thermal_bb_img_pub.publish(self.publish_images_wih_bounding_box(output_thermal_sub_roi_1_by_4,cv_image_thermal_raw))
         
if __name__ == "__main__":
    try:
        rospy.init_node("bb_orvs")
        bb_orvs = BoundingBoxesORVS()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Bounding Boxes for ORVS Failed!')
        rospy.on_shutdown(bb_orvs.cleanup)