#!/usr/bin/env python3

import math
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge

import message_filters
import ros_numpy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from image_geometry.cameramodels import PinholeCameraModel

# Transforms
import tf2_ros
import tf2_geometry_msgs

from vz_face_recognition.msg import HSE_Object, HSE_Objects


class ConvertPos3Dtobbox2D(object):
    """
    ConvertPos3Dtobbox2D class takes 3D point mostly person center and converts that to person bounding box for use by VBPR
    """

    def __init__(self):
        self.cam_model = PinholeCameraModel()

        # Instance of CV Bridge
        self.bridge = CvBridge()
        # ROS subscriptions :
        # 1) all the pedestrians from ped tracker
        # 2) the person of interest; assuming it is in the PoseStamped msg type
        # 3) camera info
        # 4) rgb rotated

        pedestrians_sub = rospy.Subscriber(
            '/pedestrian/object', HSE_Objects, self.ped_callback)

        face_sub = message_filters.Subscriber(
            '/face/object', HSE_Objects)
        person_of_interest_3d_sub = message_filters.Subscriber(
            '/HSE/position_state', PoseStamped)
        camera_info_subscriber = message_filters.Subscriber(
            '/camera/color/camera_info', CameraInfo)
        rgb_image_subscriber_rotated = message_filters.Subscriber(
            '/D435i/image_raw_rot', Image)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [face_sub, person_of_interest_3d_sub, camera_info_subscriber, rgb_image_subscriber_rotated], queue_size=10, slop=10)
        self.synchronizer.registerCallback(self.convert)

        # ROS publishers :
        # 1) input for VBPR i.e. HSEObject representing the full bounding box representing the person of interest
        # 2) inverted_roi_for_display
        self.ip_vbpr_pub = rospy.Publisher(
            '/ip_vbpr_pub', HSE_Object, queue_size=1)
        self.inverted_roi_img_pub = rospy.Publisher(
            'HSE/inverted_roi_img', Image)

        # Transform initialisation
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.person_rois = None
        self.prev_final_roi = None
        self.roi_ratio = [1, 1]
        self.dist_thresh = 150
        self.dx = 0
        self.dy = 0

    def ped_callback(self, msg):
        rois = []
        for object in msg.objects:
            roi = [object.tl.x, object.tl.y, object.width, object.height]
            rois.append(roi)

        self.person_rois = rois

    def map_face_body(self, face_roi):
        person_roi = [0, 0, 0, 0]
        person_roi[0] = face_roi[0] - self.dx
        person_roi[1] = face_roi[1] - self.dy
        person_roi[2] = self.roi_ratio[0] * face_roi[2]
        person_roi[3] = self.roi_ratio[1] * face_roi[3]

        # Hacky re-adjustments of bboxes if latching onto pedestrian's last output
        if self.dx != 0:
            person_roi[0] *= 0.9
            person_roi[1] *= 0.9

        return person_roi

    def convert(self, faces, person_of_interest_3d, cam_info, rgb_img_rot):
        # Instance of camera model to be used later for converting a pixel to 3d point
        self.cam_model.fromCameraInfo(cam_info)

        # Convert RGB images to numpy arrays
        rgb_np = ros_numpy.numpify(rgb_img_rot)

        # Information about the raw rgb image
        orig_h, orig_w, _ = rgb_np.shape

        # Transform KF output into camera frame
        transform = self.tf_buffer.lookup_transform(
            'camera_color_optical_frame', person_of_interest_3d.header.frame_id,
            person_of_interest_3d.header.stamp, rospy.Duration(1))
        person_of_interest_3d_camera = tf2_geometry_msgs.do_transform_pose(
            person_of_interest_3d, transform).pose

        # Get the person of interest 3d coordinates
        xyz = person_of_interest_3d_camera.position

        # The project3dToPixel likes a tuple so give that
        person_3d = (xyz.x, xyz.y, xyz.z)

        # Get the person of interest center
        uv = self.cam_model.project3dToPixel(person_3d)
        # Adjust
        uv = (orig_w - 1 - uv[1], uv[0])

        # Find the associated pedestrian
        found_person = None
        current_min_dist = 999999999.999

        for fac in faces.objects:
            ctr_fac = [fac.tl.x + (fac.width/2), fac.tl.y + (fac.height/2)]
            ctr_hse = [uv[0], uv[1]]
            sep_fac_hse = math.dist(ctr_fac, ctr_hse) < current_min_dist
            if(sep_fac_hse < current_min_dist):
                current_min_dist = sep_fac_hse
                found_person = fac

        if found_person:
            tl_x = found_person.tl.x
            tl_y = found_person.tl.y
            inverted_bbox = [tl_x, tl_y,
                             found_person.width, found_person.height]

            if self.person_rois:
                dist = []
                dx_list = []
                dy_list = []
                for roi in self.person_rois:
                    dist.append(
                        math.sqrt((inverted_bbox[0] - roi[0])**2 + (inverted_bbox[1] - roi[1])**2))
                    [dx, dy] = [inverted_bbox[0] -
                                roi[0], inverted_bbox[1] - roi[1]]
                    dx_list.append(dx)
                    dy_list.append(dy)

                i = np.argmin(dist)
                if dist[i] < self.dist_thresh:
                    final_roi = self.person_rois[i]
                    self.roi_ratio = [
                        final_roi[2] / found_person.width, final_roi[3] / found_person.height]
                    self.dx = dx_list[i]
                    self.dy = dy_list[i]
                else:
                    final_roi = self.map_face_body(inverted_bbox)
            else:
                final_roi = self.map_face_body(inverted_bbox)

            # final_roi has tl_X, tl_y, br_x, br_y
            final_object = HSE_Object()
            final_object.header.frame_id = 'camera_color_optical_frame'
            final_object.header.stamp = person_of_interest_3d.header.stamp
            final_object.tl.x = int(final_roi[0])
            final_object.tl.y = int(final_roi[1])
            final_object.width = min(int(final_roi[2]), orig_w)
            final_object.height = min(int(final_roi[3]), orig_h)

            final_roi[2] += final_roi[0]
            final_roi[3] += final_roi[1]

            # Publish the final roi as an HSE Image for vbpr to consume
            if self.prev_final_roi and ((final_roi[2] - self.prev_final_roi[2] < self.dist_thresh)
                                        and (final_roi[3] - self.prev_final_roi[3] < self.dist_thresh)):
                self.draw_bbox_inverted_roi(
                    final_roi, rgb_np, color=(0, 255, 0))
                self.ip_vbpr_pub.publish(final_object)

            self.prev_final_roi = final_roi

    def draw_bbox_inverted_roi(self, roi, frame, color=(0, 0, 255)):
        roi_int = [int(c) for c in roi]
        x_min, y_min, x_max, y_max = roi_int
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
        self.inverted_roi_img_pub.publish(img_msg)


def convert_3d_to_2d():
    rospy.init_node('pos3d_to_pixel', anonymous=True)
    pos3d_to_pixel = ConvertPos3Dtobbox2D()
    rospy.spin()


if __name__ == '__main__':
    convert_3d_to_2d()
