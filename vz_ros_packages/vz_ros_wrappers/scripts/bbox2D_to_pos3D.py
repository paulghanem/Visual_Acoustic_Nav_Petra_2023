#!/usr/bin/env python3

'''
Class that converts all body bounding boxes and face bounding boxes 
from pixel coordinated to 3D coordinates
'''

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo
import rospy
from vz_face_recognition.msg import HSE_Objects
from vz_ros_wrappers.msg import Bounding_Box_3D, Bounding_Boxes_3D
import message_filters
import ros_numpy
import cv2
import sys
import os

path_to_modules = os.environ.get('PATH_TO_MODULES')
path_to_catkin_ws = os.environ.get('CATKIN_DIR')
sys.path.insert(1, str(path_to_catkin_ws)+'/src/stretch_ros')
sys.path.insert(1, str(path_to_modules)+'/face_recognition')

from stretch_deep_perception.nodes import detection_2d_to_3d as d2

class ConvertBBox2DtoPos3D:

    def __init__(self):
        self.detections_2d = None
        self.detections_3d = None

        self.fit_plane = False
        self.min_box_side_m = None
        self.max_box_side_m = None
        self.modify_3d_detections = None

        # ROS Publishers
        self.face_pub = rospy.Publisher(
            '/bb_cameraframe_face', Bounding_Boxes_3D, queue_size=1)
        self.person_pub = rospy.Publisher(
            '/bb_cameraframe_person', Bounding_Boxes_3D, queue_size=1)

        # ROS Subscribers (will be time synchronised)
        self.rgb_image_subscriber = message_filters.Subscriber(
            "/camera/color/image_raw", Image)
        self.depth_image_subscriber = message_filters.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image)
        self.camera_info_subscriber = message_filters.Subscriber(
            "/camera/color/camera_info", CameraInfo)
        self.face_sub = message_filters.Subscriber("/face/object", HSE_Objects)
        self.person_sub = message_filters.Subscriber(
            "/pedestrian/object", HSE_Objects)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_image_subscriber, self.depth_image_subscriber, self.camera_info_subscriber, self.face_sub, self.person_sub], queue_size=10, slop=10)
        self.synchronizer.registerCallback(self.convert_callback)

    def convert_callback(self, ros_rgb_image, ros_depth_image, rgb_camera_info, face_hse_objects, person_hse_objects):

        self.rgb_image = ros_numpy.numpify(ros_rgb_image)
        self.depth_image = ros_numpy.numpify(ros_depth_image)
        self.camera_info = rgb_camera_info

        # OpenCV expects bgr images, but numpify by default returns rgb images.
        self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)

        centers_xyz_face = self.convert_2d_to_3d(face_hse_objects.objects)
        centers_xyz_person = self.convert_2d_to_3d(person_hse_objects.objects)

        stamped_frames_face = self.create_stamped_frame(
            centers_xyz_face, face_hse_objects)
        stamped_frames_person = self.create_stamped_frame(
            centers_xyz_person, person_hse_objects)

        self.face_pub.publish(stamped_frames_face)
        self.person_pub.publish(stamped_frames_person)

    def create_stamped_frame(self, center_xyz_array, hse_objects):
        pose_array_with_labels = Bounding_Boxes_3D()
        pose_array_with_labels.header.stamp = hse_objects.header.stamp
        pose_array_with_labels.header.frame_id = "camera_color_optical_frame"

        for index in range(len(hse_objects.objects)):
            bounding_box_3d = Bounding_Box_3D()
            camera_frame_pose = Pose()
            center_xyz = center_xyz_array[index]
            camera_frame_pose.position.x = center_xyz[0]
            camera_frame_pose.position.y = center_xyz[1]
            camera_frame_pose.position.z = center_xyz[2]
            bounding_box_3d.pose = camera_frame_pose
            bounding_box_3d.label = hse_objects.objects[index].label
            pose_array_with_labels.bounding_boxes.append(bounding_box_3d)

        return pose_array_with_labels

    def convert_2d_to_3d(self, hse_objects):

        detections_2d_list = []
        centers_xyz = []
        for hse_object in hse_objects:
            # The bounding box 2d to 3d needs tl and br.
            tl_point_x = hse_object.tl.x
            tl_point_y = hse_object.tl.y
            orig_width = hse_object.width
            orig_height = hse_object.height

            br_point_x = tl_point_x + orig_width
            br_point_y = tl_point_y + orig_height
            hse_object_2d_dict = {'class_id': 0, 'label': 'person', 'confidence': 1.0, 'box': (
                tl_point_x, tl_point_y, br_point_x, br_point_y)}
            detections_2d_list.append(hse_object_2d_dict)
        self.detections_2d = detections_2d_list

        self.detections_3d = d2.detections_2d_to_3d(self.detections_2d, self.rgb_image, self.camera_info, self.depth_image,
                                                    fit_plane=self.fit_plane, min_box_side_m=self.min_box_side_m, max_box_side_m=self.max_box_side_m)
        for detection in self.detections_3d:
            centers_xyz.append(detection.get('box_3d').get('center_xyz'))

        return centers_xyz


if __name__ == '__main__':
    try:
        rospy.init_node('pixel2D_to_pos3D', anonymous=True)
        processor = ConvertBBox2DtoPos3D()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Face and pedestrian to distance node failed!')
