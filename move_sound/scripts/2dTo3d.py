#!/usr/bin/env python3
'''
Purpose : Subscribe to HSE modules face or person bounding box and create a person of interest in map frame 
for navigation to use
'''
# Non ROS
import cv2
import struct
from scipy.spatial.transform import Rotation

# System level imports
import sys
import os

path_to_modules = os.environ.get('PATH_TO_MODULES')
path_to_gaurav_ws = '/home/hello-robot/gaurav_ws' #os.environ.get('CATKIN_DIR')
path_to_face_recognition=str(path_to_modules) + '/face_recognition'
path_to_common = str(path_to_face_recognition) + '/common/python'
sys.path.insert(0, path_to_common)
sys.path.insert(1, str(path_to_gaurav_ws)+"/src/stretch_ros")

path_to_face_recognition_demo = str(path_to_face_recognition) + '/face_recognition_demo/python'
sys.path.insert(0, path_to_face_recognition_demo)
# import face_recognition_exe as fre

from stretch_deep_perception.nodes import detection_2d_to_3d as d2
from openvino.model_zoo.model_api.models import OutputTransform
from move_sound.msg import HSE_Object, HSE_Objects
# ROS 
import rospy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import ros_numpy
import tf2_ros
import tf2_geometry_msgs
import message_filters

class CameraFrameConverter:
    
    def __init__(self):

        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        self.prev_hse_object = None

        self.detections_2d = None
        self.detections_3d = None

        self.fit_plane = False
        self.min_box_side_m = None
        self.max_box_side_m = None
        self.modify_3d_detections = None
        
        # self.rgb_topic_name = rospy.get_param('nav_rgb_topic')
        self.rgb_image_subscriber = message_filters.Subscriber('/camera/color/image_raw', Image)

        # self.depth_topic_name = rospy.get_param('nav_depth_topic')
        self.depth_image_subscriber = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

        # self.camera_info_topic_name = rospy.get_param('nav_camera_info_topic')
        self.camera_info_subscriber = message_filters.Subscriber("/camera/color/camera_info", CameraInfo)
        
        # self.hse_obj_topic_name = rospy.get_param('nav_hse_object_topic')
        self.hse_obj_subscriber = message_filters.Subscriber("/HSE/person_object", HSE_Object)   # /pedestrian/object

        # Tuning parameter for slop when time syncing
        # self.slop = rospy.get_param('nav_slop')
        self.slop=10

        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_image_subscriber, self.depth_image_subscriber, self.camera_info_subscriber,self.hse_obj_subscriber], queue_size=10,slop=self.slop)
        self.synchronizer.registerCallback(self.image_callback)
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.nav_poi_pose_topic   = rospy.get_param('nav_poi_pose_topic') 
        self.nav_poi_marker_topic = rospy.get_param('nav_poi_marker_topic')

        # Publish the PoseStamped
        self.poi_cam_frame_pose_pub = rospy.Publisher("/Visual_Goal", PoseStamped, queue_size=1)
        # Publish the Pose Marker
        self.poi_cam_frame_marker_pub = rospy.Publisher(self.nav_poi_marker_topic, Marker, queue_size=1)

        # Frame names for source (camera) and destination (map or odom)
        self.frame_from_3d = "camera_color_optical_frame"#rospy.get_param('nav_camera_frame_name') 
        self.frame_to_3d   = "map"#rospy.get_param('nav_planning_frame_name')

    # We subscribe to the images that come from the factory 
    # realsense node to make this script compatible with 
    # detection_node.py in stretch deep perception

    def image_callback(self, ros_rgb_image, ros_depth_image, rgb_camera_info, hse_object):

        self.rgb_image = ros_numpy.numpify(ros_rgb_image)
        self.depth_image = ros_numpy.numpify(ros_depth_image)
        self.camera_info = rgb_camera_info

        # Extract the raw data       
        if(hse_object != None and (self.prev_hse_object and self.prev_hse_object.tl != hse_object.tl)):
            # The bounding box 2d to 3d needs tl and br.
            tl_point_x =  hse_object.tl.x
            tl_point_y =  hse_object.tl.y
            orig_width =  hse_object.width
            orig_height = hse_object.height

            br_point_x = tl_point_x + orig_width
            br_point_y = tl_point_y + orig_height

            pixel_center = (tl_point_x + (hse_object.width / 2), tl_point_y + (hse_object.height / 2))

            print('non inverted center', pixel_center)

            # Artificially creating a bounding box in the 'detections_2d' format for ease of interface use
            self.detections_2d = [{'class_id': 0, 'label': 'person', 'confidence': 1.0, 'box': (tl_point_x, tl_point_y, br_point_x, br_point_y)}]

            # OpenCV expects bgr images, but numpify by default returns rgb images.
            self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)

            # Get the output of the detections 2d to 3d module. Need to subsequently convert it to PoseStamped 
            self.detections_3d = d2.detections_2d_to_3d(self.detections_2d, self.rgb_image, self.camera_info, self.depth_image, fit_plane=self.fit_plane, min_box_side_m=self.min_box_side_m, max_box_side_m=self.max_box_side_m)

            if self.detections_3d[0].get('box_3d'):
                center_xyz = self.detections_3d[0].get('box_3d').get('center_xyz')

                try:
                    transform = self.tf_buffer.lookup_transform(self.frame_to_3d,self.frame_from_3d,rospy.Time(0))
                    pose_stamped_camera_frame = PoseStamped()
                    pose_stamped_map_frame = PoseStamped()

                    pose_stamped_camera_frame.pose.position.x = center_xyz[0]
                    pose_stamped_camera_frame.pose.position.y = center_xyz[1]
                    pose_stamped_camera_frame.pose.position.z = center_xyz[2]

                    pose_stamped_camera_frame.header.frame_id = self.frame_from_3d
                    pose_stamped_camera_frame.header.stamp = hse_object.header.stamp

                    pose_stamped_map_frame = tf2_geometry_msgs.do_transform_pose(pose_stamped_camera_frame, transform)

                    

                    # Publish the map pose for navigation use
                    self.poi_cam_frame_pose_pub.publish(pose_stamped_map_frame)
                    
                    #################################
                    # Publish the map point marker for visualization
                    marker = Marker()
                    
                    marker.header.frame_id = self.frame_to_3d
                    marker.header.stamp = hse_object.header.stamp
                    marker.ns = "goal_point_ns"
                    marker.action = Marker.ADD
                    marker.id = 9999
                    marker.type = Marker.POINTS
                    marker.scale.x = marker.scale.y = 0.2
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 1.0

                    marker.points.append(pose_stamped_map_frame.pose.position)


                    print('blue marker 3d', pose_stamped_map_frame.pose.position)
                    self.poi_cam_frame_marker_pub.publish(marker)


                except:
                    print("Camera converter node waiting for transform!")

        # Update previous object
        self.prev_hse_object = hse_object


if __name__ == '__main__':
    try:
        rospy.init_node('CameraFrameConverterNode', anonymous=True)
        processor = CameraFrameConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Camera Frame Converter Node Failed!')
        pass
