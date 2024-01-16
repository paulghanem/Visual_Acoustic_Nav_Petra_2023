#!/usr/bin/env python

import math
import numpy as np

from sklearn.metrics import pairwise_distances

import rospy

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

import tf.transformations as tr
import tf2_ros
import tf2_geometry_msgs


class MoveBaseGoalSetter(object):
    UPDATE_PERIOD = 1  # seconds

    def __init__(self):
        self._orig_p = None
        self._orig_q = None
        self._orig_yaw = None
        self._occ_grid = None
        self._grid_img = None

        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Radius of the circle, e.g. 1m
        self._radius = rospy.get_param('~radius', 1.0)
        # Goal distance for updates
        self._num_circ_samples = rospy.get_param('~num_circ_samples', 32)

        # Start ROS publishers/subscribers
        rospy.Subscriber('/odom', Odometry, callback=self.odom_cb)
        rospy.Subscriber("/HSE/position_state", PoseStamped,
                         callback=self.update_poi)
        self.radius_timer = rospy.Timer(rospy.Duration(
            self.UPDATE_PERIOD, 0), self.update_radius)

        self.__move_base_goal_pub = rospy.Publisher(
            '/waypoint', PoseStamped, queue_size=10)
        self.__circ_marker_pub = rospy.Publisher(
            '/circle_marker', Marker, queue_size=10)

    def odom_cb(self, odom):
        self._orig_p = np.array([odom.pose.pose.position.x,
                                 odom.pose.pose.position.y,
                                 odom.pose.pose.position.z])
        self._orig_q = np.array([odom.pose.pose.orientation.x,
                                 odom.pose.pose.orientation.y,
                                 odom.pose.pose.orientation.z,
                                 odom.pose.pose.orientation.w])

    def update_radius(self, timer):
        new_radius = rospy.get_param('~radius')
        if new_radius != self._radius:
            self._radius = new_radius

    def update_poi(self, pose_stamped):
        poi_pos = np.array([pose_stamped.pose.position.x,
                            pose_stamped.pose.position.y,
                            pose_stamped.pose.position.z])

        # Evenly space angles
        theta = np.linspace(-math.pi, math.pi, self._num_circ_samples)

        marker = Marker()
        marker.header.frame_id = pose_stamped.header.frame_id
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # Marker scale
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # Marker color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Marker line points
        marker.points = []
        circ_samples = []
        for i in range(self._num_circ_samples):
            # Marker point
            circ_mark_pt = Point()
            # Calculating coordinates
            circ_mark_pt.x = self._radius * math.cos(theta[i]) + poi_pos[0]
            circ_mark_pt.y = self._radius * math.sin(theta[i]) + poi_pos[1]
            circ_mark_pt.z = 0.0
            marker.points.append(circ_mark_pt)

            circ_samples.append([circ_mark_pt.x, circ_mark_pt.y])

        circ_samples_np = np.array(circ_samples)

        # Work out array of pairwise distances from current pose
        dist = pairwise_distances(np.expand_dims(
            self._orig_p[:2], axis=0), circ_samples_np, metric='euclidean')
        dist_sorted_idx = np.argsort(dist, axis=1)
        closest_pt = circ_samples_np[dist_sorted_idx[0][0]]

        # Closest pose should have orientation determined by forward kinematics
        closest_pose = PoseStamped()
        closest_pose.header = pose_stamped.header
        closest_pose.pose.position.x = closest_pt[0]
        closest_pose.pose.position.y = closest_pt[1]
        closest_pose.pose.position.z = 0.0

        theta = np.arctan2(poi_pos[1] - closest_pt[1],
                           poi_pos[0] - closest_pt[0])
        trans_q = tr.quaternion_from_euler(0, 0, theta)
        closest_pose.pose.orientation.x = trans_q[0]
        closest_pose.pose.orientation.y = trans_q[1]
        closest_pose.pose.orientation.z = trans_q[2]
        closest_pose.pose.orientation.w = trans_q[3]

        transform = self._tf_buffer.lookup_transform('map',
                                                     # source frame:
                                                     'odom',
                                                     # get the tf at the time the pose was valid
                                                     closest_pose.header.stamp,
                                                     # wait for at most 1 second for transform, otherwise throw
                                                     rospy.Duration(1.0))

        closest_pose_transformed = tf2_geometry_msgs.do_transform_pose(
            closest_pose, transform)

        self.__move_base_goal_pub.publish(closest_pose_transformed)
        # Publish the Marker
        self.__circ_marker_pub.publish(marker)

    def on_shutdown(self):
        rospy.loginfo("Shutting down nav_goal_setter.")


def nav_goal_set():
    rospy.init_node('nav_goal_setter')
    mvgs = MoveBaseGoalSetter()

    rospy.on_shutdown(mvgs.on_shutdown)
    rospy.spin()


if __name__ == '__main__':
    try:
        nav_goal_set()
    except rospy.ROSInterruptException:
        print("Nav Goal Setter Node Failed")
        pass
