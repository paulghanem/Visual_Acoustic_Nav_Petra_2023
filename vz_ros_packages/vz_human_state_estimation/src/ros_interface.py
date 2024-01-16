#!/usr/bin/python3

# Python imports
import queue
from math import sqrt
import numpy as np

# ROS imports
import rospy
from geometry_msgs.msg import Point, PoseStamped
from leg_tracker.msg import PersonArray
from visualization_msgs.msg import Marker
import message_filters

# Transforms
import tf2_ros
import tf2_geometry_msgs

# OpenCv imports
from cv_bridge import CvBridge

# Custom imports
from vz_ros_wrappers.msg import Bounding_Boxes_3D


class KalmanFilter(object):
    def __init__(self, dt, std_acc):  # (self, dt, u_x, u_y, u_z, std_acc):
        """
        :param dt: sampling time (time for 1 cycle)
        :param u_x: acceleration in x-direction
        :param u_y: acceleration in y-direction
        :param std_acc: process noise magnitude
        """

        # Define the  control input variables
        # self.u = np.array([[u_x], [u_y], [u_z]])
        # Intial State
        self.x = np.array([[0], [0], [0], [0], [0], [0]])
        # Define the State Transition Matrix A
        self.A = np.array([[1, 0, 0, dt, 0, 0],
                           [0, 1, 0, 0, dt, 0],
                           [0, 0, 1, 0, 0, dt],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])

        self.Q = np.eye(self.A.shape[1])*std_acc**2

        # Initial Covariance Matrix
        self.P = np.eye(self.A.shape[1])*0.01

    def predict(self, dt):
        # Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, 0, dt, 0, 0],
                            [0, 1, 0, 0, dt, 0],
                            [0, 0, 1, 0, 0, dt],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])

        self.x = np.dot(self.A, self.x)

        # Calculate error covariance
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return np.array(self.x[0:3])

    def update(self, R_vars, z):
        z = z.transpose()

        # Define Measurement Mapping Matrix
        H = np.block([np.eye(3), np.zeros((3, 3))])
        # Initial Measurement Noise Covariance
        R = np.diag(R_vars[0])

        # Refer to :Eq.(11), Eq.(12) and Eq.(13)  in https://machinelearningspace.com/object-tracking-simple-implementation-of-kalman-filter-in-python/?preview_id=1364&preview_nonce=52f6f1262e&preview=true&_thumbnail_id=1795
        S = np.dot(H, np.dot(self.P, H.T)) + R
        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))  # Eq.(11)
        self.x = (self.x + np.dot(K, (z - np.dot(H, self.x))))  # Eq.(12)
        I = np.eye(H.shape[1])

        # Update error covariance matrix
        self.P = np.dot((I - np.dot(K, H)), self.P)  # Eq.(13)
        return np.array(self.x[0:3])


class HumanStateEstimation:

    def __init__(self):
        self.bridge = CvBridge()

        self.queue = queue.PriorityQueue()
        self.horizon_time = rospy.Duration.from_sec(1)

        # Subscriber for all the face and person bboxes as 3d points
        self.fr_data_sub_3d = message_filters.Subscriber(
            '/bb_cameraframe_face', Bounding_Boxes_3D, queue_size=15)
        self.pt_data_sub_3d = message_filters.Subscriber(
            '/bb_cameraframe_person', Bounding_Boxes_3D, queue_size=15)
        comb = message_filters.TimeSynchronizer(
            [self.fr_data_sub_3d, self.pt_data_sub_3d], 25)
        comb.registerCallback(self.enqueue_fb)

        # Subscriber for all the leg persons as 3d points
        rospy.Subscriber("/transformed_leg_poses",
                         PersonArray, self.enqueue_legs, queue_size=10)

        self.position_pub = rospy.Publisher(
            "/HSE/position_state", PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher(
            "/HSE/position_marker", Marker, queue_size=10)

        # Transform initialisation
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Kalman filter parameter initialization
        self.pos = np.zeros(3)
        self.df = np.zeros(3)
        self.dp = np.zeros(3)
        self.rf = np.full(3, 1e-5)
        self.rp = np.full(3, 1e-1)
        self.rl = np.full(3, 1e-4)

        self.thresh_fb = 3
        self.thresh_leg = 3

        self.dt = 0.15
        self.last_timestamp = None
        self.start_time = 0

        q = 1e-3
        self.KF = KalmanFilter(self.dt, q)

        # 1/dt hz update KF state publishing system
        rospy.Timer(rospy.Duration(self.dt), self.process_queue)

    # ------------- QUEUE METHODS --------- #
    def process_queue(self, *args):
        while True:
            # Gets the first frame in the queue, has smallest timestamp
            latest = self.queue.get()

            # Break while if latest time is ahead of horizon
            if latest[0] > rospy.Time.now() - self.horizon_time:
                break

            if not self.last_timestamp:
                self.last_timestamp = latest[0]

            if latest[1] == 'L':
                # Processes leg frame
                self.dt = (latest[2].header.stamp -
                           self.last_timestamp).to_sec()
                self.last_timestamp = latest[2].header.stamp

                positions = []
                for object in latest[2].people:
                    positions.append(object.pose.position)

                self.update_KF('L', positions, latest[2].header.stamp)
            elif latest[1] == 'FB':
                # Processes dequeued Face/Body frame
                self.dt = (latest[2].header.stamp -
                           self.last_timestamp).to_sec()
                self.last_timestamp = latest[2].header.stamp

                transform = self.tf_buffer.lookup_transform(
                    'odom', latest[2].header.frame_id, latest[2].header.stamp, rospy.Duration(1))

                # Processes face frame
                face_pts = []
                ids = []
                for object in latest[2].bounding_boxes:
                    fr_pose_stamped = PoseStamped()
                    fr_pose_stamped.header = latest[2].header
                    fr_pose_stamped.pose = object.pose
                    # Transform to selected frame
                    output_face_pose = tf2_geometry_msgs.do_transform_pose(
                        fr_pose_stamped, transform).pose

                    face_pts.append(output_face_pose.position)
                    ids.append(object.label)

                self.face_3d_times = latest[2].header.stamp
                self.ids = ids

                transform = self.tf_buffer.lookup_transform(
                    'odom', latest[3].header.frame_id, latest[3].header.stamp, rospy.Duration(1))

                # Processes body frame
                person_pts = []
                for object in latest[3].bounding_boxes:
                    pt_pose_stamped = PoseStamped()
                    pt_pose_stamped.header = latest[3].header
                    pt_pose_stamped.pose = object.pose
                    # Transform to selected frame
                    output_ped_pose = tf2_geometry_msgs.do_transform_pose(
                        pt_pose_stamped, transform).pose

                    person_pts.append(output_ped_pose.position)

                self.update_KF(
                    'FB', face_pts, latest[2].header.stamp, ids, person_pts, latest[3].header.stamp)
            else:
                self.update_KF('N')

    # Adds Face/body synchronized frame to the queue
    def enqueue_fb(self,  fr_data, pt_data):
        timestamp = fr_data.header.stamp

        if timestamp >= rospy.Time.now() - self.horizon_time:
            if self.start_time == 0:
                self.start_time = fr_data.header.stamp.to_nsec()

            self.queue.put((timestamp, 'FB', fr_data, pt_data))

    # Adds legs frame to the queue
    def enqueue_legs(self,  data):
        timestamp = data.header.stamp

        if timestamp > rospy.Time.now() - self.horizon_time:
            if self.start_time == 0:
                self.start_time = data.header.stamp.to_nsec()

            self.queue.put((timestamp, 'L', data))

    def update_KF(self, mode, *params):
        multimodal_q = []
        multimodal_time = []

        # KF position update
        KF_3d_pt = self.pos.flatten() - self.df
        face_exists = leg_exists = person_exists = False

        if mode == 'L':
            leg_3d_pts = params[0]
            leg_3d_times = params[1]

            dist = []
            if len(leg_3d_pts) > 0:
                for pt in leg_3d_pts:
                    distance = sqrt(
                        (KF_3d_pt[0] - pt.x)**2 + (KF_3d_pt[1] - pt.y)**2 + (KF_3d_pt[2] - pt.z)**2)
                    dist.append(distance)

                i = np.argmin(dist)
                if dist[i] < self.thresh_leg:
                    leg_3d_pt = np.array(
                        [leg_3d_pts[i].x, leg_3d_pts[i].y, leg_3d_pts[i].z])
                    leg_exists = True
                    multimodal_q.append(leg_3d_pt)
                    multimodal_time.append(leg_3d_times)
        elif mode == 'FB':
            face_3d_pts = params[0]
            face_3d_times = params[1]
            face_labels = params[2]

            person_3d_pts = params[3]
            person_3d_times = params[4]

            if len(face_3d_pts) > 0:
                for index, label in enumerate(face_labels):
                    if len(face_labels) == len(face_3d_pts) and label == "0":
                        face_3d_pt = np.array(
                            [face_3d_pts[index].x, face_3d_pts[index].y, face_3d_pts[index].z])
                        face_exists = True
                        multimodal_q.append(face_3d_pt)
                        multimodal_time.append(face_3d_times)

            dist = []
            if len(person_3d_pts) > 0:
                for pt in person_3d_pts:
                    distance = sqrt(
                        (KF_3d_pt[0] - pt.x)**2 + (KF_3d_pt[1] - pt.y)**2 + (KF_3d_pt[2] - pt.z)**2)
                    dist.append(distance)

                i = np.argmin(dist)
                if dist[i] < self.thresh_fb:
                    person_3d_pt = np.array(
                        [person_3d_pts[i].x, person_3d_pts[i].y, person_3d_pts[i].z])
                    person_exists = True
                    multimodal_q.append(person_3d_pt)
                    multimodal_time.append(person_3d_times)

        # case 1 FP
        if face_exists and person_exists:
            R_vars = np.array((self.rf, self.rp))
            z = np.array([face_3d_pt + self.df, person_3d_pt + self.dp])
        # case 2 F
        elif face_exists:
            R_vars = np.array([self.rf])
            z = np.array([face_3d_pt + self.df])
        # case 3 P
        elif person_exists:
            R_vars = np.array([self.rp])
            z = np.array([person_3d_pt + self.dp])
        # case 4 L
        elif leg_exists:
            R_vars = np.array([self.rl])
            z = np.array([leg_3d_pt])

        if multimodal_q:
            # Sort multimodal queue and iterate through readings, updating the KF
            sorted_time_idx = np.argsort(multimodal_time)
            self.KF.predict(self.dt)

            for i in sorted_time_idx:
                self.pos = self.KF.update(
                    np.array([R_vars[i]]), np.array([z[i]]))
        # case 5 empty
        else:
            self.KF.predict(self.dt)
            self.pos = np.array(self.KF.x[0:3])

        position_msg = PoseStamped()
        position_msg.header.frame_id = "odom"
        position_msg.header.stamp = self.last_timestamp

        position_msg_point = Point()
        position_msg_point.x = self.pos[0]
        position_msg_point.y = self.pos[1]
        position_msg_point.z = self.pos[2]

        position_msg.pose.position = position_msg_point

        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = self.last_timestamp
        marker.ns = "map_goal_point_ns"
        marker.action = Marker.ADD
        marker.id = 9999
        marker.type = Marker.POINTS
        marker.scale.x = marker.scale.y = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points.append(position_msg_point)

        self.position_pub.publish(position_msg)
        self.marker_pub.publish(marker)


if __name__ == '__main__':
    try:
        rospy.init_node('human_state_estimation', anonymous=True)
        processor = HumanStateEstimation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
