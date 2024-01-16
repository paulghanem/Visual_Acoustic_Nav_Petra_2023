#!/usr/bin/env python

# imports

# ROS
import rospy
import message_filters

# Transforms
import tf
import tf2_ros
import tf2_geometry_msgs

# Messages
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point
from leg_tracker.msg import Person, PersonArray, Leg, LegArray


class Transformer:
    """
    NAME IS TEMPORARY - Transforms people_tracked data into camera frame.
    """

    def __init__(self):
        """
        Class constructor
        """

        # Frames
        self.leg_tracker_target = rospy.get_param("leg_tracker_target_frame")
        self.leg_tracker_source = None # Set-up in callback.

        # Input topic parameter name
        self.input_array_topic_name = rospy.get_param("leg_tracker_topic")

        # Input callback
        self.input_subscriber = rospy.Subscriber(self.input_array_topic_name, PersonArray, self.pose_callback)


        # Transform initialisation
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Output topic initialisation
        self.output_array_topic_name = rospy.get_param("transformed_leg_tracker_topic")
        self.output_publisher = rospy.Publisher(self.output_array_topic_name, PersonArray, queue_size=1)

    def pose_callback(self, personarray):
        """
        Creates the transform of the pose transforming it to the correct frame. Publishes the new array.
        :param personarray: Callback data input
        """

        self.leg_tracker_source = personarray.header.frame_id

        transform = self.tf_buffer.lookup_transform(self.leg_tracker_target, self.leg_tracker_source, rospy.Time(0))

        output_pose_stamped = PersonArray()

        output_pose_array = []

        output_pose_stamped.header.seq = personarray.header.seq
        output_pose_stamped.header.stamp = personarray.header.stamp
        output_pose_stamped.header.frame_id = self.leg_tracker_target

        for person_object in personarray.people:
            input_person = Person()
            output_person = Person()

            # Get input pose

            # input pose positions
            input_person.pose.position.x = person_object.pose.position.x
            input_person.pose.position.y = person_object.pose.position.y
            input_person.pose.position.z = person_object.pose.position.z
            
            # input pose orientations
            input_person.pose.orientation.x = person_object.pose.orientation.x
            input_person.pose.orientation.y = person_object.pose.orientation.y
            input_person.pose.orientation.z = person_object.pose.orientation.z
            input_person.pose.orientation.w = person_object.pose.orientation.w
            
            
            input_person.id = person_object.id
            
            # Transform to selected frame
            output_person.pose = tf2_geometry_msgs.do_transform_pose(input_person, transform).pose
            output_person.id = input_person.id

            output_pose_array.append(output_person)
        
        output_pose_stamped.people = output_pose_array
        
        # Publish transformed poses
        self.output_publisher.publish(output_pose_stamped)
        rospy.loginfo(output_pose_stamped)


if __name__ == "__main__":
    try:
        rospy.init_node("leg_pose_transformer", anonymous=True)
        processor = Transformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("leg_pose_transformer node has failed")
        pass

