#!/usr/bin/env python
'''
# Republish the image part of the /face/object and /pedestrian/object
'''

import rospy
from sensor_msgs.msg import Image

from vz_face_recognition.msg import HSE_Object, HSE_Objects

def face_callback(data):
    pub_face.publish(data.image)

def ped_callback(data):
    pub_person.publish(data.image)
    
if __name__ == '__main__':
    try:
        rospy.init_node('republish_ped_face', anonymous=True)

        pub_face = rospy.Publisher('republished_face_obj_img', Image, queue_size=1)
        pub_person = rospy.Publisher('republished_person_obj_img', Image, queue_size=1)

        rospy.Subscriber("/pedestrian/object", HSE_Objects, ped_callback)
        rospy.Subscriber("/face/object", HSE_Objects, face_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        print("Republish ped face images failed!")