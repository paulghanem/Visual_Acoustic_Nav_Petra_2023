#!/usr/bin/python3
# ROS imports
from asyncio import tasks
from asyncore import loop
from multiprocessing.dummy import current_process
import rospy
import numpy as np
import std_msgs.msg
# Python imports
import sys
from cv_bridge import CvBridge, CvBridgeError
from math import sqrt
import cv2
import os


class Scheduler:

    ASA_TASK = 'ASA'
    ORVS_TASK = 'ORVS'
    VBPR_TASK = 'VBPR'
    INBED_TASK = 'IN-BED'

    # param paths
    # param path to state that ASA is registering
    ASA_REGISTERING = 'REGISTERING'
    # param path to state when face is registering
    FACE_REGISTERING = 'face_registration'
    # param path to state when navigation is ok to be running
    START_NAVIGATING = 'run_navigation'

    # face registraction is assumed to just happening at launch
    # once param switches I will wait 10 seconds then start audio registraction

    # once audio registration finishes goes into loops of tasks listed bellow (does not return to registration tasks)
    TASK_ORDER = [ASA_TASK, ORVS_TASK, VBPR_TASK, INBED_TASK]

    # specifies at want point the counter should swtich for each task (>= value_specified)
    COUNTER_DICT = {ASA_TASK: 5, ORVS_TASK: 3, VBPR_TASK: 3, INBED_TASK: 20}

    # specifies the location of the 'task is running' parameter for each task
    IS_RUNNING_PARAM_DICT = {
        ASA_TASK: 'scheduler/ASA_run',
        ORVS_TASK: 'scheduler/ORVS_run',
        VBPR_TASK: 'scheduler/VBPR_run',
        INBED_TASK: 'scheduler/INBED_run'
    }

    # specifies the goal parameters for each task
    GOAL_PARAM_VALUES_DICT = {
        ASA_TASK: {'radius': 0.5, 'pan': 0, 'tilt': 0},
        ORVS_TASK: {'radius': 0.5, 'pan': 0, 'tilt': 0},
        VBPR_TASK: {'radius': 1, 'pan': 0, 'tilt': 0},
        INBED_TASK: {'radius': 2.13, 'pan': 0, 'tilt': -15}
    }

    def __init__(self):
        # Load params for feedback subscription
        asa_feedback_topic = rospy.get_param("asa_feedback_topic")
        orvs_feedback_topic = rospy.get_param("orvs_feedback_topic")
        vbpr_feedback_topic = rospy.get_param("vbpr_feedback_topic")
        inbed_feedback_topic = rospy.get_param("inbed_feedback_topic")

        # set initial schedule and params
        self.current_task_position = 0
        self.current_task = self.TASK_ORDER[self.current_task_position]
        self.set_params_for_task(self.current_task)

        # initially set all modules to not be running
        for task in self.TASK_ORDER:
            rospy.set_param(self.IS_RUNNING_PARAM_DICT[task], False)

        # set main cycle loop to not be operational
        self.running_cycle = False

        # Subscribe to nodes
        asa_fb_sub = rospy.Subscriber(
            asa_feedback_topic, std_msgs.msg.Int16, callback=self.asa_callback, queue_size=1)
        orvs_fb_sub = rospy.Subscriber(
            orvs_feedback_topic, std_msgs.msg.Int16, callback=self.orvs_callback, queue_size=1)
        vbpr_fb_sub = rospy.Subscriber(
            vbpr_feedback_topic, std_msgs.msg.Int16, callback=self.vbpr_callback, queue_size=1)
        inbed_fb_sub = rospy.Subscriber(
            inbed_feedback_topic, std_msgs.msg.Int16, callback=self.inbed_callback, queue_size=1)

    def asa_callback(self, msg_data):
        if self.current_task == self.ASA_TASK:
            self.general_callback(msg_data=msg_data)

    def orvs_callback(self, msg_data):
        if self.current_task == self.ORVS_TASK:
            self.general_callback(msg_data=msg_data)

    def vbpr_callback(self, msg_data):
        if self.current_task == self.VBPR_TASK:
            self.general_callback(msg_data=msg_data)

    def inbed_callback(self, msg_data):
        if self.current_task == self.INBED_TASK:
            self.general_callback(msg_data=msg_data)


    def general_callback(self, msg_data):
        if not self.running_cycle:
            # if the cycle is not running do nothing
            return

        counter = msg_data.data

        # check if counter is greater than counter count for task
        if counter >= self.COUNTER_DICT[self.current_task]:
            # if so swich which task is running
            previous_task = self.current_task
            self.current_task_position = (
                self.current_task_position + 1) % len(self.TASK_ORDER)
            self.current_task = self.TASK_ORDER[self.current_task_position]
            self.function_switch(ending_task=previous_task,
                                 starting_task=self.current_task)

    def function_switch(self, ending_task, starting_task):
        # set the start and end values
        rospy.set_param(self.IS_RUNNING_PARAM_DICT[starting_task], True)
        rospy.set_param(self.IS_RUNNING_PARAM_DICT[ending_task], False)

        # update pan angle, tilt angle, and goal radius
        self.set_params_for_task(starting_task)

    def set_params_for_task(self, current_task):
        param_values = self.GOAL_PARAM_VALUES_DICT[current_task]
        radius = param_values['radius']
        pan = param_values['pan']
        tilt = param_values['tilt']
        rospy.set_param('nav_goal_radius', radius)
        rospy.set_param('nav_goal_pan_angle', pan)
        rospy.set_param('nav_goal_tilt_angle', tilt)

    def run(self):
        # run face registration
        while rospy.get_param(self.FACE_REGISTERING):
            rospy.sleep(0.5)  # wait half a second between checks

        # wait 10 seconds between face and audio
        rospy.sleep(10)

        #  run audio
        rospy.set_param(self.ASA_REGISTERING, True)
        while rospy.get_param(self.ASA_REGISTERING):
            rospy.sleep(0.5)  # wait half a second between checks

        rospy.set_param(self.START_NAVIGATING, True)

        # run standard loop
        self.running_cycle = True
        self.current_task_position = 0
        self.current_task = self.TASK_ORDER[self.current_task_position]

        # set only the first module to be running
        rospy.set_param(self.IS_RUNNING_PARAM_DICT[self.current_task], True)
        for task_index in range(1, len(self.TASK_ORDER)):
            task = self.TASK_ORDER[task_index]
            rospy.set_param(self.IS_RUNNING_PARAM_DICT[task], False)
        # update the parameter values for the current task
        self.set_params_for_task(self.current_task)

        # let the loop run through the subscribers and callbacks
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('Scheduler', anonymous=True)
        processor = Scheduler()
        processor.run()
    except rospy.ROSInterruptException:
        print('Scheduler node failed!')
        pass
