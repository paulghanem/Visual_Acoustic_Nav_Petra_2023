#! /usr/bin/python3
import rospy
import stretch_body.robot as robot
import utils

class NavAngleController(object):

    UPDATE_PERIOD = 1 # seconds

    def __init__(self) -> None:
        self.robot = robot.Robot()
        self.robot.startup()
        self.head = self.robot.head

        #test what happens if there are multiple robot instances
        # print('creating new robot')
        # self.robot2 = robot.Robot()
        # self.robot2.startup()
        # self.head2 = self.robot2.head
        # print('created new robot')


        self.rate = 1 #hz

        self.current_goal_tilt_angle = rospy.get_param('nav_goal_tilt_angle')
        self.current_goal_pan_angle = rospy.get_param('nav_goal_pan_angle')

        self.angle_timer = rospy.Timer(rospy.Duration(self.UPDATE_PERIOD, 0), self.angle_update)



    def angle_update(self, timer):
        self.current_goal_pan_angle = rospy.get_param('nav_goal_pan_angle')
        self.current_goal_tilt_angle = rospy.get_param('nav_goal_tilt_angle')
            # self.current_goal_tilt_angle = rospy.get_param('nav_goal_tilt_angle')
        print('got to angle update')
        print('pan: ', self.current_goal_pan_angle, ' tilt: ', self.current_goal_tilt_angle)
        self.head.move_to('head_pan', utils.deg_to_rad(self.current_goal_pan_angle))
        self.head.move_to('head_tilt', utils.deg_to_rad(self.current_goal_tilt_angle))
        

    def shut_down(self):
        print('stopping robot')
        self.robot.stop()



if __name__ == '__main__':
    try:
        rospy.init_node('nav_angle_controller')
        processor = NavAngleController()
        created = True
        rospy.spin() 
    except rospy.ROSInterruptException:
        print('nav_angle_controller node failed')