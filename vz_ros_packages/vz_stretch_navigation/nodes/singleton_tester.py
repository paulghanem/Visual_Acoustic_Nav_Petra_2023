'''
File to test single instance of robot being created across calls
'''

import stretch_body.robot as robot
import utils


class Tester:
    test_value = 0

    def __init__(self) -> None:
        self.robot = robot.Robot()
        self.head = self.robot.head
        


    def create_and_destroy(self) -> None:
        new_instance = Tester()
        print('created instance')

    def __del__(self):
        print('I got destroyed')

    def move_head(self, degrees):
        self.head.move_to('head_pan', utils.deg_to_rad(degrees))

    def test_update_param(self, test_number=None):
        
        def function():
            if test_number is None:
                test_number_final = self.test_value
            return test_number_final

        return function



# tester1 = Tester()
# tester2 = Tester()
# tester3 = Tester()

# print(tester1.robot == tester2.robot)
# print(tester1.head == tester2.head)

# tester1.create_and_destroy()

# print(tester1.robot == tester2.robot)
# print(tester1.head == tester2.head)

# # # startup testing single
# # tester1.robot.startup()
# #     # test move commands
# # tester1.move_head(30)
# # time.sleep(2)
# # tester1.move_head(-30)
# # time.sleep(2)
# # tester1.robot.stop()
# # time.sleep(2)
# # tester1.move_head(30)


# # multiple startup testing
# tester1.robot.startup()
# tester2.robot.startup()

# # test that both robots can move the head
# tester1.move_head(30)
# time.sleep(2)
# tester2.move_head(-30)
# time.sleep(2)

# tester2.robot.stop()

# # test that after stopping tester2 can still move the robot
# tester1.move_head(30)
# time.sleep(2)
# tester2.move_head(-30)
# time.sleep(2)

# # test that a robot that is instancated but not started can move the robot
# print('test 4')
# tester4 = Tester()
# tester1.move_head(30)
# time.sleep(2)
# tester4.move_head(0)
# time.sleep(2)


# # test that no robot can now move the robot
# tester1.robot.stop() # stops last instance
# tester1.move_head(-30)
# time.sleep(2)
# tester2.move_head(-30)
# time.sleep(2)
# tester3.move_head(-30)

# # check how starting back up works (it doesn't)
# tester1.robot.startup()
# time.sleep(2)
# tester1.move_head(-30)
# time.sleep(2)

# # tests that if an object is used to set something in a function. if that objects value is updated does it get updated
# # (answer: it does not)
# my_number = 4
# func1 = Tester.test_update_param(my_number)
# my_number = 2
# func2 = Tester.test_update_param(my_number)
# print(func1(), func2())

# # tests that if an object is used to set something in a function. if that objects value is updated does it get updated
# #  (when the object is mutable)
# # (answer: it does)
# # a way to do this is to give it a mutable object (dict, list) that contains the needed information then update it.
# my_number = [4]
# func1 = Tester.test_update_param(my_number)
# my_number[0] = 2
# func2 = Tester.test_update_param(my_number)
# print(func1(), func2())

# test
my_tester = Tester()
my_func = my_tester.test_update_param()
print(my_func())
my_tester.test_value = 4
my_func2 = my_tester.test_update_param()
print(my_func(), my_func2())


# it curently only reruns the function when the waypoint value is reset
# this also need to be changed whenever the radius is updated

# proposal1
# two functions funcA and funcB
# there is also a funcC that is the current function (but modified to use no parameters and only use class values)
# when the position is update funcA that then updates a value within funcC then calls it
# when the radius is updated (since this is a param is a little different) funcB updates a different value ten called funcC
    # this could also be modified to only call whenever there is a change


# proposal2
# allow for the radius object within the class to be setable outside the class (no longer uses param)
# this could cause for a lot of potential issues


# questions about the function

# where does the value 0.78 for theta come from (is that radians (close to 45 degrees)? is that degrees?)
# which was is the positive x direction for the robot (as used in arctan)
# isn't it possible that the function called while changing the point does nothing
# could this back up since the angle would presumably be greater the 45 degrees?
# since you are only ever looking for the closest point along the circle I belive
#       this could be solved for directly instead of looping


# for update:
# turn current function into helper
# create it so there are calls to it for different updates
# remove the 45 degree hack
# to ensure that the case of radius change the previous position is remove (since the goal is no longer valid)






