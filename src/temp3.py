#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import time
# from multiprocessing import Process
import sys

class RobotMoveBase:
    def __init__(self, robot_id):
        self.id = robot_id

    # Callbacks definition
    def active_cb(self):
        rospy.loginfo(self.id + ". Goal pose being processed")

    def feedback_cb(self, feedback):
        pass    # Don't spam the console for now..
        # rospy.loginfo("Current location: "+str(feedback))

    def done_cb(self, status, result):
        if status == 3:
            rospy.loginfo(self.id + ". Goal reached")
        if status == 2 or status == 8:
            rospy.loginfo(self.id + ". Goal cancelled")
        if status == 4:
            rospy.loginfo(self.id + ". Goal aborted")

    def move_robot(self, x_start, y_start, x_finish, y_finish):
        self.navclient = actionlib.SimpleActionClient(self.id + '/move_base',MoveBaseAction)
        self.navclient.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x_start
        goal.target_pose.pose.position.y = y_start
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.662
        goal.target_pose.pose.orientation.w = 0.750

        self.navclient.send_goal(goal, done_cb=self.done_cb, active_cb=self.active_cb, feedback_cb=self.feedback_cb)
        #finished = navclient.wait_for_result()

        #if not finished:
        #    rospy.logerr("Action server not available!")
        #else:
        #    rospy.loginfo ( navclient.get_result())
    
    def get_status(self):
        return self.navclient.get_state()

rospy.init_node('send_goal')

robot_move_list = [RobotMoveBase("rdg01"), RobotMoveBase("rdg02"), RobotMoveBase("rdg03")]
robot_move_list[0].move_robot(-1, -3, 7, -10)
robot_move_list[1].move_robot(0, 0, 7, -10)
robot_move_list[2].move_robot(-4, 4, 7, -10)
print('if you see this before robots finished their task - it works')

def timer_cb(event):
    # Checking if get_status works. Another way to get the status of the move_base action is to subscribe to /robot_id/move_base/status
    print('-------')
    # print('Last timer called at ' + str(event.last_real))
    # print('Timer should be called at ' + str(event.current_expected))
    # print('Timer was actually called at ' + str(event.current_real))
    # if event.last_real and event.current_real:
    #     print('Actual time between calls ' + str(event.current_real - event.last_real))
    for robot in robot_move_list:
        print(robot.id + ". Status: " + str(robot.get_status()))

rospy.Timer(rospy.Duration(2), timer_cb)

rospy.spin()