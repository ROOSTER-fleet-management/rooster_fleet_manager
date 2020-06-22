#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Callbacks definition

def move_robot(x_coord, y_coord):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x_coord
    goal.target_pose.pose.position.y = y_coord
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.662
    goal.target_pose.pose.orientation.w = 0.750

    navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    finished = navclient.wait_for_result()

    if not finished:
        rospy.logerr("Action server not available!")
    else:
        rospy.loginfo ( navclient.get_result())

def active_cb(extra):
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    rospy.loginfo("Current location: "+str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")

class Location:
    def __init__(self, name, x_coordinate, y_coordinate):
        self.location_name = name
        self.location_x_coordinate = x_coordinate
        self.location_y_coordinate = y_coordinate

    def location_info(self):
        print('Location info: ', self.location_name, self.location_x_coordinate, self.location_y_coordinate)

storage_1 = Location('Storage #1', -13, -4)
#j1.location_info()

storage_2 = Location('Storage #2', -11, 11)

assembly_station_1 = Location('Assembly station #1', 7, -10)

assembly_station_2 = Location('Assembly station #2', 7, 9)

start_point = input('where is a cargo: \n "1" for Storage #1 \n "2" for Storage #2 \n')


rospy.init_node('send_goal')

navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
navclient.wait_for_server()

print('press any key to execute logistic task from storage to assembly line')
user_input1 = input()

#move to a start point
move_robot(-13, -4)

print('press any key if loading is finished and robot can move to a finish point')
user_input = input()

#move to a finish point
move_robot(7, -10)

print('task finished')