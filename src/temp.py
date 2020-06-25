#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
from multiprocessing import Process
import sys
# Callbacks definition

def move_robot(robot_id, x_start, y_start, x_finish, y_finish):
    rospy.init_node(robot_id[0:2]+'send_goal')
    navclient = actionlib.SimpleActionClient(robot_id+'move_base',MoveBaseAction)
    navclient.wait_for_server()

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

    #navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    navclient.send_goal(goal, done_cb, active_cb)
    finished = navclient.wait_for_result()

    if not finished:
        rospy.logerr("Action server not available!")
    else:
        rospy.loginfo ( navclient.get_result())
    t = 20
    time.sleep(t)
    i=0
    while i<t:
        print('sleeping ', i)
        i = i+1
    navclient = actionlib.SimpleActionClient(robot_id+'move_base',MoveBaseAction)
    navclient.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x_finish
    goal.target_pose.pose.position.y = y_finish
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

class Task:
    def __init__(self, task_id, start_location, finish_location):
        self.task_task_id = task_id
        self.task_start_location = start_location
        self.task_finish_location = finish_location

    def task_info(self):
        print('Task info: ' 'id', self.task_task_id, 'go from', self.task_start_location.location_name, 'to', self.task_finish_location.location_name)



#---------------------------


#navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
#navclient.wait_for_server()

print('press any key to execute logistic task from storage to assembly line')
user_input1 = input()

#move to a start point


p1 = Process(target=move_robot('r3/', -13, -4, 7, -10))
p1.start()
time.sleep(5)
p2 = Process(target=move_robot('r2/', -11, 11, 7, 9))
p2.start()
time.sleep(5)
p3 = Process(target=move_robot('r1/', 7, -3, -1, 7))
p2.start()
print('task finished')