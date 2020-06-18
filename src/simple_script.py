#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#-------------------------set_goal----------------------------------------
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
#--------------------------------------------------------------------------

#-------------------------set_goal----------------------------------------

""" def move_robot(x_coord, y_coord):
    rospy.init_node('send_goal')

    navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    navclient.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x_coord
    goal.target_pose.pose.position.y = y_coord
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 0.0

    navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    finished = navclient.wait_for_result()

    if not finished:
        rospy.logerr("Action server not available!")
    else:
        rospy.loginfo ( navclient.get_result()) """
#--------------------------------------------------------------------------




class Location:
    def __init__(self, name, x_coordinate, y_coordinate):
        self.location_name = name
        self.location_x_coordinate = x_coordinate
        self.location_y_coordinate = y_coordinate

    def location_info(self):
        print('Location info: ', self.location_name, self.location_x_coordinate, self.location_y_coordinate)

point_1 = Location('Storage', 7, -10)
#j1.location_info()

point_2 = Location('Assembly station', -13, -4)
#j2.location_info()

class Task:
    def __init__(self, task_id, start_location, finish_location):
        self.task_task_id = task_id
        self.task_start_location = start_location
        self.task_finish_location = finish_location

    def task_info(self):
        print('Task info: ' 'id', self.task_task_id, 'go from', self.task_start_location, 'to', self.task_finish_location)

#test_task = Task('#1', point_1.location_name, point_2.location_name)
#test_task.task_info()

print('choose where to go: "1" for storage; "2" for assembly station')
user_input = input()
if user_input == 1:
    print('going to ', point_1.location_name)
    rospy.init_node('send_goal')

    navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    navclient.wait_for_server()

# Example of navigation goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = 7.0
    goal.target_pose.pose.position.y = -10.0
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
    
elif user_input == 2:
    print('going to ', point_2.location_name)
    rospy.init_node('send_goal')

    navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    navclient.wait_for_server()

# Example of navigation goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = -13.0
    goal.target_pose.pose.position.y = -4.0
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
else:
    print('stay')




