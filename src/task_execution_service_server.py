#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service 
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


def my_callback(request):
    print("My_callback has been called")
    #-----------------------------------------------------------------
    print('going to point')
    #rospy.init_node('send_goal')
    navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    navclient.wait_for_server()
    # Example of navigation goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = -13.0
    goal.target_pose.pose.position.y = 4.0
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
    
    print('press any key if robot is loaded')
    user_input = input()
    if user_input == 1:
        print('done')
    else:
        print('done')
    print('done 2')
    #-----------------------------------------------------------------
    return EmptyResponse() # the service Response class, in this case EmptyResponse
    #return MyServiceResponse(len(request.words.split()))

rospy.init_node('task_execution_node')
my_service = rospy.Service('/task_execution_service_server', Empty , my_callback) # create the Service called my_service with the defined callback
rospy.spin() # maintain the service open.