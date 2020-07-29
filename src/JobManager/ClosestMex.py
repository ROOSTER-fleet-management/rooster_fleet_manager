#! /usr/bin/env python

import rospy
from simple_sim.srv import GetMexList, GetMexListRequest
from MobileExecutor import MExStatus
from nav_msgs.srv import GetPlan, GetPlanResponse
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
import numpy as np

#get coordinates of each mex. 3 ways: (1) by odometry and (2) by tf from /map to /base_link and (3) by amcl_pose topic
#(1)https://www.theconstructsim.com/ros-qa-know-pose-robot-python/
#(2)https://answers.ros.org/question/341062/how-to-get-a-robot-position-xy-in-a-map-instead-of-odometry-in-python/

#init locations
# loc01 = Location("loc01", "Storage #1", -0.5, -2.5, 1.57)
# loc02 = Location("loc02", "Assembly station #1", 4.5, 2.5, 3.1415/2.0)
# loc03 = Location("loc03", "Storage #2", -2.0, 0.0, 3.1415)
# loc04 = Location("loc04", "Assembly station #2", -5.0, 4.5, 6.283)

#objects from this class are added to the list where we choose the closest mex to a location
class Distance:
    def __init__(self, mex_id, distance):
        self.id = mex_id
        self.dist = distance

#function to get list of all mex from service provided by mex_sentinel
def call_get_mex_list():
    try:
        rospy.wait_for_service('/mex_sentinel/get_mex_list', rospy.Duration(1))     # Wait for service but with 1 sec timeout
        try:
            call_service = rospy.ServiceProxy('/mex_sentinel/get_mex_list', GetMexList)
            call = GetMexListRequest()
            result = call_service(call)
            return result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    except rospy.ROSException:
        pass

#function to calculate path to a location using (!) /move_base/make_plan on a certain mex
def get_plan(x_start, y_start, x_finish, y_finish, mex_id):
    start = PoseStamped() #current mex pose (will be defined by amcl)
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = x_start
    start.pose.position.y = y_start

    Goal = PoseStamped() #location to go
    Goal.header.seq = 0
    Goal.header.frame_id = "map"
    Goal.header.stamp = rospy.Time(0)
    Goal.pose.position.x = x_finish
    Goal.pose.position.y = y_finish

    srv = GetPlan()
    srv.start = start
    srv.goal = Goal
    srv.tolerance = 1.5

    get_plan = rospy.ServiceProxy('/'+str(mex_id)+'/move_base/make_plan', GetPlan)
    req = GetPlan()
    req.start = start
    req.goal = Goal
    req.tolerance = .5
    resp = get_plan(req.start, req.goal, req.tolerance)
    return resp

#function to calculate euclidian destance for each pair of poses from the mex path
def calculate_euclidian_distance(plan):
    global path_length
    path_length = 0
    for i in range(len(plan.poses) - 1):
        position_a_x = plan.poses[i].pose.position.x
        position_b_x = plan.poses[i+1].pose.position.x
        position_a_y = plan.poses[i].pose.position.y
        position_b_y = plan.poses[i+1].pose.position.y

        path_length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2))
    return path_length

#main function which call all other functions
def choose_closest_mex(location):
    mex_list_from_service = call_get_mex_list().mex_list #make list with all mex
    mex_list_for_calculating_distance = [] #empty list for mex filtered by STANDBY status
    for i in mex_list_from_service: #filtering mex by STANDBY status
        if i.status == str(MExStatus.STANDBY.name):
            mex_list_for_calculating_distance.append(i.id)
    distances = [] #list for Distance classes to contain distance and mex id for each mex
    for i in mex_list_for_calculating_distance: #calculating distance for each mex in STANDBY status
        current_pose = rospy.wait_for_message('/'+ str(i)+'/amcl_pose', PoseWithCovarianceStamped) #get mex current location
        plan = get_plan(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, location.x, location.y, str(i)) #get path from current location to destination
        res = calculate_euclidian_distance(plan.plan) #calculate euclidian distance based on the path
        distances.append(Distance(str(i), res)) #add distance and mex id to the list
    the_closest_id = None
    the_closest_dist = np.inf #number which is bigger than possible distance
    for i in distances: #choosing closest distance 
        if i.dist < the_closest_dist:
            the_closest_dist = i.dist
            the_closest_id = i.id
    # result = ('The closest mex to ' + str(location.name) + ' is ' + the_closest_id + ' with distance:')
    result = (the_closest_id, the_closest_dist)
    print(result)
    # print(the_closest_dist)
    
    return result       # A tuple of closest MEx ID and it's distance.


# rospy.init_node('shortest_path_calculator')

# choose_closest_mex(loc01)
# choose_closest_mex(loc02)
# choose_closest_mex(loc03)
# choose_closest_mex(loc04)