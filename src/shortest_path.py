#! /usr/bin/env python

import rospy
from simple_sim.srv import GetMexList, GetMexListRequest
from JobManager.MobileExecutor import MExStatus
from nav_msgs.srv import GetPlan, GetPlanResponse
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from JobManager.Location import Location
#done call service get mex list. Filter mex in STANDBY

#get coordinates of each mex. 3 ways: (1) by odometry and (2) by tf from /map to /base_link and (3) by amcl_pose topic
#(1)https://www.theconstructsim.com/ros-qa-know-pose-robot-python/
#(2)https://answers.ros.org/question/341062/how-to-get-a-robot-position-xy-in-a-map-instead-of-odometry-in-python/

# call /move_base/make_plan of each mex



#calculate Euclidian distance for each plan


loc01 = Location("loc01", "Storage #1", -0.5, -2.5, 1.57)
loc02 = Location("loc02", "Assembly station #1", 4.5, 2.5, 3.1415/2.0)
loc03 = Location("loc03", "Storage #2", -2.0, 0.0, 3.1415)
loc04 = Location("loc04", "Assembly station #2", -5.0, 4.5, 6.283)


def call_get_mex_list():
    rospy.wait_for_service('/mex_sentinel/get_mex_list')
    try:
        call_service = rospy.ServiceProxy('/mex_sentinel/get_mex_list', GetMexList)
        call = GetMexListRequest()
        result = call_service(call)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_make_plan(x_start, y_start, x_finish, y_finish, mex_id):
    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = x_start
    start.pose.position.y = y_start

    Goal = PoseStamped()
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

rospy.init_node('shortest_path_calculator')
current_pose = rospy.wait_for_message('/rdg01/amcl_pose', PoseWithCovarianceStamped)

plan = call_make_plan(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, loc01.x, loc01.y, 'rdg01')
print(plan)
#print(current_pose.pose.pose.position.x)
#print(current_pose.pose.pose.position.y)


mex_list_from_service = call_get_mex_list().mex_list
mex_list_for_calculating_distance = []
#print(type(mex_list_from_service[1].status))
for i in mex_list_from_service:
    if i.status == str(MExStatus.STANDBY.name):
        mex_list_for_calculating_distance.append(i.id)
#print(mex_list_for_calculating_distance)
"""if __name__ == '__main__':
    rospy.init_node('move_turtlebot', )
    try:
        odom = myNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')"""


#if __name__ == "__main__":