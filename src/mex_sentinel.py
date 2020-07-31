#! /usr/bin/env python

import rospy
from simple_sim.srv import AssignJobToMex, AssignJobToMexResponse, UnassignJobFromMex, UnassignJobFromMexResponse, ChangeMexStatus, ChangeMexStatusResponse, GetMexStatus, GetMexStatusResponse, GetMexList, GetMexListResponse
from simple_sim.msg import MexInfo, MexListInfo
from JobManager.MobileExecutor import MExStatus, MobileExecutor

# Make mex_list global
mex_list = []

def mobile_executor_initialization():
    """
    Function which is called once to initialize MobileExecutors
    and add them the MEx list based on the ROS parameter server.
    """
    # Read robot_list from ROS parameter server. Robotnames are strings.
    robot_namespaces = rospy.get_param("/robot_list")
    # Go through the robot list, extract each name and create MobileExecutor object with that name, then append to the MEx list.
    for i in robot_namespaces:
        robot_name = i 
        robot_instance = MobileExecutor(robot_name)
        mex_list.append(robot_instance)

#service to provide all mex with status
def get_mex_list(request):
    """
    GetMexList service callback function. 
    Takes in a empty request.
    Returns a GetMexListResponse response with 
    a list of MexInfo objects for each MExs and
    a flag if the call was handled succesfully.
    """
    response = GetMexListResponse() # Initialize mex list and bool success to return.

    # Making new list of MexInfo objects by copying each mex from original mex_info list.
    for i in mex_list:
        mex = MexInfo()
        mex.id = i.id
        mex.status = str(i.status.name) # Return the status as the Enum's name (string)
        mex.job_id = str(i.job_id)
        response.mex_list.append(mex) # Add current object to the response list.
    if len(response.mex_list) == len(mex_list):
        response.success = True
    else:
        response.success = False
    return response

def assign_job(request):
    """
    AssignJob service callback function.
    Takes in a MEx ID and Job ID and assigns the Job to the MEx,
    updating it's information.
    Returns a flag if the call was handled succesfully.
    """
    assignment = False #flag
    for i in mex_list:
        if i.id == request.mex_id:
            i.job_id = request.job_id
            i.status = MExStatus.ASSIGNED
            assignment = True
    if assignment == True:
        response = AssignJobToMexResponse()
        response.success = True
    else:
        response = AssignJobToMexResponse()
        response.success = False
    return response

def unassign_job(request):
    """
    UnassignJob service callback function.
    Takes in a MEx ID and Job ID and unassigns the Job from the MEx,
    updating it's information.
    Returns a flag if the call was handled succesfully.
    """
    unassignment = False #flag
    for i in mex_list:
        if i.id == request.mex_id:
            i.job_id = None
            i.status = MExStatus.STANDBY
            unassignment = True
    if unassignment == True:
        response = UnassignJobFromMexResponse()
        response.success = True
    else:
        response = UnassignJobFromMexResponse()
        response.success = False
    return response

#callback function for service that return status of a certain MEx
def get_mex_status(request):
    """
    GetMexStatus service callback function.
    Takes in a MEx ID. 
    Returns a response with the MEx status and Job ID, and a 
    flag if the call was handled succesfully.
    """
    flag = False #flag that is used for indicating successful search of MEx in MEx list
    response = GetMexStatusResponse() #create response object 
    for i in mex_list: #search for MEx in MEx list
        if i.id == request.mex_id:
            response.mex_status = i.status.name #MEx status query 'name' method from enum object returns text
            response.job_id = str(i.job_id)
            flag = True
    if flag == True: #if we found requested Mex in the list, together with status and job id we return bool success 
        response.success = True
        return response
    else:
        response.success = False
        return response

def change_mex_status(request):
    """
    ChangeMexStatus service callback function.
    Takes in a MEx ID and status.
    Updates status of the MobileExecutor in the MEx list matching
    the provided ID.
    Returns a response with a flag if the call was handled succesfully.
    """
    flag = None
    for i in mex_list:
        if i.id == request.mex_id:
            i.status = MExStatus(request.mex_new_status)
            flag = True
    if flag == True:
        response = ChangeMexStatusResponse()
        response.success = True
    else:
        response = ChangeMexStatusResponse()
        response.success = False
    return response

def mex_list_info():
    """
    MexListInfo publish function. 
    Sets up a published on the local /mex_list_info topic, publishing
    information on all MobileExecutors in the MEx list at a rate of 1 Hz.
    """
    pub = rospy.Publisher('~mex_list_info', MexListInfo, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        # Loop here, publishing the MexListInfo message with a list of MexInfo objects at the specified rate.
        mexlistinfo = MexListInfo()
        mexlistinfo.stamp = rospy.Time.now()
        mexlistinfo.total_mex_number = len(mex_list)
        for i in mex_list:
            mex_info = MexInfo()
            mex_info.status = i.status.name
            mex_info.id = i.id
            mex_info.job_id = str(i.job_id)
            mexlistinfo.mex_list_info_array.append(mex_info)

        pub.publish(mexlistinfo)
        rate.sleep()


if __name__ == '__main__':
    try:
        # Initialize mobile executors
        mobile_executor_initialization()

        # Initialize node.
        rospy.init_node('mex_sentinel')

        # Initialize services.
        get_mex_list_service = rospy.Service('~get_mex_list', GetMexList, get_mex_list)
        assign_job_to_mex_service = rospy.Service('~assign_job_to_mex', AssignJobToMex, assign_job)
        unassign_job_from_mex_service = rospy.Service('~unassign_job_from_mex', UnassignJobFromMex, unassign_job)
        get_mex_status_service = rospy.Service('~get_mex_status', GetMexStatus, get_mex_status)
        change_mex_status_service = rospy.Service('~change_mex_status', ChangeMexStatus, change_mex_status)

        # Initialise mex list publisher 
        mex_list_info()

        # Maintain the services open.
        rospy.spin()
    except rospy.ROSInterruptException:
        # Handle InterruptExceptions without crashing.
        pass