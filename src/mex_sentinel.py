#! /usr/bin/env python

import rospy
from enum import Enum
from JobManager.Job import Job
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
import time
from simple_sim.srv import AssignJobToMex, AssignJobToMexResponse, UnassignJobFromMex, UnassignJobFromMexResponse, ChangeMexStatus, ChangeMexStatusResponse, GetMexStatus, GetMexStatusResponse, GetMexList, GetMexListResponse
from simple_sim.msg import MexInfo, MexListInfo
from std_msgs.msg import String
from JobManager.MobileExecutor import MExStatus, MobileExecutor


#--------------------MEX initialization--------------------------
rdg01 = MobileExecutor('rdg01')
#rdg01.mex_info()

rdg02 = MobileExecutor('rdg02')
#rdg02.mex_info()

rdg03 = MobileExecutor('rdg03')
#rdg03.mex_info()

mex_list = [rdg01, rdg02, rdg03]
#---------------------------------------------------------------

#service to provide all mex with status
def get_mex_list(request):
    response = GetMexListResponse() #initialize mex list and bool success to return
    for i in mex_list: #making new list of MexInfo objects by copying each mex from original mex_info list
        mex = MexInfo()
        mex.id = i.id
        mex.status = str(i.status.name) #decide between name and value
        mex.job_id = str(i.job_id)
        response.mex_list.append(mex) #add current object to the response list
    if len(response.mex_list) == len(mex_list):
        response.success = True
    else:
        response.success = False
    return response

def assign_job(request):
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
    flag = None
    for i in mex_list:
        if i.id == request.mex_id:
            i.status = MExStatus(request.mex_new_status)
            #i.status = request.mex_new_status
            flag = True
    if flag == True:
        response = ChangeMexStatusResponse()
        response.success = True
    else:
        response = ChangeMexStatusResponse()
        response.success = False
    return response

def mex_list_info():
    pub = rospy.Publisher('~mex_list_info', MexListInfo, queue_size=10)
    rate = rospy.Rate(1) # 30hz
    while not rospy.is_shutdown():
        mexlistinfo = MexListInfo()
        mexlistinfo.stamp = rospy.Time.now()
        mexlistinfo.total_mex_number = len(mex_list)
        for i in mex_list:
            mexlistinfo.mex_list_info_array.append(i.mex_info())        
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(mexlistinfo)
        rate.sleep()


rospy.init_node('mex_sentinel')

get_mex_list_service = rospy.Service('~get_mex_list', GetMexList, get_mex_list) # create the Service called my_service with the defined callback

assign_job_to_mex_service = rospy.Service('~assign_job_to_mex', AssignJobToMex, assign_job)

unassign_job_from_mex_service = rospy.Service('~unassign_job_from_mex', UnassignJobFromMex, unassign_job)

get_mex_status_service = rospy.Service('~get_mex_status', GetMexStatus, get_mex_status)

change_mex_status_service = rospy.Service('~change_mex_status', ChangeMexStatus, change_mex_status)

mex_list_info()

rospy.spin() # maintain the service open.