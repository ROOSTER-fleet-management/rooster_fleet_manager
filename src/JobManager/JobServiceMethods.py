#! /usr/bin/env python

import rospy

from simple_sim.srv import GetMexListRequest, GetMexList, AssignJobToMex, AssignJobToMexRequest, \
    ChangeMexStatus, ChangeMexStatusRequest, UnassignJobFromMex, UnassignJobFromMexRequest
from MobileExecutor import MExStatus, MobileExecutor

def call_get_mex_list():
    """ Function to get most recent list of all MEx from service provided by mex_sentinel node."""
    rospy.wait_for_service('/mex_sentinel/get_mex_list')
    try:
        get_mex_list_service = rospy.ServiceProxy('/mex_sentinel/get_mex_list', GetMexList)
        req = GetMexListRequest()
        result = get_mex_list_service(req)
        mex_list = []
        for mex in result.mex_list:
            mex_id = mex.id
            mex_status = MExStatus[mex.status]
            mex_job_id = mex.job_id 
            mex_list.append(MobileExecutor(mex_id, mex_status, mex_job_id))
        return mex_list
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_assign_job(job_id, mex_id):
    """ Function to send an update to the MEx Sentinel to assign a Job to an MEx. """
    rospy.wait_for_service('/mex_sentinel/assign_job_to_mex')
    try:
        assign_job = rospy.ServiceProxy('mex_sentinel/assign_job_to_mex', AssignJobToMex)
        req = AssignJobToMexRequest()
        req.job_id = job_id
        req.mex_id = mex_id
        result = assign_job(req)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_unassign_job(mex_id):
    """ Function to send an update to the MEx Sentinel to unassign a MEx, no matter what the Job is. """
    rospy.wait_for_service('/mex_sentinel/unassign_job_from_mex')
    try:
        unassign_job = rospy.ServiceProxy('mex_sentinel/unassign_job_from_mex', UnassignJobFromMex)
        req = UnassignJobFromMexRequest()
        req.mex_id = mex_id
        result = unassign_job(req)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_change_mex_status(mex_id, status):
    """ Function to send an update to the MEx Sentinel to update an MEx's status. """
    rospy.wait_for_service('/mex_sentinel/change_mex_status')
    try:
        change_mex_status = rospy.ServiceProxy('mex_sentinel/change_mex_status', ChangeMexStatus)
        req = ChangeMexStatusRequest()
        req.mex_id = mex_id
        req.mex_new_status = status.value
        result = change_mex_status(req)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)