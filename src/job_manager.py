#! /usr/bin/env python
#import json         # Used for reading JSON files (loading jobs to JobQueue)
#import os           # Used to get base filename and file and directory handling

"""
TO DO: ENTER THE DESCRIPTION OF THIS MODULE/ROS NODE HERE (IN THE SOURCE CODE DOC STRING)!!!
"""

import rospy
#import actionlib

#from JobManager.Tasks import TaskStatus, TaskType, RobotMoveBase, AwaitingLoadCompletion
from JobManager.Job import JobStatus, Job, JobPriority
from JobManager.Location import Location, make_location_dict
from JobManager.JobBuilder import job_builder
from JobManager.JobActivation import job_allocator, job_refiner
from JobManager.MobileExecutor import MExStatus, MobileExecutor
from JobManager.Order import *
from JobManager.JobServiceMethods import call_get_mex_list, call_unassign_job

from simple_sim.srv import PlaceOrder, PlaceOrderResponse, GetPendingJobs, GetPendingJobsResponse, \
    GetActiveJobs, GetActiveJobsResponse, GetJobInfo, GetJobInfoResponse, \
    GetMexList, GetMexListResponse, GetMexListRequest, AssignJobToMex, AssignJobToMexRequest
from simple_sim.msg import PendingJob, ActiveJob, JobInfo, TaskInfo

NODE_NAME = "[job_manager] "

#region Service callback definitions

# - PlaceOrder service callback -
def order_service_cb(request):
    """
    PlaceOrder service callback function.
    Takes in a PlaceOrderRequest request with a order keyword, priority and arguments.
    Processes these and adds the processed order in list form to the order_list.
    Returns a PlaceOrderResponse response with information on if the call failed and why.
    """
    print(NODE_NAME + "Order service has been called with: " + str(request))
    # print(NODE_NAME + "Arguments: ", request.order_args, "List size: ", len(request.order_args) )
    order_response = PlaceOrderResponse()

    # NOTE RUDIMENTARY ORDER PROCESSING, RIGID IN NATURE. 
    if request.keyword == OrderKeyword.TRANSPORT.name: # Keyword: TRANSPORT.
        # Expecting: transport, priority, [from_location, to_location]
        if len(request.order_args) == OrderTypeArgCount.TRANSPORT.value:
            priority = JobPriority[request.priority].value   # Check if it's >= 1 and <= 4.
            from_loc = request.order_args[0]        # Check if it's in the known locations dictionary
            to_loc = request.order_args[1]          # Check if it's in the known locations dictionary
            if (priority >= 1 and priority <= 4) and (from_loc in location_dict) and (to_loc in location_dict):
                # Succesful check on keyword and arguments, add order to order_list.
                order_response.error_status = OrderResponseStatus.SUCCES.name
                order_response.error_report = ""
                order_list.append([request.keyword, JobPriority(priority), from_loc, to_loc])
            else:
                # Error occured, set status to ERROR and supply erorr report.
                order_response.error_status = OrderResponseStatus.ERROR.name
                order_response.error_report = \
                    "[TRANSPORT] Invalid priority (" + str(not (priority >= 1 and priority <= 4)) + \
                    ") or location argument (" + \
                    str(not ((from_loc in location_dict) and (to_loc in location_dict))) + \
                    ")."
        else:
            # Error occured, set status to ERROR and supply erorr report.
            order_response.error_status = OrderResponseStatus.ERROR.name
            order_response.error_report = "[TRANSPORT] Invalid number of arguments, expected " + \
                str(OrderTypeArgCount.TRANSPORT.value) + \
                ", received " + str(len(request.order_args))
        
    elif request.keyword == OrderKeyword.MOVE.name:    # Keyword: MOVE.
        # Expecting: move, priority, [to_location]
        if len(request.order_args) == OrderTypeArgCount.MOVE.value:
            priority = JobPriority[request.priority].value   # Check if it's >= 1 and <= 4.
            to_loc = request.order_args[0]          # Check if it's in the known locations dictionary
            if (priority >= 1 and priority <= 4) and (to_loc in location_dict):
                # Succesful check on keyword and arguments, add order to order_list.
                order_response.error_status = OrderResponseStatus.SUCCES.name
                order_response.error_report = ""
                order_list.append([request.keyword, JobPriority(priority), to_loc])
            else:
                # Error occured, set status to ERROR and supply error report.
                order_response.error_status = OrderResponseStatus.ERROR.name
                order_response.error_report = "[MOVE] Invalid priority (" + \
                    str(not (priority >= 1 and priority <= 4)) + \
                    ") or location argument (" + \
                    str(not (to_loc in location_dict) ) + ")."
        else:
            # Error occured, set status to ERROR and supply erorr report.
            order_response.error_status = OrderResponseStatus.ERROR.name
            order_response.error_report = "[MOVE] Invalid number of arguments, expected " + \
                str(OrderTypeArgCount.MOVE.value) + \
                ", received " + str(len(request.order_args))

    elif request.keyword == OrderKeyword.LOAD.name:     # Keyword: LOAD
        # Expecting: load, priority, []
        if len(request.order_args) == OrderTypeArgCount.LOAD.value:
            priority = JobPriority[request.priority].value  # Check if it's >= 1 and <= 4.
            if (priority >= 1 and priority <= 4):
                # Succesful check on keyword and arguments, add order to order_list.
                order_response.error_status = OrderResponseStatus.SUCCES.name
                order_response.error_report = ""
                order_list.append([request.keyword, JobPriority(priority)])
            else:
                # Error occured, set status to ERROR and supply error report.
                order_response.error_status = OrderResponseStatus.ERROR.name
                order_response.error_report = "[LOAD] Invalid priority."
        else:
            # Error occured, set status to ERROR and supply error report.
            order_response.error_status = OrderResponseStatus.ERROR.name
            order_response.error_report = "[LOAD] Invalid number of arguments, expected " + \
                str(OrderTypeArgCount.LOAD.value) + \
                ", received " + str(len(request.order_args))

    elif request.keyword == OrderKeyword.UNLOAD.name:     # Keyword: UNLOAD
        # Expecting: load, priority, []
        if len(request.order_args) == OrderTypeArgCount.UNLOAD.value:
            priority = JobPriority[request.priority].value  # Check if it's >= 1 and <= 4.
            if (priority >= 1 and priority <= 4):
                # Succesful check on keyword and arguments, add order to order_list.
                order_response.error_status = OrderResponseStatus.SUCCES.name
                order_response.error_report = ""
                order_list.append([request.keyword, JobPriority(priority)])
            else:
                # Error occured, set status to ERROR and supply error report.
                order_response.error_status = OrderResponseStatus.ERROR.name
                order_response.error_report = "[LOAD] Invalid priority."
        else:
            # Error occured, set status to ERROR and supply error report.
            order_response.error_status = OrderResponseStatus.ERROR.name
            order_response.error_report = "[LOAD] Invalid number of arguments, expected " + \
                str(OrderTypeArgCount.UNLOAD.value) + \
                ", received " + str(len(request.order_args))
    else:
        # Error occured, set status to ERROR and supply erorr report.
        order_response.error_status = OrderResponseStatus.ERROR.name
        order_response.error_report = "Invalid keyword."    # Could not interpret order keyword.

    # print(NODE_NAME + "Order service is done, current order_list: " + str(order_list))
    return order_response # the service Response class, in this case PlaceOrderResponse

# - GetPendingJobs service callback -
def get_pending_jobs_service_cb(request):
    """
    GetPendingJobs service callback function.
    Takes in a empty request.
    Returns a response with a list of all Pending Jobs and some basic information.
    """
    # print(NODE_NAME + "A service request for the Pending Jobs List has been received.")
    pending_jobs_response = GetPendingJobsResponse()
    pending_jobs_response.jobs_count = len(pending_job_list)
    for job in pending_job_list:
        pending_job = PendingJob()
        pending_job.priority = job.priority.name
        pending_job.job_id = job.id
        pending_job.task_count = job.task_count
        pending_job.keyword = job.keyword
        pending_jobs_response.jobs.append(pending_job)
    return pending_jobs_response

# - GetActiveJobs service callback -
def get_active_jobs_service_cb(request):
    """
    GetActiveJobs service callback function.
    Takes in a empty request.
    Returns a response with a list of all Active Jobs and all basic information.
    """
    # print(NODE_NAME + "A service request for the Active Jobs List has been received.")
    active_jobs_response = GetActiveJobsResponse()
    active_jobs_response.jobs_count = len(active_job_list)
    for job in active_job_list:
        active_job = ActiveJob()
        active_job.job_id = job.id
        active_job.status = job.status.name
        active_job.priority = job.priority.name
        active_job.mex_id = job.mex_id
        active_job.task_count = job.task_count
        active_job.current_task = job.task_current
        active_job.keyword = job.keyword
        active_jobs_response.jobs.append(active_job)
    return active_jobs_response

# - GetJobInfo service callback -
def get_job_info_service_cb(request):
    """
    GetJobInfo service callback function.
    Takes in a request with a Job ID.
    Returns a response with all information on the Job 
    matching the call ID, including a list of the Job's Tasks. 
    """
    job_id_to_find = request.job_id
    print(NODE_NAME + "A service request for the Get Job Info has been received for " + job_id_to_find + ".")
    job_info_response = GetJobInfoResponse()
    found_job = None
    for job in pending_job_list:
        if job.id == job_id_to_find:
            found_job = job
            break
    else:
        for job in active_job_list:
            if job.id == job_id_to_find:
                found_job = job
                break
        else:
            job_info_response.error_status = "ERROR"  # TODO Replace with error messages enum.
            job_info_response.error_report = "Could not find " + job_id_to_find + " in pending or active job lists."
    
    if found_job != None:
        job_information = JobInfo()
        job_information.job_id = found_job.id
        job_information.status = found_job.status.name
        job_information.priority = found_job.priority.name
        job_information.mex_id = found_job.mex_id if not found_job.mex_id == None else ""
        job_information.task_count = found_job.task_count 
        job_information.current_task = found_job.task_current if not found_job.task_current == None else 0
        job_information.keyword = found_job.keyword
        for task in found_job.task_list:
            task_info = TaskInfo()
            task_info.status = task.status.name
            task_info.type = task.type.name
            job_information.task_list.append(task_info)
        job_info_response.job_info = job_information
        job_info_response.error_status = "SUCCES"

    return job_info_response
#endregion

#region Timer callback definitons

# - Order list timer callback -
def order_list_timer_cb(event):
    """ Order list timer callback function. Calls process_order_list function. """
    #print(NODE_NAME + "Order list timer callback, processing order_list and building rough jobs.")
    process_order_list()

# - Job allocator timer callback -
def job_allocator_timer_cb(event):
    """ Job allocator timer callback function. Calls job allocator function. """
    if job_allocator(pending_jobs_list=pending_job_list, active_jobs_list=active_job_list, mexs_list=mex_list) != 0:
        # rospy.loginfo("Failed to allocate job.")
        pass

#endregion

#region Job completion callback definition
def job_completion_cb(job_id, mex_id):
    """
    Job completion callback function, gets called when a 
    Jobs is finished (succesful, cancel, error or abort).
    It is attached to a Job in the job_builder function.
    """
    print(NODE_NAME + "Job called the job_completion_cb function: " + str(job_id) + ", " + str(mex_id))
    # First send update to MEx Sentinel to unassign job.
    call_unassign_job(mex_id=mex_id)
    # Then call MEx Sentinel to provide latest MEx List.
    mex_list = call_get_mex_list()

    # Remove completed Job from the active_job_list. 
    index_to_pop = None
    for index, job in enumerate(active_job_list):
        if job.id == job_id:
            index_to_pop = index
            break
    else:
        # Couldn't find job_id in active_job_list... Handle this?
        pass
    if not index_to_pop == None:
        active_job_list.pop(index_to_pop)
#endregion

def process_order_list():
    """
    Build jobs from all orders in the order_list by calling 
    the job_builder function, then clear the order_list.
    """
    global job_index
    for order in order_list:
        job_index = job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index, location_dict=location_dict, completion_cb=job_completion_cb)
    del order_list[:]

if __name__ == '__main__':
    try:
        # Retrieve robots and set up a list of available MobileExecutor (MEx) instances
        mex_list = call_get_mex_list()
        # for robot in robot_namespaces:
        #     mex_list.append(MobileExecutor(robot))
        for mex in mex_list:
            print(NODE_NAME + str( (mex.id, mex.status, mex.job_id) ) )
        
        # Lists for Orders, PendingJobs, ActiveJobs:
        order_list = []
        pending_job_list = []
        active_job_list = []
        job_index = 1

        # Testing the adding of multiple Location class instances, storing them in a dictionary.
        # TODO Replace this with a dynamic(?) dictionary which is constructed from the MEx Sentinel.
        location_dict = make_location_dict()
        
        # Initialize the node.
        rospy.init_node('job_manager')

        # Initialize services.
        order_service = rospy.Service('~place_order', PlaceOrder , order_service_cb)
        get_pending_jobs_service = rospy.Service('~get_pending_jobs', GetPendingJobs, get_pending_jobs_service_cb)
        get_active_jobs_service = rospy.Service('~get_active_jobs', GetActiveJobs, get_active_jobs_service_cb)
        get_job_info_service = rospy.Service('~get_job_info', GetJobInfo, get_job_info_service_cb)

        # Initialize timers for the JobBuilder, and JobAllocator.
        rospy.Timer(rospy.Duration(1), order_list_timer_cb)     # Every second check order list to try build rough jobs.
        rospy.Timer(rospy.Duration(5), job_allocator_timer_cb)  # Call the job_allocator in 10 seconds to try allocating jobs. 
        # The above job_allocator timer should be replaced with something more elegant...

        # Keep node running.
        rospy.spin()

    except rospy.ROSInterruptException:
        pass