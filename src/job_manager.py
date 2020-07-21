#! /usr/bin/env python
import json         # Used for reading JSON files (loading jobs to JobQueue)
import os           # Used to get base filename and file and directory handling

import rospy
import actionlib

from JobManager.Tasks import TaskStatus, TaskType, RobotMoveBase, AwaitingLoadCompletion
from JobManager.Job import JobStatus, Job, JobPriority
from JobManager.Location import Location
from JobManager.JobBuilder import job_builder
from JobManager.JobActivation import job_allocator, job_refiner
from JobManager.MobileExecutor import MExStatus, MobileExecutor
from JobManager.Order import *

from simple_sim.srv import PlaceOrder, PlaceOrderResponse, GetPendingJobs, GetPendingJobsResponse, GetActiveJobs, GetActiveJobsResponse, GetJobInfo, GetJobInfoResponse
from simple_sim.msg import PendingJob, ActiveJob, JobInfo, TaskInfo

# - Retrieve all robot ids (e.g. their namespace names) from the rosparameter server -
# - Note: This parameter only exsists if the robots were launched using the multi_robot_sim GUI! -
# - Note: Launching robots using the multi_robot_sim GUI is the preferred method! -
robot_namespaces = rospy.get_param("/robot_list")
print("Robot list: ")
for robot in robot_namespaces:
    print(robot)

#region Service callback definitions

# - PlaceOrder service callback -
def order_service_cb(request):
    print("Order service has been called with: " + str(request))
    # print("Arguments: ", request.order_args, "List size: ", len(request.order_args) )
    order_response = PlaceOrderResponse()

    # NOTE RUDIMENTARY ORDER PROCESSING, RIGID IN NATURE. 
    if int(request.order_args[0]) == OrderKeyword.TRANSPORT: # Keyword: TRANSPORT.
        # Expecting: [transport, priority, from_location, to_location]
        if len(request.order_args) == OrderTypeArgCount.TRANSPORT:
            priority = int(request.order_args[1])   # Check if it's >= 1 and <= 4.
            from_loc = request.order_args[2]        # Check if it's in the known locations dictionary
            to_loc = request.order_args[3]          # Check if it's in the known locations dictionary
            if (priority >= 1 and priority <= 4) and (from_loc in location_dict) and (to_loc in location_dict):
                # Succesful check on keyword and arguments, add order to order_list.
                order_response.error_status = OrderResponseStatus.SUCCES
                order_response.error_report = ""
                order_list.append([int(request.order_args[0]), priority, from_loc, to_loc])
            else:
                # Error occured, set status to ERROR and supply erorr report.
                order_response.error_status = OrderResponseStatus.ERROR
                order_response.error_report = \
                    "[TRANSPORT] Invalid priority (" + str(not (priority >= 1 and priority <= 4)) + \
                    ") or location argument (" + \
                    str(not ((from_loc in location_dict) and (to_loc in location_dict))) + \
                    ")."
        else:
            # Error occured, set status to ERROR and supply erorr report.
            order_response.error_status = OrderResponseStatus.ERROR
            order_response.error_report = "[TRANSPORT] Invalid number of arguments, expected " + \
                str(OrderTypeArgCount.TRANSPORT) + \
                ", received " + str(len(request.order_args))
        
    elif int(request.order_args[0]) == OrderKeyword.MOVE:    # Keyword: MOVE.
        # Expecting: [move, priority, to_location]
        if len(request.order_args) == OrderTypeArgCount.MOVE:
            priority = int(request.order_args[1])   # Check if it's >= 1 and <= 4.
            to_loc = request.order_args[2]          # Check if it's in the known locations dictionary
            if (priority >= 1 and priority <= 4) and (to_loc in location_dict):
                # Succesful check on keyword and arguments, add order to order_list.
                order_response.error_status = OrderResponseStatus.SUCCES
                order_response.error_report = ""
                order_list.append([int(request.order_args[0]), priority, to_loc])
            else:
                # Error occured, set status to ERROR and supply erorr report.
                order_response.error_status = OrderResponseStatus.ERROR
                order_response.error_report = "[MOVE] Invalid priority (" + \
                    str(not (priority >= 1 and priority <= 4)) + \
                    ") or location argument (" + \
                    str(not (to_loc in location_dict) ) + ")."
        else:
            # Error occured, set status to ERROR and supply erorr report.
            order_response.error_status = OrderResponseStatus.ERROR
            order_response.error_report = "[MOVE] Invalid number of arguments, expected " + \
                str(OrderTypeArgCount.MOVE) + \
                ", received " + str(len(request.order_args))
    else:
        # Error occured, set status to ERROR and supply erorr report.
        order_response.error_status = OrderResponseStatus.ERROR   # Could not interpret order keyword.
        order_response.error_report = "Invalid keyword."

    # print("Order service is done, current order_list: " + str(order_list))
    return order_response # the service Response class, in this case PlaceOrderResponse

# - GetPendingJobs service callback -
def get_pending_jobs_service_cb(request):
    print("A service request for the Pending Jobs List has been received.")
    pending_jobs_response = GetPendingJobsResponse()
    pending_jobs_response.jobs_count = len(pending_job_list)
    for job in pending_job_list:
        pending_job = PendingJob()
        pending_job.priority = job.priority
        pending_job.job_id = job.id
        pending_job.task_count = job.task_count
        pending_jobs_response.jobs.append(pending_job)
    return pending_jobs_response

# - GetActiveJobs service callback -
def get_active_jobs_service_cb(request):
    print("A service request for the Active Jobs List has been received.")
    active_jobs_response = GetActiveJobsResponse()
    active_jobs_response.jobs_count = len(active_job_list)
    for job in active_job_list:
        active_job = ActiveJob()
        active_job.job_id = job.id
        active_job.status = job.status
        active_job.priority = job.priority
        active_job.mex_id = job.mex_id
        active_job.task_count = job.task_count
        active_job.current_task = job.task_current
        active_jobs_response.jobs.append(active_job)
    return active_jobs_response

def get_job_info_service_cb(request):
    job_id_to_find = request.job_id
    print("A service request for the Get Job Info has been received for " + job_id_to_find + ".")
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
            job_info_response.error_status = 1  # TODO Replace with error messages enum.
            job_info_response.error_report = "Could not find " + job_id_to_find + " in pending or active job lists."
    
    if found_job != None:
        job_information = JobInfo()
        job_information.job_id = found_job.id
        job_information.status = found_job.status
        job_information.priority = found_job.priority
        job_information.mex_id = found_job.mex_id if not found_job.mex_id == None else ""
        job_information.task_count = found_job.task_count 
        job_information.current_task = found_job.task_current if not found_job.task_current == None else 0
        for task in found_job.task_list:
            task_info = TaskInfo()
            task_info.status = task.status
            task_info.type = task.type
            job_information.task_list.append(task_info)
        job_info_response.job_info = job_information

    return job_info_response
#endregion

#region Timer callback definitons

# - Order list timer callback -
def order_list_timer_cb(event):
    # print("Order list timer callback, processing order_list and building rough jobs.")
    process_order_list()

# - Job allocator timer callback -
def job_allocator_timer_cb(event):
    if job_allocator(pending_jobs_list=pending_job_list, active_jobs_list=active_job_list, mexs_list=mex_list) != 0:
        rospy.loginfo("Failed to allocate job.")

#endregion

def process_order_list():
    # Build jobs from all orders in the order_list, then clear the order_list.
    global job_index
    for order in order_list:
        job_index = job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index, location_dict=location_dict)
    del order_list[:]


if __name__ == '__main__':
    try:
        # Retrieve robots and set up a list of available MobileExecutor (MEx) instances
        mex_list = []
        for robot in robot_namespaces:
            mex_list.append(MobileExecutor(robot))
        
        # Lists for Orders, PendingJobs, ActiveJobs:
        order_list = []
        pending_job_list = []
        active_job_list = []
        job_index = 1

        # Testing the adding of multiple Location class instances, storing them in a dictionary.
        # TODO Replace this with a dynamic(?) dictionary which is constructed from the sentinel? 
        #       Or which is loaded from a config file but can be adjusted on-line?
        location_dict = {
            "loc01" : Location("loc01", "Storage #1", -0.5, -2.5, 1.57),
            "loc02" : Location("loc02", "Assembly station #1", 4.5, 2.5, 3.1415/2.0),
            "loc03" : Location("loc03", "Storage #2", -2.0, 0.0, 3.1415),
            "loc04" : Location("loc04", "Assembly station #2", -5.0, 4.5, 6.283)
        }
        
        # Initialize the node.
        rospy.init_node('job_manager')

        # Initialize services.
        order_service = rospy.Service('~place_order', PlaceOrder , order_service_cb)
        get_pending_jobs_service = rospy.Service('~get_pending_jobs', GetPendingJobs, get_pending_jobs_service_cb)
        get_active_jobs_service = rospy.Service('~get_active_jobs', GetActiveJobs, get_active_jobs_service_cb)
        get_job_info_service = rospy.Service('~get_job_info', GetJobInfo, get_job_info_service_cb)

        # Initialize timers for the JobBuilder, and JobAllocator.
        rospy.Timer(rospy.Duration(1), order_list_timer_cb)     # Every second check order list to try build rough jobs.
        rospy.Timer(rospy.Duration(10), job_allocator_timer_cb)  # Call the job_allocator in 10 seconds to try allocating jobs. 
        # The above job_allocator timer should be replaced with something more elegant...

        # Keep node running.
        rospy.spin()

    except rospy.ROSInterruptException:
        pass