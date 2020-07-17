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

from simple_sim.srv import Order, OrderResponse

# - Retrieve all robot ids (e.g. their namespace names) from the rosparameter server -
# - Note: This parameter only exsists if the robots were launched using the multi_robot_sim GUI! -
# - Note: Launching robots using the multi_robot_sim GUI is the preferred method! -
robot_namespaces = rospy.get_param("/robot_list")
print("Robot list: ")
for robot in robot_namespaces:
    print(robot)

#region Service callback definitions

# - Order service callback -
def order_service_cb(request):
    print("Order service has been called with: " + str(request))
    # print("Arguments: ", request.order_args, "List size: ", len(request.order_args) )
    order_response = OrderResponse()

    # NOTE RUDIMENTARY ORDER PROCESSING, RIGID IN NATURE. 
    if int(request.order_args[0]) == OrderKeyword.TRANSPORT: # Keyword: TRANSPORT.
        # Expecting: [transport, priority, from_location, to_location]
        if len(request.order_args) == OrderTypeArgCount.TRANSPORT:
            priority = int(request.order_args[1])   # Check if it's >= 1 and <= 4.
            from_loc = request.order_args[2]        # Check if it's in the known locations dictionary
            to_loc = request.order_args[3]          # Check if it's in the known locations dictionary
            if (priority >= 1 and priority <= 4) and (from_loc in location_dict) and (to_loc in location_dict):
                # Succesful check on keyword and arguments, add order to order_list.
                order_response.status = OrderResponseStatus.SUCCES
                order_response.error_report = ""
                order_list.append([int(request.order_args[0]), priority, from_loc, to_loc])
            else:
                # Error occured, set status to ERROR and supply erorr report.
                order_response.status = OrderResponseStatus.ERROR
                order_response.error_report = \
                    "[TRANSPORT] Invalid priority (" + str(not (priority >= 1 and priority <= 4)) + \
                    ") or location argument (" + \
                    str(not ((from_loc in location_dict) and (to_loc in location_dict))) + \
                    ")."
        else:
            # Error occured, set status to ERROR and supply erorr report.
            order_response.status = OrderResponseStatus.ERROR
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
                order_response.status = OrderResponseStatus.SUCCES
                order_response.error_report = ""
                order_list.append([int(request.order_args[0]), priority, to_loc])
            else:
                # Error occured, set status to ERROR and supply erorr report.
                order_response.status = OrderResponseStatus.ERROR
                order_response.error_report = "[MOVE] Invalid priority (" + \
                    str(not (priority >= 1 and priority <= 4)) + \
                    ") or location argument (" + \
                    str(not (to_loc in location_dict) ) + ")."
        else:
            # Error occured, set status to ERROR and supply erorr report.
            order_response.status = OrderResponseStatus.ERROR
            order_response.error_report = "[MOVE] Invalid number of arguments, expected " + \
                str(OrderTypeArgCount.MOVE) + \
                ", received " + str(len(request.order_args))
    else:
        # Error occured, set status to ERROR and supply erorr report.
        order_response.status = OrderResponseStatus.ERROR   # Could not interpret order keyword.
        order_response.error_report = "Invalid keyword."

    # print("Order service is done, current order_list: " + str(order_list))
    return order_response # the service Response class, in this case OrderResponse


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
        order_service = rospy.Service('~order', Order , order_service_cb) # Create the Service called order with the defined callback

        # Initialize timers for the JobBuilder, and JobAllocator.
        rospy.Timer(rospy.Duration(1), order_list_timer_cb)     # Every second check order list to try build rough jobs.
        rospy.Timer(rospy.Duration(10), job_allocator_timer_cb)  # Call the job_allocator in 10 seconds to try allocating jobs. 
        # The above job_allocator timer should be replaced with something more elegant...

        # Keep node running.
        rospy.spin()

    except rospy.ROSInterruptException:
        pass