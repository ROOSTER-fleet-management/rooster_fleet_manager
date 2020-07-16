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

from simple_sim.srv import Order, OrderResponse

# - Retrieve all robot ids (e.g. their namespace names) from the rosparameter server -
# - Note: This parameter only exsists if the robots were launched using the multi_robot_sim GUI! -
# - Note: Launching robots using the multi_robot_sim GUI is the preferred method! -
robot_namespaces = rospy.get_param("/robot_list")
print("Robot list: ")
for robot in robot_namespaces:
    print(robot)

#region Service callback definitions

# - Order service -
def order_service_cb(request):
    print("Order service has been called with: " + str(request))
    i = 1
    while i < 11:
        print('Order service is executing '+str(i)+' seconds')
        rospy.sleep(1)
        i = i + 1
    print("Order service is done.")
    order_response = OrderResponse()
    order_response.status = 0   # 0 = SUCCES, 1 = FAILED. TODO Make this into an enum.
    return order_response # the service Response class, in this case OrderResponse

#endregion

if __name__ == '__main__':
    try:
        # Retrieve robots and set up a list of available MobileExecutor (MEx) instances
        mex_list = []
        for robot in robot_namespaces:
            mex_list.append(MobileExecutor(robot))
        
        #Lists for Orders, PendingJobs, ActiveJobs:
        order_list = []
        pending_job_list = []
        active_job_list = []
        job_index = 1
        
        # Initialize the node.
        rospy.init_node('job_manager')

        # Initialize services.
        order_service = rospy.Service('~order', Order , order_service_cb) # create the Service called order with the defined callback

        #region TESTING PACKAGE MODULES THROUGH HARDCODED ORDERS AND PRIORITIES
        # print("Pending:", pending_job_list, "Active: ", active_job_list)
        order = JobPriority.LOW      # order should not JUST be priority.
        # TODO Actually have an input order string(?). 
        # TODO Read from order_list instead. 
        job_index = job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index)
        job_index = job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index)
        job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index)
        order = JobPriority.MEDIUM
        job_index = job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index)
        order = JobPriority.HIGH
        job_index = job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index)
        order = JobPriority.MEDIUM
        job_index = job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index)
        job_index = job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index)
        order = JobPriority.LOW
        job_index = job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index)
        order = JobPriority.CRITICAL
        job_index = job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index)

        # for job in pending_job_list:
        #     print(job.id + ", " + str(job.priority))

        # print("Pending:", pending_job_list, "Active: ", active_job_list)
        if job_allocator(pending_jobs_list=pending_job_list, active_jobs_list=active_job_list, mexs_list=mex_list) != 0:
            rospy.loginfo("Failed to allocate job.")
        # print("Pending:", pending_job_list, "Active: ", active_job_list)

        #endregion

        # Keep node running.
        rospy.spin()

    except rospy.ROSInterruptException:
        pass