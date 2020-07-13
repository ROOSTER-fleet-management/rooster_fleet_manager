#! /usr/bin/env python
import json         # Used for reading JSON files (loading jobs to JobQueue)
import os           # Used to get base filename and file and directory handling

import rospy
import actionlib

from JobManager.Tasks import TaskStatus, TaskType, RobotMoveBase, AwaitingLoadCompletion
from JobManager.Job import JobStatus, Job
from JobManager.Location import Location
from JobManager.JobBuilder import job_builder
from JobManager.JobActivation import job_allocator, job_refiner
from JobManager.MobileExecutor import MExStatus, MobileExecutor

# - Retrieve all robot ids (e.g. their namespace names) from the rosparameter server -
# - Note: This parameter only exsists if the robots were launched using the multi_robot_sim GUI! -
# - Note: Launching robots using the multi_robot_sim GUI is the preferred method! -
robot_namespaces = rospy.get_param("/robot_list")
print("Robot list: ")
for robot in robot_namespaces:
    print(robot)


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

        #region TESTING PACKAGE MODULES
        print("Pending:", pending_job_list, "Active: ", active_job_list)
        order = None        
        # TODO Actually have an input order string(?). 
        # TODO Read from order_list instead. 
        job_builder(pending_jobs_list=pending_job_list, order=order, job_index=job_index)
        print("Pending:", pending_job_list, "Active: ", active_job_list)
        job_allocator(pending_jobs_list=pending_job_list, active_jobs_list=active_job_list, mexs_list=mex_list)
        print("Pending:", pending_job_list, "Active: ", active_job_list)

        #endregion

        # Keep node running.
        rospy.spin()

    except rospy.ROSInterruptException:
        pass