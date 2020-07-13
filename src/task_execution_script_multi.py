#! /usr/bin/env python


############# ! NOTE ! NOTE ! NOTE ! NOTE ! NOTE ! NOTE !NOTE ! NOTE ! ####################
#
# This file will be removed soon and is replaced by job_manager.py
#
###########################################################################################


import json         # Used for reading JSON files (loading jobs to JobQueue)
import os           # Used to get base filename and file and directory handling

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from task_class import TaskStatus, TaskType, RobotMoveBase, AwaitingLoadCompletion

#region #################### TODOLIST #########################
# DONE 1. Add option to remove job from JobQueue.
# DONE 2. Add method which will assign job to available MEx:
# DONE 2a. Keep track of available MExs.
# DONE 2b. Check when a MEx is available for a job
# DONE 2c. Assign job to selected MEx and start the job, change this MEx's status to 3 (EXECUTING_TASK).
# DONE 3. Deal with pre-assinged Jobs
# DONE 4. Set up Job Dispatcher (starting assigned jobs)
# TODO 5. Refactor code into modules with classes/methods.
#endregion ####################################################




# - Retrieve all robot ids (e.g. their namespace names) from the rosparameter server -
# - Note: This parameter only exsists if the robots were launched using the multi_robot_sim GUI! -
# - Note: Launching robots using the multi_robot_sim GUI is the preferred method! -
robot_namespaces = rospy.get_param("/robot_list")
print("Robot list: ")
for robot in robot_namespaces:
    print(robot)

class JobStatus:
    """ Class that acts as Enumerator for Job status. """
    PENDING = 0
    ASSIGNED = 1
    ACTIVE = 2
    SUCCEEDED = 3
    ABORTED = 4

class MExStatus:
    """ Class that acts as Enumerator for Mobile Executor (MEx) status. """
    STANDBY = 0
    CHARGING = 1
    ASSIGNED = 2
    EXECUTING_TASK = 3
    ERROR = 4

class MobileExecutor:
    """ Class with Mobile Executor (MEx) information (MEx id, status, assigned job id) """
    def __init__(self, id, status=MExStatus.STANDBY, job_id=None):
        self.id = id                # Unique identifier for this MEx, e.g. "rdg01"
        self.status = status        # Status of the MEx (STANDBY, CHARGING, ASSIGNED, EXECUTING_TASK, ERROR)
        self.job_id = job_id        # The unique id of the job the MEx is assigned to.


class Location:
    """ Class with location information (name, position, orientation) based on map reference frame. """
    def __init__(self, id, name, x_coordinate, y_coordinate, theta):
        self.id = id                # Unique identifier for this location, e.g. "loc01"
        self.name = name            # Name of the location, string
        self.x = x_coordinate       # X position in map frame in meters, float
        self.y = y_coordinate       # Y position in map frame in meters, float
        self.theta = theta          # Orientation (yaw angle) in map frame in radians, float

    def info(self):
        print(
            "Location info [" + self.name + "]: x, y, theta = " + str(self.x) + 
            ", " + str(self.y) + ", " + str(self.theta))

class Job:
    """ Class which contains overal job details and a list of job-tasks for individual jobs. """
    def __init__(self, job_id, mex_id=None):
        self.id = job_id                                # Unique identifier for this job, e.g. "job001"
        self.mex_id = mex_id                            # Unique identifier for an existing Mobile Executor (MEx), e.g. "rdg01"
        self.status = JobStatus.PENDING                 # PENDING, ASSIGNED, ACTIVE, SUCCEEDED, ABORTED
        self.task_count = 0                             # Current number of tasks for this job in the Job's task_list
        self.task_current = None                        # The task currently active.
        self.task_list = []                             # List of tasks for this Job.

    def add_task(self, task):
        """ Add a single task to the job's task list and update task count. """
        self.task_list.append(task)
        self.task_count = len(self.task_list)

    def add_tasks(self, list_of_tasks):
        """ Add a multiple tasks from a list to the job's task list and update task count. """
        for task in list_of_tasks:
            self.task_list.append(task)
        self.task_count = len(self.task_list)

    def assign_mex(self, mex_id):
        """ Assign a mex_id to this job, if status is either pending or assigned. """
        if self.status == JobStatus.PENDING or self.status == JobStatus.ASSIGNED:
            self.mex_id = mex_id
            self.status = JobStatus.ASSIGNED

    def start_job(self):
        """ Start executing the first task in the job's task_list if the status is ASSIGNED. """
        if self.status == JobStatus.ASSIGNED:
            self.status = JobStatus.ACTIVE        # Set status to active
            self.task_current = 0   # Set the current task to the 1st task in the list
            self.task_list[self.task_current].start(self.mex_id, self.task_current, self.task_cb)
        else:
            rospy.loginfo(self.id + " Job status is not equal to assigned; cannot start the job.")
    
    def next_task(self):
        """ Start executing the next task in the job's task_list, if the status is still ACTIVE. """
        if self.status == JobStatus.ACTIVE:
            self.task_current += 1
            self.task_list[self.task_current].start(self.mex_id, self.task_current, self.task_cb)

    def task_cb(self, data):
        """ Callback method for the tasks in the job's task list to call upon completion/cancellation/abort. """
        print(self.id + ". Task cb: " + str(data))
        task_id = data[0]
        task_status = data[1]
        if task_id == self.task_current:
            # task_status can be:    PENDING, ACTIVE, CANCELLED, SUCCEEDED, ABORTED

            if task_status == TaskStatus.SUCCEEDED:
                # Succesful completion of the task.
                if self.task_current + 1 < len(self.task_list):
                    # Continue with next.
                    self.next_task()
                else:
                    # End of the job's task_list reached. Job is complete.
                    self.status = JobStatus.SUCCEEDED
                    self.info()
            elif task_status == TaskStatus.CANCELLED or task_status == TaskStatus.ABORTED:
                # The task was cancelled/aborted, update job status
                self.status = JobStatus.ABORTED
            elif task_status == TaskStatus.ACTIVE:
                # The task is still active, don't do anything.
                pass

        else:
            rospy.logwarn("Mismatch between task callback ID and job's current task.")

    def info(self):
        print(
            "Job info [" + str(self.id) + "]: status = " + str(self.status) + 
            ", mex_id = " + str(self.mex_id) + ", tasks = " + str(self.task_count) + 
            ", current task = " + str(self.task_current)) #+ "\nTask list = " + str(self.task_list))

class JobQueue:
    """ Class which contains a lists of Job instances (pending or active). """
    def __init__(self):
        self.pending_job_list = []
        self.active_job_list = []

    def add_job_from_dict(self, job_dict):
        """ Add a Job to the JobQueue from a dictionary containing a Job's specifications. """
        job_id = job_dict["job_id"]
        mex_id = job_dict["mex_id"]
        # status = job_dict["status"]
        task_list = []

        # Task order matters, so don't just loop over the dictionary, but check size and loop over task id's
        task_list_length = len(job_dict["task_list"])
        for task_id in range(task_list_length):
            task_dict = job_dict["task_list"][str(task_id)]
            # Add task based on defined task type.
            if task_dict["task_type"] == TaskType.ROBOTMOVEBASE:
                location = location_dict[task_dict["location_id"]]
                task = RobotMoveBase(location)
            elif task_dict["task_type"] == TaskType.AWAITINGLOADCOMPLETION:
                task = AwaitingLoadCompletion()

            task_list.append(task)
        
        # Generated Job instance with specifications and add to the job list.
        job = Job(job_id, mex_id)
        job.add_tasks(task_list)
        self.pending_job_list.append(job)

    def add_jobs_from_dict(self, jobs_dict):
        """ Add multiple Jobs to the JobQueue from a dictionary containing multiple Jobs each with its specifications. """
        # Job order matters, so don't just loop over the dictionary, but check size and loop over job id's
        jobs_dict_length = len(jobs_dict)
        for job_id in range(jobs_dict_length):
            job_dict = jobs_dict[str(job_id)]
            self.add_job_from_dict(job_dict)

    def add_jobs_from_JSON(self, filepath):
        """ Add one or multiple jobs to the JobQueue from a JSON file. """
        # Load JSON file into dictionary
        loaddata_dict = None
        with open(filepath) as json_loadfile:
            loaddata_dict = json.load(json_loadfile)
        
        # Add jobs from loaded JSON dictionary to the JobQueue
        self.add_jobs_from_dict(loaddata_dict)
    
    def remove_pending_job(self, job_id):
        index_for_removal = None
        for index, job in enumerate(self.pending_job_list):
            if job.id == job_id:
                index_for_removal = index
                break

        if index_for_removal:
            self.pending_job_list.pop(index_for_removal)
            # TODO Handle the case when job was actually being executed/active at this point.
        else:
            rospy.loginfo("Cannot find job with id "+job_id+" in JobQueue's pending_job_list.")

    def allocate_pending_job(self):
        """
        Loops through the JobQueue,
        finds the first pending Job and matches it with an available MEx.
        If succesful, it removes this Job from the pending jobs list and returns
        the allocated Job instance, otherwise returns None.
        """
        allocated_job_index = None
        for index, job in enumerate(self.pending_job_list):
            if allocated_job_index != None:
                print("Allocated index: " + str(allocated_job_index))
                break
            elif job.status == JobStatus.PENDING:
                print("Trying for index: " + str(index))
                # Found a job which is still pending, now find available MEx.
                for mex in mex_list:
                    if mex.status == MExStatus.STANDBY:
                        # Found a MEx which is available (standby)
                        # Check if the job has not got a MEx pre-assigned OR the pre-assigned mex_id matches the MEx's id
                        if (not job.mex_id) or (job.mex_id == mex.id): 
                            mex.status = MExStatus.ASSIGNED
                            mex.job_id = job.id
                            job.assign_mex(mex.id)
                            allocated_job_index = index
                            break
        
        if allocated_job_index != None:
            return self.pending_job_list.pop(allocated_job_index)
        else:
            return None
    
    def dispatch_job(self, job):
        """ Dispatch the provided job; starting the job and adding it to the Active Jobs list. """
        job.start_job()
        for mex in mex_list:
            if mex.id == job.mex_id and mex.job_id == job.id:
                mex.status = MExStatus.EXECUTING_TASK
        self.active_job_list.append(job)


if __name__ == '__main__':
    try:
        # Retrieve robots and set up a list of available MobileExecutor (MEx) instances
        mex_list = []
        for robot in robot_namespaces:
            mex_list.append(MobileExecutor(robot))
        
        # Initialize the node
        rospy.init_node('job_manager')

        # Testing the adding of multiple Location class instances, storing them in a dictionary.
        location_dict = {
            "loc01" : Location("loc01", "Storage #1", -0.5, -2.5, 1.57),

            "loc02" : Location("loc02", "Assembly station #1", 4.5, 2.5, 3.1415/2.0),

            "loc03" : Location("loc03", "Storage #2", -2.0, 0.0, 3.1415),

            "loc04" : Location("loc04", "Assembly station #2", -5.0, 4.5, 6.283)
        }

        job_queue = JobQueue()    # Create the JobQueue
        print("JobQueue's list BEFORE job assignment from dict: " +str(job_queue.pending_job_list))  # JobQueue is empty.

        # Set up a single job in a dictionary to be added to the JobQueue.
        example_job_dict = {
            "job_id" : "job001",
            "mex_id" : None,
            #"status" : 0,
            "task_list" : {
                "0" : {
                    "task_id" : 0,
                    "task_type" : TaskType.ROBOTMOVEBASE,
                    "location_id" : "loc01"
                },
                "1" : {
                    "task_id" : 1,
                    "task_type" : TaskType.AWAITINGLOADCOMPLETION
                }
            }
        }
        job_queue.add_job_from_dict(example_job_dict)    # Adding job from the dictionary to the JobQueue.
        print("JobQueue's list AFTER job assignment from dict: " +str(job_queue.pending_job_list))
        print("JobQueue's first job: ")
        job_queue.pending_job_list[0].info()
        print("JobQueue's first job's first task: " + str(job_queue.pending_job_list[0].task_list[0]))

        # Setting up multiple jobs in a dictionary to be added to the JobQueue at once.
        example_multiple_jobs_dict = {
            "0" : {
                "job_id" : "job002",
                "mex_id" : "rdg02",
                #"status" : 0,
                "task_list" : {
                    "0" : {
                        "task_id" : 0,
                        "task_type" : TaskType.ROBOTMOVEBASE,
                        "location_id" : "loc03"
                    },
                    "1" : {
                        "task_id" : 1,
                        "task_type" : TaskType.AWAITINGLOADCOMPLETION
                    },
                    "2" : {
                        "task_id" : 2,
                        "task_type" : TaskType.ROBOTMOVEBASE,
                        "location_id" : "loc04"
                    },
                    "3" : {
                        "task_id" : 3,
                        "task_type" : TaskType.ROBOTMOVEBASE,
                        "location_id" : "loc03"
                    }
                }
            },
            "1" : {
                "job_id" : "job003",
                "mex_id" : None,
                #"status" : 0,
                "task_list" : {
                    "0" : {
                        "task_id" : 0,
                        "task_type" : TaskType.ROBOTMOVEBASE,
                        "location_id" : "loc02"
                    },
                    "1" : {
                        "task_id" : 1,
                        "task_type" : TaskType.AWAITINGLOADCOMPLETION
                    },
                    "2" : {
                        "task_id" : 2,
                        "task_type" : TaskType.ROBOTMOVEBASE,
                        "location_id" : "loc01"
                    }
                }
            }
        }
        job_queue.add_jobs_from_dict(example_multiple_jobs_dict)     # Adding multiple jobs from the dictionary to the JobQueue.
        print("JobQueue's length after multiple jobs from dict: " + str(len(job_queue.pending_job_list)))

        filepath = os.getcwd()+"/"+"example_jobs_json.JSON"
        print(filepath)
        job_queue.add_jobs_from_JSON(filepath)            # Adding one or multiple jobs from JSON file to the JobQueue.
        print("JobQueue's length after jobs JSON: " + str(len(job_queue.pending_job_list)))

        print("\nInfo on all pending Jobs in the JobQueue:")
        for job in job_queue.pending_job_list:
            job.info()
        
        job_queue.remove_pending_job("job005")       # Remove job with id "job005" from the JobQueue.
        print("\nJobQueue's length after removing 'job005': " + str(len(job_queue.pending_job_list)))

        print("Before allocation, first job, first MEx.")
        job_queue.pending_job_list[0].info()
        print(mex_list[0].id, mex_list[0].status)

        tmp_job = job_queue.allocate_pending_job()           # Allocate/assign the first job to the first available MEx.
        print(tmp_job)
        if tmp_job != None:
            job_queue.dispatch_job(tmp_job)                  # Dispatch the allocated job, starting it.

        print("After allocation, first job, first MEx.")
        job_queue.pending_job_list[0].info()  # Check if the first job in the list is now allocated.
        print(mex_list[0].id, mex_list[0].status)
        
        tmp_job = job_queue.allocate_pending_job()           # Try allocating next job in queue.
        print(tmp_job)
        if tmp_job != None:
            job_queue.dispatch_job(tmp_job)                  # Dispatch the allocated job, starting it.

        print("\nInfo on all pending Jobs in the JobQueue:")
        for job in job_queue.pending_job_list:
            job.info()

        print("\nInfo on all active Jobs in the JobQueue:")
        for job in job_queue.active_job_list:
            job.info()

        # # Testing the adding of multiple Job class instances and their functionality.    OLD CODE FOR TEMPORARY REFERENCE
        # job_1 = Job("job001")
        # job_1.add_task(RobotMoveBase(location_1))      # Tasks can be added 1 by 1
        # job_1.add_task(AwaitingLoadCompletion())
        # list_of_tasks = [RobotMoveBase(location_2), RobotMoveBase(location_1), RobotMoveBase(location_2)]
        # job_1.add_tasks(list_of_tasks)                 # Or multiple at a time in a list.
        # job_1.info()

        # job_2 = Job("job002", "rdg02")               # MEx id can be assigned immediately or left blank and assigned later.
        # list_of_tasks = [RobotMoveBase(location_3), AwaitingLoadCompletion(), RobotMoveBase(location_4), RobotMoveBase(location_3), RobotMoveBase(location_4)]
        # job_2.add_tasks(list_of_tasks)
        # job_2.info()

        # job_1.start_job()             # Try starting job even though mex_id is not assigned. It should loginfo.
        # job_1.assign_mex("rdg01")     # Assigning a mex_id to the job
        # job_1.start_job()             # Try starting the job again. It should succeed this time.
        # job_1.info()

        # job_2.start_job()             # In parallel call another MEx to perform a job.
        # job_2.info()

        # Spin the node while jobs are being executing, while receiving messages, callbacks etc.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass