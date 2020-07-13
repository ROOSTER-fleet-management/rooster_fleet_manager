#! /usr/bin/env python

import rospy

from Tasks import TaskStatus
from MobileExecutor import MExStatus, MobileExecutor

class JobStatus:
    """ Class that acts as Enumerator for Job status. """
    PENDING = 0
    ASSIGNED = 1
    ACTIVE = 2
    SUCCEEDED = 3
    ABORTED = 4

class Job:
    """ Class which contains overall job details and a list of job-tasks for individual jobs. """
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
                    # TODO: Remove itself from the Active Jobs list and free up the MEx, send update to MEx Sentinel to update MEx status.
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