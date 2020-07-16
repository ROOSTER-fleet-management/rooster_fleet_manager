#! /usr/bin/env python

from Job import Job, JobPriority
from Tasks import *
from Location import Location

# Testing the adding of multiple Location class instances, storing them in a dictionary.
location_dict = {
    "loc01" : Location("loc01", "Storage #1", -0.5, -2.5, 1.57),
    "loc02" : Location("loc02", "Assembly station #1", 4.5, 2.5, 3.1415/2.0),
    "loc03" : Location("loc03", "Storage #2", -2.0, 0.0, 3.1415),
    "loc04" : Location("loc04", "Assembly station #2", -5.0, 4.5, 6.283)
}

# Job Builder
# Takes in order and creates a Job class instance containing Task class instances. Then adds the Job to the Pending Jobs list.
# Job Builder can probably be just (a couple of) methods, no need for a class here..?

def job_builder(pending_jobs_list, order, job_index):
    index = "00" + str(job_index) if job_index < 10 else ( "0" + str(job_index) if job_index < 100 else str(job_index) )
    rough_job = Job("job"+index, priority=order)

    # TODO Replace the below hardcoded tasks with actual order interpretation and rough job creation!
    rough_job.add_task(AwaitingLoadCompletion())
    rough_job.add_task(RobotMoveBase(location_dict["loc01"]))

    # Loop over the current list of Pending Jobs with index, find the last spot in the list  within the same priority section.
    for index, job in enumerate(pending_jobs_list):
        priority = job.priority
        if priority < rough_job.priority:
            print("Inserting Rough Job (" + rough_job.id + ") at position " + str(index))
            pending_jobs_list.insert(index, rough_job)
            break
    else:   # Looped over all jobs in the pending_jobs_list and it wasn't inserted, so just append to the end.
        print("Appending Rough Job (" + rough_job.id + ") to end")
        pending_jobs_list.append(rough_job)
        
    return job_index + 1