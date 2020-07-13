#! /usr/bin/env python

from Job import Job
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
    rough_job = Job("job"+index)

    # TODO Replace the below hardcoded tasks with actual order interpretation and rough job creation!
    rough_job.add_task(AwaitingLoadCompletion())
    rough_job.add_task(RobotMoveBase(location_dict["loc01"]))

    pending_jobs_list.append(rough_job)