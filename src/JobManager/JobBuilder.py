#! /usr/bin/env python

from Job import Job, JobPriority
from Tasks import *
from Location import Location
from Order import *

def job_builder(pending_jobs_list, order, job_index, location_dict, completion_cb):
    """
    Job Builder
    Takes in order and creates a Job class instance containing Task 
    class instances. Then adds the Job to the Pending Jobs list.
    """
    index = "00" + str(job_index) if job_index < 10 else ( "0" + str(job_index) if job_index < 100 else str(job_index) )
    keyword = order[0]
    priority = order[1]
    rough_job = Job("job"+index, completion_cb=completion_cb, priority=priority)

    if keyword == OrderKeyword.TRANSPORT.name:
        # Transport order, consists of moving somewhere, getting loaded, moving somewhere, getting unloaded.
        from_loc = order[2]
        to_loc = order[3]
        rough_job.add_task(RobotMoveBase(location_dict[from_loc]))
        rough_job.add_task(AwaitingLoadCompletion())
        rough_job.add_task(RobotMoveBase(location_dict[to_loc]))
        rough_job.add_task(AwaitingUnloadCompletion())
    elif keyword == OrderKeyword.MOVE.name:
        # Move order, consists of moving somehwere.
        to_loc = order[2]
        rough_job.add_task(RobotMoveBase(location_dict[to_loc]))
    elif keyword == OrderKeyword.LOAD.name:
        # Load order on the spot.
        rough_job.add_task(AwaitingLoadCompletion())
    elif keyword == OrderKeyword.UNLOAD.name:
        # Unload order on the spot.
        rough_job.add_task(AwaitingUnloadCompletion())

    # Loop over the current list of Pending Jobs with index, find the last spot in the list within the same priority section.
    for index, job in enumerate(pending_jobs_list):
        priority = job.priority.value
        if priority < rough_job.priority.value:
            print("Inserting Rough Job (" + rough_job.id + ") at position " + str(index))
            pending_jobs_list.insert(index, rough_job)
            break
    else:   # Looped over all jobs in the pending_jobs_list and it wasn't inserted, so just append to the end.
        print("Appending Rough Job (" + rough_job.id + ") to end")
        pending_jobs_list.append(rough_job)
        
    return job_index + 1