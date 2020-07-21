#! /usr/bin/env python

from enum import Enum

class MExStatus(Enum):
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
    def mex_info(self):
        #print('Mobile executor #' + str(self.id) + '\nStatus: ' + str(self.status.name) + '\nJob_id: ' + str(self.job_id) + '\n')
        #rospy.loginfo('Mobile executor #' + str(self.id) + '\nStatus: ' + str(self.status.name) + '\nJob_id: ' + str(self.job_id) + '\n')
        #mexinfo = str('Mobile executor #' + str(self.id) + '\nStatus: ' + str(self.status.name) + '\nJob_id: ' + str(self.job_id) + '\n')
        mexinfo = str('Mobile executor #' + str(self.id) + '; Status: ' + str(self.status.name) + '; Job_id: ' + str(self.job_id))
        return mexinfo