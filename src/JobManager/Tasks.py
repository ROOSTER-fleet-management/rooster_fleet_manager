#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_msgs.msg import UInt8
from enum import Enum

#region ################## TODOLIST ########################
# DONE 1. Add Task class and make other task classes inherit from it.
#endregion #################################################

class TaskType(Enum):
    """Class that acts as an enum for the different kinds of tasks."""
    ROBOTMOVEBASE = 0
    AWAITINGLOADCOMPLETION = 1
    AWAITINGUNLOADCOMPLETION = 2

class TaskStatus(Enum):
    """ Class that acts as Enumerator for Task status. """
    # 0 = PENDING, 1 = ACTIVE, 2 = CANCELLED, 3 = SUCCEEDED, 4 = ABORTED
    PENDING = 0
    ACTIVE = 1
    CANCELLED = 2
    SUCCEEDED = 3
    ABORTED = 4

class Task(object):
    """
    Base class from which specific child task classes inherit.
    """
    def __init__(self, tasktype, child_start):
        self.id = None                          # ID of the MEx to perform the task on/with
        self.status = TaskStatus.PENDING        # Status of this task.
        self.type = tasktype                    # Type of the task, defined by child.
        self.child_start = child_start          # The child's task specific start method.
    
    def start(self, mex_id, task_id, job_callback):
        """
        Start the task's specific action.
        All job tasks should have this method (through inheritance): 'start', with these
        arguments: 'self', 'mex_id', 'task_id', 'job_callback'.
        """
        self.id = mex_id
        self.task_id = task_id
        self.job_callback = job_callback
        self.status = TaskStatus.ACTIVE
        self.child_start()

    def get_status(self):
        """ Return the status of the task. """
        return self.status


class RobotMoveBase(Task):
    """
    Task class: RobotMoveBase, implements move_base action calls to robot navigation stack.
    Used by the higher level Job class to populate a list with its job tasks.
    """
    def __init__(self, location):
        super(RobotMoveBase, self).__init__(TaskType.ROBOTMOVEBASE, self.move_robot)
        self.location = location            # location of the goal of the move_base.

    #region Callback definitions
    def active_cb(self):
        """
        Callback for starting move_base action.
        Connected to actionlib send_goal call.
        """
        rospy.loginfo(self.id + ". Goal pose being processed")

    def feedback_cb(self, feedback):
        """
        Callback for continuous feedback of move_base position.
        Connected to actionlib send_goal call.
        """
        pass    # Don't spam the console for now..
        # rospy.loginfo("Current location: "+str(feedback))

    def done_cb(self, status, result):
        """
        Callback for stopping of goal.
        Connected to actionlib send_goal call.
        move_base callback status options: PENDING=0, ACTIVE=1, PREEMPTED=2, SUCCEEDED=3,
        ABORTED=4, REJECTED=5, PREEMPTING=6, RECALLING=7, RECALLED=8, LOST=9.
        """
        if status == 3:
            self.status = TaskStatus.SUCCEEDED
            rospy.loginfo(self.id + ". Goal reached")
        if status == 2 or status == 8:
            self.status = TaskStatus.CANCELLED
            rospy.loginfo(self.id + ". Goal cancelled")
        if status == 4:
            self.status = TaskStatus.ABORTED
            rospy.loginfo(self.id + ". Goal aborted")
        
        if self.job_callback:
            self.job_callback([self.task_id, self.status])
    #endregion

    def move_robot(self):
        """ Start a move_base action using actionlib. """
        self.navclient = actionlib.SimpleActionClient(self.id + '/move_base',MoveBaseAction)
        self.navclient.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.location.x
        goal.target_pose.pose.position.y = self.location.y
        goal.target_pose.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0, 0, self.location.theta)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.navclient.send_goal(goal, done_cb=self.done_cb, active_cb=self.active_cb, feedback_cb=self.feedback_cb)
    
    def get_status(self):
        """ 
        Overwrites base class get_status, but inherits using super().
        Retrieve the status of the move_base action. 
        """
        if self.navclient:
            # navclient state options: PENDING=0, ACTIVE=1, PREEMPTED=2, SUCCEEDED=3,
            # ABORTED=4, REJECTED=5, PREEMPTING=6, RECALLING=7, RECALLED=8, LOST=9.
            navclient_state = self.navclient.get_state()

            if navclient_state == 0 or navclient_state == 1:
                self.status = TaskStatus.ACTIVE
            elif navclient_state == 2 or navclient_state == 5 or navclient_state == 8:
                self.status = TaskStatus.CANCELLED
            elif navclient_state == 3:
                self.status = TaskStatus.SUCCEEDED
            elif navclient_state == 4 or navclient_state == 9:
                self.status = TaskStatus.ACTIVE
        
        super(RobotMoveBase, self).get_status()

class AwaitingLoadCompletion(Task):
    """
    Task class: AwaitingLoadCompletion, waits for input from user or system to mark loading of the MEx as succeeded, cancelled, aborted.
    Used by the higher level Job class to populate a list with its job tasks.
    """
    def __init__(self):
        super(AwaitingLoadCompletion, self).__init__(TaskType.AWAITINGLOADCOMPLETION, self.child_start)

    def child_start(self):
        """ Start the task's specific action, subscribing to the /LoadInput topic on the MEx's namespace. """
        self.input_subcriber = rospy.Subscriber(self.id + "/LoadInput", UInt8, self.input_cb)	# Subscribe to /'mex_id'/LoadInput topic to listen for published user/system input.
        rospy.loginfo(self.id + ". Awaiting load completion input...")
    
    def input_cb(self, data):
        """
        Callback method for any user or system input.
        Updates the instance status and calls the higher level job_callback.
        load status option: PENDING=0 ACTIVE=1, CANCELLED=2, SUCCEEDED=3,
        ABORTED=4.
        """
        if self.job_callback:      # Only process callback if this task was started.
            # Input received from user/system,
            if data.data == 3:
                # Loading was completed succesfully.
                self.status = TaskStatus.SUCCEEDED
            elif data.data == 2:
                # Loading was cancelled by user.
                self.status = TaskStatus.CANCELLED
            elif data.data == 4:
                # Loading encountered an error and had to abort.
                self.status = TaskStatus.ABORTED
            
            if data.data == 2 or data.data == 3 or data.data == 4:   # User input meaning some kind of end: cancel, succes or abort.
                self.input_subcriber.unregister()   # Unsubscribe to topic, as this task of the job is done.

            self.job_callback([self.task_id, self.status])     # Call the higher level Job callback.

class AwaitingUnloadCompletion(AwaitingLoadCompletion):
    """
    Task class: AwaitingUnloadCompletion, waits for input from user or system to mark unloading of the MEx as succeeded, cancelled, aborted.
    Used by the higher level Job class to populate a list with its job tasks.
    Inherets from AwaitingLoadCompletion.
    """
    def __init__(self):
        super(AwaitingUnloadCompletion, self).__init__()
        self.type = TaskType.AWAITINGUNLOADCOMPLETION
    
    def child_start(self):
        """ Start the task's specific action, subscribing to the /UnloadInput topic on the MEx's namespace. """
        self.input_subcriber = rospy.Subscriber(self.id + "/UnloadInput", UInt8, self.input_cb) # Subscribe to /'mex_id'/UnloadInput topic to listen for published user/system input.
        rospy.loginfo(self.id + ". Awaiting unload completion input...")