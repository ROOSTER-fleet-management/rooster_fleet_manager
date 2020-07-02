#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_msgs.msg import UInt8

class RobotMoveBase:
    """
    Item class: RobotMobeBase, implements move_base action calls to robot navigation stack.
    Used by the higher level Task class to populate a list with its task items.
    """
    def __init__(self, location):
        self.id = None              # ID of the robot to perform the move_base on
        self.location = location    # location of the goal of the move_base.

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
        """
        callback_status = 0             # 0 = PENDING, 1 = ACTIVE, 2 = CANCELLED, 3 = SUCCEEDED, 4 = ABORTED
        if status == 3:
            callback_status = 3
            rospy.loginfo(self.id + ". Goal reached")
        if status == 2 or status == 8:
            callback_status = 2
            rospy.loginfo(self.id + ". Goal cancelled")
        if status == 4:
            callback_status = 4
            rospy.loginfo(self.id + ". Goal aborted")
        
        if self.task_callback:
            self.task_callback([self.item_id, callback_status])
    #endregion

    def start(self, robot_id, item_id, task_callback):
        """
        Start the item's specific action.
        All task items should have this method: 'start', with these
        arguments: 'self', 'robot_id', 'item_id', 'task_callback'.
        """
        self.id = robot_id
        self.item_id = item_id
        self.task_callback = task_callback
        self.move_robot()

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
        """ Retrieve the status of the move_base action. """
        if self.navclient:
            return self.navclient.get_state()
        else:
            return None     # Return None if the navclient was not initialized yet.

class AwaitingLoadCompletion:
    """
    Item class: AwaitingLoadCompletion, waits for input from user or system to mark loading of the robot as succeeded, cancelled, aborted.
    Used by the higher level Task class to populate a list with its task items.
    """
    def __init__(self):
        self.id = None      # ID of the robot awaiting loading.
        self.status = 0     # 0 = PENDING, 1 = ACTIVE, 2 = CANCELLED, 3 = SUCCEEDED, 4 = ABORTED

    def start(self, robot_id, item_id, task_callback):
        """
        Start the item's specific action.
        All task items should have this method: 'start', with these
        arguments: 'self', 'robot_id', 'item_id', 'task_callback'.
        """
        self.id = robot_id
        self.item_id = item_id
        self.task_callback = task_callback
        self.status = 1
        self.input_subcriber = rospy.Subscriber(self.id + "/LoadInput", UInt8, self.input_cb)	# Subscribe to /'robot_id'/LoadInput topic to listen for published user/system input.
        rospy.loginfo(self.id + ". Awaiting load completion input...")
    
    def input_cb(self, data):
        """
        Callback method for any user or system input.
        Updates the instance status and calls the higher level task_callback.
        """
        if self.task_callback:      # Only process callback if this item was started.
            # Input received from user/system,
            if data.data == 3:
                # Loading was completed succesfully.
                self.status = 3
            elif data.data == 2:
                # Loading was cancelled by user.
                self.status = 2
            elif data.data == 4:
                # Loading encountered an error and had to abort.
                self.status = 4
            
            if data.data == 2 or data.data == 3 or data.data == 4:   # User input meaning some kind of end: cancel, succes or abort.
                self.input_subcriber.unregister()   # Unsubscribe to topic, as this item of the task is done.

            self.task_callback([self.item_id, self.status])     # Call the higher level Task callback.
    
    def get_status(self):
        """ Retrieve the status of the 'AwaitingLoadCompletion'  item. """
        return self.status