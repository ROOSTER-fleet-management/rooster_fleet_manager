#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from task_item_class import RobotMoveBase, AwaitingLoadCompletion

class Location:
    """ Class with location information (name, position, orientation) based on map reference frame. """
    def __init__(self, name, x_coordinate, y_coordinate, theta):
        self.name = name            # Name of the location, string
        self.x = x_coordinate       # X position in map frame in meters, float
        self.y = y_coordinate       # Y position in map frame in meters, float
        self.theta = theta          # Orientation (yaw angle) in map frame in radians, float

    def info(self):
        print(
            "Location info [" + self.name + "]: x, y, theta = " + str(self.x) + 
            ", " + str(self.y) + ", " + str(self.theta))

class Task:
    """ Class which contains individual overal task details and a list of task-items. """
    def __init__(self, task_id, robot_id=None):
        self.id = task_id
        self.robot_id = robot_id
        self.status = 0 if not self.robot_id else 1     # 0 = PENDING, 1 = ASSIGNED, 2 = ACTIVE, 3 = SUCCEEDED, 4 = ABORTED
        self.item_count = 0
        self.item_current = None
        self.item_list = []

    def add_item(self, item):
        """ Add a single item to the task's item list and update item count. """
        self.item_list.append(item)
        self.item_count = len(self.item_list)

    def add_items(self, list_of_items):
        """ Add a multiple items from a list to the task's item list and update item count. """
        for item in list_of_items:
            self.item_list.append(item)
        self.item_count = len(self.item_list)

    def assign_robot(self, robot_id):
        """ Assign a robot_id to this task, if status is either pending or assigned. """
        if self.status == 0 or self.status == 1:
            self.robot_id = robot_id
            self.status = 1

    def start_task(self):
        """ Start executing the first item in the task's item_list if the status = 1 (ASSIGNED). """
        if self.status == 1:
            self.status = 2         # Set status to active
            self.item_current = 0   # Set the current item to the 1st item in the list
            self.item_list[self.item_current].start(self.robot_id, self.item_current, self.item_cb)
        else:
            rospy.loginfo(self.id + " Task status is not equal to assigned; cannot start the task.")
    
    def next_item(self):
        """ Start executing the next item in the task's item_list, if the status is still 2 (ACTIVE). """
        if self.status == 2:
            self.item_current += 1
            self.item_list[self.item_current].start(self.robot_id, self.item_current, self.item_cb)

    def item_cb(self, data):
        """ Callback method for the items in the task's item list to call upon completion/cancellation/abort. """
        print("Item cb: ", data)
        item_id = data[0]
        item_status = data[1]
        if item_id == self.item_current:
            # item_status can be:    0 = PENDING, 1 = ACTIVE, 2 = CANCELLED, 3 = SUCCEEDED, 4 = ABORTED

            if item_status == 3:
                # Succesful completion of the item.
                if self.item_current + 1 < len(self.item_list):
                    # Continue with next.
                    self.next_item()
                else:
                    # End of the task's item_list reached. Task is complete.
                    self.status = 3
                    self.info()

        else:
            rospy.loginfo("Mismatch between item callback ID and task's current item.")

    def info(self):
        print(
            "Task info [" + str(self.id) + "]: status = " + str(self.status) + 
            ", robot_id = " + str(self.robot_id) + ", items = " + str(self.item_count) + 
            ", current item = " + str(self.item_current)) #+ "\nItem list = " + str(self.item_list))

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('simple_fleet_manager')

        # Testing the adding of multiple Location class instances
        location_1 = Location('Storage #1', -0.5, -2.5, 1.57)
        # location_1.info()

        location_2 = Location('Assembly station #1', 4.5, 2.5, 3.1415/2.0)
        # location_2.info()

        location_3 = Location("Storage #2", -2.0, 0.0, 3.1415)
        # location_3.info()

        location_4 = Location("Assembly station #2", -5.0, 4.5, 6.283)
        # location_4.info()

        # Testing the adding of multiple Task class instances and their functionality.
        task_1 = Task("task001")
        task_1.add_item(RobotMoveBase(location_1))      # Items can be added 1 by 1
        task_1.add_item(AwaitingLoadCompletion())
        list_of_items = [RobotMoveBase(location_2), RobotMoveBase(location_1), RobotMoveBase(location_2)]
        task_1.add_items(list_of_items)                 # Or multiple at a time in a list.
        task_1.info()

        task_2 = Task("task002", "rdg02")               # Robot id can be assigned immediately or left blank and assigned later.
        list_of_items = [RobotMoveBase(location_3), AwaitingLoadCompletion(), RobotMoveBase(location_4), RobotMoveBase(location_3), RobotMoveBase(location_4)]
        task_2.add_items(list_of_items)
        task_2.info()

        task_1.start_task()             # Try starting task even though robot_id is not assigned. It should loginfo.
        task_1.assign_robot("rdg01")    # Assigning a robot_id to the task
        task_1.start_task()             # Try starting the task again. It should succeed this time.
        task_1.info()

        task_2.start_task()             # In parallel call another robot to perform a task.
        task_2.info()

        # Spin the node while tasks are being executing, while receiving messages, callbacks etc.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass