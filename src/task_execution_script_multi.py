#! /usr/bin/env python
import json         # Used for reading JSON files (loading tasks to TaskQueue)
import os           # Used to get base filename and file and directory handling

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from task_item_class import ItemEnum, RobotMoveBase, AwaitingLoadCompletion

#region #################### TODOLIST #########################
# DONE 1. Add option to remove task from TaskQueue.
# DONE 2. Add method which will assign task to available robot:
# DONE 2a. Keep track of available robots.
# DONE 2b. Check when a robot is available for a task
# DONE 2c. Assign task to selected robot and start the task, change this robot's status to 3 (EXECUTING_TASK).
# DONE 3. Deal with pre-assinged Tasks
# DONE 4. Set up Task Dispatcher (starting assigned tasks)
# TODO 5. Set up Task Handler (deleting finished tasks from Active Tasks list and updating robot status)
#endregion ####################################################

# - Retrieve all robot ids (e.g. their namespace names) from the rosparameter server -
# - Note: This parameter only exsists if the robots were launched using the multi_robot_sim GUI! -
# - Note: Launching robots using the multi_robot_sim GUI is the preferred method! -
robot_namespaces = rospy.get_param("/robot_list")
print("Robot list: ")
for robot in robot_namespaces:
    print(robot)


class Robot:
    """ Class with robot information (robot id, status, assigned task id) """
    def __init__(self, id, status=0, task_id=None):
        self.id = id                # Unique identifier for this robot, e.g. "rdg01"
        self.status = status        # Status of the robot (0=STANDBY, 1=CHARGING, 2=ASSIGNED, 3=EXECUTING_TASK, 4=ERROR)
        self.task_id = task_id      # The unique id of the task the robot is assigned to.


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

class Task:
    """ Class which contains overal task details and a list of task-items for individual tasks. """
    def __init__(self, task_id, robot_id=None):
        self.id = task_id                               # Unique identifier for this task, e.g. "task001"
        self.robot_id = robot_id                        # Unique identifier for an existing robot, e.g. "rdg01"
        self.status = 0                                 # 0 = PENDING, 1 = ASSIGNED, 2 = ACTIVE, 3 = SUCCEEDED, 4 = ABORTED
        self.item_count = 0                             # Current number of items for this task in the Task's item_list
        self.item_current = None                        # The item currently active.
        self.item_list = []                             # List of items for this Task.

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
        print(self.id + ". Item cb: " + str(data))
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
            elif item_status == 2 or item_status == 4:
                # The item was cancelled/aborted, update task status
                self.status = 4
            elif item_status == 1:
                # The item is still active, don't do anything.
                pass

        else:
            rospy.loginfo("Mismatch between item callback ID and task's current item.")

    def info(self):
        print(
            "Task info [" + str(self.id) + "]: status = " + str(self.status) + 
            ", robot_id = " + str(self.robot_id) + ", items = " + str(self.item_count) + 
            ", current item = " + str(self.item_current)) #+ "\nItem list = " + str(self.item_list))

class TaskQueue:
    """ Class which contains a lists of Task instances (pending or active). """
    def __init__(self):
        self.pending_task_list = []
        self.active_task_list = []

    def add_task_from_dict(self, task_dict):
        """ Add a Task to the TaskQueue from a dictionary containing a Task's specifications. """
        task_id = task_dict["task_id"]
        robot_id = task_dict["robot_id"]
        # status = task_dict["status"]
        item_list = []

        # Item order matters, so don't just loop over the dictionary, but check size and loop over item id's
        item_list_length = len(task_dict["item_list"])
        for item_id in range(item_list_length):
            item_dict = task_dict["item_list"][str(item_id)]
            # Add item based on defined item type.
            if item_dict["item_type"] == ItemEnum.ROBOTMOVEBASE:
                location = location_dict[item_dict["location_id"]]
                item = RobotMoveBase(location)
            elif item_dict["item_type"] == ItemEnum.AWAITINGLOADCOMPLETION:
                item = AwaitingLoadCompletion()

            item_list.append(item)
        
        # Generated Task instance with specifications and add to the task list.
        task = Task(task_id, robot_id)
        task.add_items(item_list)
        self.pending_task_list.append(task)

    def add_tasks_from_dict(self, tasks_dict):
        """ Add multile Tasks to the TaskQueue from a dictionary containing multiple Tasks each with its specifications. """
        # Task order matters, so don't just loop over the dictionary, but check size and loop over task id's
        tasks_dict_length = len(tasks_dict)
        for task_id in range(tasks_dict_length):
            task_dict = tasks_dict[str(task_id)]
            self.add_task_from_dict(task_dict)

    def add_tasks_from_JSON(self, filepath):
        """ Add one or multiple tasks to the TaskQueue from a JSON file. """
        # Load JSON file into dictionary
        loaddata_dict = None
        with open(filepath) as json_loadfile:
            loaddata_dict = json.load(json_loadfile)
        
        # Add tasks from loaded JSON dictionary to the TaskQueue
        self.add_tasks_from_dict(loaddata_dict)
    
    def remove_pending_task(self, task_id):
        index_for_removal = None
        for index, task in enumerate(self.pending_task_list):
            if task.id == task_id:
                index_for_removal = index
                break

        if index_for_removal:
            self.pending_task_list.pop(index_for_removal)
            # TODO Handle the case when task was actually being executed/active at this point.
        else:
            rospy.loginfo("Cannot find task with id "+task_id+" in TaskQueue's pending_task_list.")

    def allocate_pending_task(self):
        """
        Loops through the TaskQueue,
        finds the first pending Task and matches it with an available Robot.
        If succesful, it removes this Task from the pending tasks list and returns
        the allocated Task instance, otherwise returns None.
        """
        allocated_task_index = None
        for index, task in enumerate(self.pending_task_list):
            if allocated_task_index != None:
                print("Allocated index: " + str(allocated_task_index))
                break
            elif task.status == 0:
                print("Trying for index: " + str(index))
                # Found a task which is still pending, now find available robot.
                for robot in robot_list:
                    if robot.status == 0:
                        # Found a robot which is available (standby)
                        # Check if the task has not got a robot pre-assigned OR the pre-assigned robot_id matches the robot's id
                        if (not task.robot_id) or (task.robot_id == robot.id): 
                            robot.status = 2
                            robot.task_id = task.id
                            task.assign_robot(robot.id)
                            allocated_task_index = index
                            break
        
        if allocated_task_index != None:
            return self.pending_task_list.pop(allocated_task_index)
        else:
            return None
    
    def dispatch_task(self, task):
        """ Dispatch the provided task; starting the task and adding it to the Active Tasks list. """
        task.start_task()
        for robot in robot_list:
            if robot.id == task.robot_id and robot.task_id == task.id:
                robot.status = 3
        self.active_task_list.append(task)


if __name__ == '__main__':
    try:
        # Retrieve robots and set up a list of available Robot instances
        robot_list = []
        for robot in robot_namespaces:
            robot_list.append(Robot(robot))
        
        # Initialize the node
        rospy.init_node('simple_fleet_manager')

        # Testing the adding of multiple Location class instances, storing them in a dictionary.
        location_dict = {
            "loc01" : Location("loc01", "Storage #1", -0.5, -2.5, 1.57),

            "loc02" : Location("loc02", "Assembly station #1", 4.5, 2.5, 3.1415/2.0),

            "loc03" : Location("loc03", "Storage #2", -2.0, 0.0, 3.1415),

            "loc04" : Location("loc04", "Assembly station #2", -5.0, 4.5, 6.283)
        }

        task_queue = TaskQueue()    # Create the TaskQueue
        print("TaskQueue's list BEFORE task assignment from dict: " +str(task_queue.pending_task_list))  # TaskQueue is empty.

        # Set up a single task in a dictionary to be added to the TaskQueue.
        example_task_dict = {
            "task_id" : "task001",
            "robot_id" : None,
            #"status" : 0,
            "item_list" : {
                "0" : {
                    "item_id" : 0,
                    "item_type" : ItemEnum.ROBOTMOVEBASE,
                    "location_id" : "loc01"
                },
                "1" : {
                    "item_id" : 1,
                    "item_type" : ItemEnum.AWAITINGLOADCOMPLETION
                }
            }
        }
        task_queue.add_task_from_dict(example_task_dict)    # Adding task from the dictionary to the TaskQueue.
        print("TaskQueue's list AFTER task assignment from dict: " +str(task_queue.pending_task_list))
        print("TaskQueue's first task: ")
        task_queue.pending_task_list[0].info()
        print("TaskQueue's first task's first item: " + str(task_queue.pending_task_list[0].item_list[0]))

        # Setting up multiple tasks in a dictionary to be added to the TaskQueue at once.
        example_multiple_tasks_dict = {
            "0" : {
                "task_id" : "task002",
                "robot_id" : "rdg02",
                #"status" : 0,
                "item_list" : {
                    "0" : {
                        "item_id" : 0,
                        "item_type" : ItemEnum.ROBOTMOVEBASE,
                        "location_id" : "loc03"
                    },
                    "1" : {
                        "item_id" : 1,
                        "item_type" : ItemEnum.AWAITINGLOADCOMPLETION
                    },
                    "2" : {
                        "item_id" : 2,
                        "item_type" : ItemEnum.ROBOTMOVEBASE,
                        "location_id" : "loc04"
                    },
                    "3" : {
                        "item_id" : 3,
                        "item_type" : ItemEnum.ROBOTMOVEBASE,
                        "location_id" : "loc03"
                    }
                }
            },
            "1" : {
                "task_id" : "task003",
                "robot_id" : None,
                #"status" : 0,
                "item_list" : {
                    "0" : {
                        "item_id" : 0,
                        "item_type" : ItemEnum.ROBOTMOVEBASE,
                        "location_id" : "loc02"
                    },
                    "1" : {
                        "item_id" : 1,
                        "item_type" : ItemEnum.AWAITINGLOADCOMPLETION
                    },
                    "2" : {
                        "item_id" : 2,
                        "item_type" : ItemEnum.ROBOTMOVEBASE,
                        "location_id" : "loc01"
                    }
                }
            }
        }
        task_queue.add_tasks_from_dict(example_multiple_tasks_dict)     # Adding multiple tasks from the dictionary to the TaskQueue.
        print("TaskQueue's length after multiple tasks from dict: " + str(len(task_queue.pending_task_list)))

        filepath = os.getcwd()+"/"+"example_tasks_json.JSON"
        print(filepath)
        task_queue.add_tasks_from_JSON(filepath)            # Adding one or multiple tasks from JSON file to the TaskQueue.
        print("TaskQueue's length after tasks JSON: " + str(len(task_queue.pending_task_list)))

        print("\nInfo on all pending Tasks in the TaskQueue:")
        for task in task_queue.pending_task_list:
            task.info()
        
        task_queue.remove_pending_task("task005")       # Remove task with id "task005" from the TaskQueue.
        print("\nTaskQueue's length after removing 'task005': " + str(len(task_queue.pending_task_list)))

        print("Before allocation, first task, first robot.")
        task_queue.pending_task_list[0].info()
        print(robot_list[0].id, robot_list[0].status)

        tmp_task = task_queue.allocate_pending_task()           # Allocate/assign the first task to the first available robot.
        print(tmp_task)
        if tmp_task != None:
            task_queue.dispatch_task(tmp_task)                  # Dispatch the allocated task, starting it.

        print("After allocation, first task, first robot.")
        task_queue.pending_task_list[0].info()  # Check if the first task in the list is now allocated.
        print(robot_list[0].id, robot_list[0].status)
        
        tmp_task = task_queue.allocate_pending_task()           # Try allocating next task in queue.
        print(tmp_task)
        if tmp_task != None:
            task_queue.dispatch_task(tmp_task)                  # Dispatch the allocated task, starting it.

        print("\nInfo on all pending Tasks in the TaskQueue:")
        for task in task_queue.pending_task_list:
            task.info()

        print("\nInfo on all active Tasks in the TaskQueue:")
        for task in task_queue.active_task_list:
            task.info()

        # # Testing the adding of multiple Task class instances and their functionality.    OLD CODE FOR TEMPORARY REFERENCE
        # task_1 = Task("task001")
        # task_1.add_item(RobotMoveBase(location_1))      # Items can be added 1 by 1
        # task_1.add_item(AwaitingLoadCompletion())
        # list_of_items = [RobotMoveBase(location_2), RobotMoveBase(location_1), RobotMoveBase(location_2)]
        # task_1.add_items(list_of_items)                 # Or multiple at a time in a list.
        # task_1.info()

        # task_2 = Task("task002", "rdg02")               # Robot id can be assigned immediately or left blank and assigned later.
        # list_of_items = [RobotMoveBase(location_3), AwaitingLoadCompletion(), RobotMoveBase(location_4), RobotMoveBase(location_3), RobotMoveBase(location_4)]
        # task_2.add_items(list_of_items)
        # task_2.info()

        # task_1.start_task()             # Try starting task even though robot_id is not assigned. It should loginfo.
        # task_1.assign_robot("rdg01")    # Assigning a robot_id to the task
        # task_1.start_task()             # Try starting the task again. It should succeed this time.
        # task_1.info()

        # task_2.start_task()             # In parallel call another robot to perform a task.
        # task_2.info()

        # Spin the node while tasks are being executing, while receiving messages, callbacks etc.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass