**********************
Fleet Manager Overview
**********************

This page describes the overall structure of the Fleet manager with its components and its application.
The Fleet manager consists of the following 4 main components:

* `Front End GUI (application)`__
* `Job Manager`_
* `MEx Sentinel`_
* `Marker Array`_

__ `The Fleet Manager application`_

The architectural diagram of the ROOSTER Fleet Manager looks as follows (each ROS node is signified by a blue outline):
 .. image:: ../_static/images/ROOSTER_architectural_diagram.png
    :alt: alternate text test
    :width: 800

The Fleet Manager application
------------------------------
In the Fleet Manager application you can get an overview of the current fleet and jobs, and place new orders. Each is on its own tab; 

In the **Fleet tab** an overview is given of the current *Jobs* and *Mobile Executors* in the fleet. Clicking any of the column headers will filter the lists in ascending or descending order based on that column's header.

In the **Orders tab** new orders can created, added to a pool and placed (send to the Job Manager). Right clicking an order in the *Order list* allows the user to delete it. Depending upon the provided *keyword*, a placeholder text is placed in the *Order arguments* input field clarifying the required and types of arguments.

Hovering over certain items in the Fleet Manager will provide further information in the *status bar* at the bottom of the application.

More information on the Fleet Manager and ROOSTER package version and license can be found in the *About screen* (File>About or *Ctrl+A*).

**Shortcuts**

============  ===============
**Shortcut**  **Explanation**
------------  ---------------
`Ctrl+A`      Show the About screen.
`Ctrl+Q`      Quit the application.
`Alt+C`       Clear the Order list.
`Alt+P`       Place the orders in the Order list.
`Alt+A`       Add a new order to the Order list based on the provided input fields.
============  ===============
 
For more information on the Fleet Manager Front node see `Fleet Manager Front node`_.

.. _`Fleet Manager Front node`: src/fleet_manager_front.html

MEx Sentinel
-----------------
The MEx Sentinel runs in the background and keeps track of the Mobile Executor's MEx ID, status and assigned Job ID.

It provides *callable services locally* for retrieving the MEx List (:code:`/get_mex_list`), retrieving and changing a MEx's status (:code:`/get_mex_status` and :code:`/change_mex_status`) and for assigning and unassigning Jobs (:code:`/assign_job_to_mex` and :code:`/unassign_job_from_mex`).

It also publishes the *MEx List* to it's local :code:`/mex_list_info` topic.

For more information on the MEx Sentinel node see `MEx Sentinel node`_.

.. _`MEx Sentinel node`: src/mex_sentinel.html

Job Manager
----------------
The Job Manager takes in orders from the Fleet Manager Front and appends them to an order list.

Every second the Job Manager then checks this Order List and builds Jobs with Tasks from the orders, sorting them based on priority and placing them in the *Pending Job* list.

Every few seconds the Job Manager will check this Pending Job list and pick the first Pending Job(highest priority first, then within priotity First In First Out). It then allocates this Job to an available (and if local is applicable; closest) MEx, passing the Job on to the *Active Jobs* list.

Active Jobs keep track of their own Tasks and updates the MEx on the MEx Sentinel. Upon Job completion (succes, cancel, abort) the Job is removed from the Active Jobs list and the MEx is freed and set to STANDBY in the MEx Sentinel.

The Job Manager provides *callable services locally* for placing Orders (:code:`/place_order`), retrieving the Pending Jobs list (:code:`/get_pending_jobs`), retrieving the Active Jobs list (:code:`/get_active_jobs`) and retrieving detailed Job Information (:code:`/get_job_info`).

For more information on the Job Manager node see `Job Manager node`_.

.. _`Job Manager node`: src/job_manager.html

Marker Array
-----------------
The Marker Array publisher takes in the Fleet's order locations and publishes these to the *Rviz MarkerArray* topic (:code:`/visualization_marker_array`).

Inside Rviz these Markers are visualized as magenta sphere's with their name in white text.
In order to see these markers: open RViz, add "MarkerArray" view, and choose the right topic as mentioned above.

For more information on the Marker Array node, see `Marker Array node`_.

.. _`Marker Array node`: src/marker_array.html

Order
-----
An order is a to-be-created Job. It comes in through the PlaceOrder service as hosted by the Job Manager. The PlaceOrder service is by default called by the Fleet Manager Front, but provided the right structure is used, any node could call the service to add orders.

For more information on the PlaceOrder service, see `ROS Services`_.

.. _`ROS Services`: SrvLandingPage.html

Jobs
----
Each Job is made up out of one or more Tasks. Currently available tasks are 
'robot movebase', 'wait for load completion' and 'wait for unload completion'. 

For more information on Jobs, see the `Job Manager package, Job module`_.

.. _`Job Manager package, Job module`: src/JobManager.html#module-JobManager.Job



Mobile Executor (MEx)
---------------------
A Mobile Executor or MEx is the overarching name for mobile robots and manually controlled vehicles.
It signifies those vehicles, be it manned or autonomous, that are mobile and capable of executing Jobs.

For more information on Mobile Executors, see the `Job Manager package, MEx module`_.

.. _`Job Manager package, MEx module`: src/JobManager.html#module-JobManager.MobileExecutor