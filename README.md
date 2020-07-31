# The simple_sim Fleet Management package



#### Dependencies
**multi_robot_sim by @neel.nagda, @patrick.keesmaat**

The simple_sim package is dependend upon the multi_robot_sim package, which can be found here: https://git.tu-delft.ne-kloud.de/neel.nagda/multi_robot_sim/src/melodic-devel

Install this package before continuing, carefully following the README.md instructions. 


**PyQT4**

This package requires the python module *PyQt4* to be installed:

```console
sudo apt-get install python-qt4
```


#### Installation
Once all dependencies have been succesfully installed you can continue with the simple_sim package itself.

1. Clone the repository into your workspace src folder. 
2. Run catkin_make in the toplevel directory of your workspace.
3. Launch the multi_robot_sim according to it's README.
4. Use the following rosluanch command to run the simple_sim package, starting the MEx Sentinel (MobileExecutor), Markey Array Publisher, Job Manager and finally the Fleet Manager Front End GUI.

```console
roslaunch simple_sim fleet_manager.launch
```
*Note: Before launching the simple_sim Fleet Manager, first use the multi_robot_sim gui_based_launcher to set up the simulation environment and necessary ROS parameters, see also the [multi_robot_sim README.md file](https://git.tu-delft.ne-kloud.de/neel.nagda/multi_robot_sim/src/melodic-devel/README.md).*


#### Usage
***The Fleet Manager application.***

In the **Fleet tab** gives an overview of the current *Jobs* and *Mobile Executors* in the fleet. Clicking any of the column headers will filter the lists in ascending or descending order based on that column's header.\
In the **Orders tab** new orders can created, added to a pool and placed (send to the Job Manager). Right clicking a order in the *Order lists* allows the user to delete it. Depending upon the provided *keyword*, a placeholder text is placed in the *Order arguments* input field clarifying the required and types of arguments.\
Hovering over certain items in the Fleet Manager will provide further information in the *status bar* at the bottom of the application.\
More information on the Fleet Manager and simple_sim package, including license information, can be found in the *About screen* (File>About or *Ctrl+A*).\
**Shortcuts**\
Ctrl+A  Show the **A**bout screen\
Ctrl+Q  **Q**uit the application\
Alt+C   **C**lear the Order list\
Alt+P   **P**lace the orders in the Order list\
Alt+A   **A**dd a new order to the Order list based on the provided input fields.\

***MEx Sentinel.***\
The MEx Sentinel runs in the background and keeps track of the Mobile Executor's MEx ID, status and assigned Job ID.\
It provides *callable services locally* for the MEx List (`/get_mex_list`), retrieving and changing a MEx's status (`/get_mex_status` and `/change_mex_status`), assigning and unassigning Jobs (`/assign_job_to_mex` and `/unassign_job_from_mex`).\
It also publishes the *MEx List* to it's local `/mex_list_info` topic.\

***Job Manager.***\
The Job Manager takes in orders from the Fleet Manager Front and appends them to a order list.\
Every second the Job Manager then checks this Order List and builds Jobs with Tasks from the orders, sorting them based on priority and placing them in the *Pending Job* list.\
Every few seconds the Job Manager will check this Pending Job list and pick the first Pending Job(highest priority first, thn within priotity First In First Out). It then allocates this Job to an available (and if local is applicable; closest) MEx, passing the Job on to the *Active Jobs* list.\
Active Jobs keep track of their own Tasks and updates the MEx on the MEx Sentinel. Upon Job completion (succes, cancel, abort) the Job is removed from the Active Jobs list and the MEx is freed and set to STANDBY in the MEx Sentinel.\

The Job Manager provides *callable services locally* for placing Orders (`/place_order`), retrieving the Pending Jobs list (`/get_pending_jobs`), retrieving the Active Jobs list (`/get_active_jobs`) and retrieving detailed Job Information (`/get_job_info`).\

***Marker Array.***\
The Marker Array publisher takes in the Fleet's order locations and publishes these to the *Rviz MarkerArray* topic (`/visualization_marker_array`).\
Inside Rviz these Markers are visualized as magenta sphere's with their name in white text.\
In order to see these markers: open RViz, add "MarkerArray" view, and choose the right topic as mentioned above.\



#### Planned
The following items are planned to be implemented in the simple_sim package:

* [ ] Job Refinement based on Mobile Executors capabilities and attributes.
* [ ] Modular launching of Rviz based on provided Fleet information (Mobile Executors).
* [ ] Job assignment optimization.
* [ ] MEx/Task Routing 