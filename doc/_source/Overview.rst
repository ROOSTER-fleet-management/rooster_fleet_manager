**********************
Fleet Manager Overview
**********************

This page describes the overall structure of the envisioned Fleet manager.
The envisioned Fleet manager consists of the following 3 components:

* Job Manager
* MEx Sentinel
* Frontend GUI

Job Manager
===========
This component of the fleet manager pertains to
obtaining orders from the Frontend GUI, 
either by a user or reading from a file,
processing them to create job instances,
allocating these jobs to the available robots (Mobile Executors),
and dispatching these jobs to them
and also updating information on the MEx Sentinel.

It consists of:

* job builder
* job refiner
* job allocator

Job
###
Each Job is made up of tasks. Currently available tasks are 
movebase, wait for load completion and wait for unload completion. 
TODO: describe more



