fleet_manager.launch 
====================

This launch file launches the following ROS nodes:

* mex_sentinel.py
* job_manager.py
* fleet_manager_front.py
* marker_array.py

The order of node launching is important, first the MEx Sentinel node should be launched, which sets up a local robot list and the necessary services and publisher.
Next the Job Manager node should be launched, which retrieves the MEx list from the MEx Sentinel, after which it sets up its own services.
Finally the Fleet Manager Front node should be launched, which acts as the GUI for the Fleet Manager and calls the MEx Sentinel's and Job Manager's services to function.

The Marker Array node launch order is irrelevent with respect to the above mentioned nodes as it acts independently from them. In order to see the markers however, an instance of dynaRViz (as launched from the Multi-Robot Sim launcher) or RViz (with the necessary Marker Array panel) should be running.