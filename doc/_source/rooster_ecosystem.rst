**************************
Fleet Simulation Ecosystem 
**************************

To simulate and assess the task allocation, scheduling and routing capabilites 
of the fleet manager, a collection of simulation related ROS packages have been developed
in the ROOSTER fleet management ecosystem.

These packages include

* `multi_ridgeback_gazebo <https://github.com/ROOSTER-fleet-management/multi_ridgeback_gazebo.git>`__ and `and multi_ridgeback_nav <https://github.com/ROOSTER-fleet-management/multi_ridgeback_nav.git>`__
* `multi_husky_gazebo <https://github.com/ROOSTER-fleet-management/multi_husky_gazebo.git>`__ and `multi_husky_nav <https://github.com/ROOSTER-fleet-management/multi_husky_nav.git>`__

These packages have been built on top of simulation packages developed by Clearpath Robotics
to simulate their robots 'Ridgeback' and 'Husky'.
The primary contribution of our packages is :

* launching the localization, navigation, and other robot specific nodes in the correct namespace for each robot in the fleet
* Setting the correct tf (coordinate transformation) frame ids for the links of each robot

.. note::

	 Currently, only simulating Husky and Ridgeback is supported in ROOSTER.
	 The platform can and will be extended to support more robots.      

Additionally, `multi_robot_sim <https://github.com/ROOSTER-fleet-management/multi_robot_sim.git>`__ 
package  
 
* provides a GUI interface to choose and launch the desired type and number of robots to form the heterogenous fleet 
* launches a `gazebo <http://gazebosim.org/>`__ simulation with the specified map and chosen robots
* launches `rviz <http://wiki.ros.org/rviz>`__ to visualize the laser scan data of the robots and their navigation paths
