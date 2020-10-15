*************************
Installation Instructions
*************************

* Ensure you have a functional installation of ROS Melodic Morenia. 
  (It is primarily targeted at the Ubuntu 18.04 (Bionic) release, though other Linux systems as well as Mac OS X, Android, and Windows are supported to varying degrees. 
  For more information on compatibility on other platforms, please see `REP 3: Target Platforms <https://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023>`__). 
  
  .. note::
  
     Our software has been built and tested only on Ubuntu 18.04 using ROS Melodic. 
  
  Refer these `installation instructions <http://wiki.ros.org/melodic/Installation/Ubuntu>`__ for installing ROS Melodic on Ubuntu 18.04.

* In your home directory, make a folder called catkin_ws and an src folder within it. 
  Do so by running the following command in a terminal [#]_.

  .. code-block:: console
     
     mkdir -p ~/catkin_ws/src

* Clone the following repositories into the src folder.

  * `rooster_fleet_manager <https://github.com/ROOSTER-fleet-management/rooster_fleet_manager.git>`__
  * `multi_robot_sim <https://github.com/ROOSTER-fleet-management/multi_robot_sim.git>`__
  * `multi_ridgeback_gazebo <https://github.com/ROOSTER-fleet-management/multi_ridgeback_gazebo.git>`__
  * `multi_ridgeback_nav <https://github.com/ROOSTER-fleet-management/multi_ridgeback_nav.git>`__
  * `multi_husky_gazebo <https://github.com/ROOSTER-fleet-management/multi_husky_gazebo.git>`__
  * `multi_husky_nav <https://github.com/ROOSTER-fleet-management/multi_husky_nav.git>`__

* Install the ROS packages developed by Clearpathpath Robotics for simulating their robots 'Ridgeback' and 'Husky' [#]_ . 
  
  .. code-block:: console
       
     sudo apt install ros-melodic-ridgeback-simulator ros-melodic-ridgeback-desktop ros-melodic-ridgeback-navigation
     sudo apt install ros-melodic-husky-simulator ros-melodic-husky-desktop ros-melodic-husky-navigation

.. * Install ridgeback and multi-ridgeback packages:
  
..   * Install the ridgeback simulation and navigation packages by clearpath robotics using the bash commands in the terminal. 
    
..     .. code-block:: console
       
..        sudo apt-get install ros-melodic-ridgeback-simulator ros-melodic-ridgeback-desktop ros-melodic-ridgeback-navigation
    
..     To learn about using these packages check out the `tutorials`_ by Clearpath Robotics.

..     .. _tutorials: http://www.clearpathrobotics.com/assets/guides/kinetic/ridgeback/simulation.html

..   * Install rospackage for teleoperation from terminal.

..   * Clone the multi_ridgeback_gazebo ROS package into the src directory of your catkin_ws and run catkin_make from the root of your catkin_ws.

..     .. code-block:: console
       
..        mkdir -p ~/catkin_ws/src
..        cd ~/catkin_ws/src
..        git clone https://contact/n.n.nagda@tudelft.nl/for/access.git 
..        cd ..
..        catkin_make

..   * .. note::
       
..        To confirm the installation uptil this point, run the following roslaunch command in a terminal. 
       
..        .. code-block:: console
          
..           roslaunch multi_ridgeback_gazebo multi_ridgeback_world.launch

..        This should launch a gazebo simulation with 2 ridgebacks that can be teleoperated using the teleop nodes running in the launched terminals. 
..        Also, an instance of Rviz is launched, wherein you can visualize their laserscans and the TF tree.   

..   * Clone the multi_ridgeback_nav ROS package into the src directory of your catkin_ws and run catkin_make from the root of your catkin_ws.
    
..     .. code-block:: console
       
..        cd ~/catkin_ws/src
..        git clone https://contact/n.n.nagda@tudelft.nl/for/access.git 
..        cd ..
..        catkin_make

..   * .. note::
       
..        To confirm the installation uptil this point, run the following roslaunch command in a terminal. 
       
..        .. code-block:: console
          
..           roslaunch multi_ridgeback_nav multi_ridgeback_nav.launch

..        This should launch a gazebo simulation with 3 ridgebacks and an instance of Rviz to issue navigation goals. 
..        Additionally terminals are also launched to manually operate the robot if it is stuck. 
..        The 3 ridgebacks have IDs as rdg01 rdg02 and rdg03 respectively and their nodes and topics are live under those namespaces respectively.
..        The 2d Nav Goal tool from the tool ribbon can be used to issue navigation goals to the respective robot by setting the current topic name for it in the tool properties.


..        .. |rviz_toolp1| image:: ../_static/images/set_rviz_tool_properties_1.png
..           :alt: alternate text
..           :width: 150

..        .. |rviz_toolp2| image:: ../_static/images/set_rviz_tool_properties_2.png
..           :alt: alternate text
..           :width: 150

..        .. |rviz_toolp3| image:: ../_static/images/set_rviz_tool_properties_3.png
..           :alt: alternate text
..           :width: 150      

..        .. list-table:: setting 2D Nav Goal topic
..           :widths: 50 50 50
..           :header-rows: 0

..           * - |rviz_toolp1|        
..             - |rviz_toolp2|
..             - |rviz_toolp3|
..           * - Right click on 2D Nav Goal tool and click on tool properties
..             - Click the 2D Nav Goal topic
..             - Set the correct robot ID in the 2D Nav Goal topic base namespace 

* Install python module PyQt4, required for rendering the GUI.

  .. code-block:: console

     sudo apt install python-qt4

* Build the cloned packages. 

  .. code-block:: console

     cd ~/catkin_ws
     catkin_make

* Add the setup.bash file of catkin_ws to .bashrc and source it to set up the environmnet variables pertaining to this ROS workspace.

  .. code-block:: console
     
     echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
     source ~/.bashrc



* [**Optional**] Install the tools required to develop and generate this documentation.

  * Install `rosdoc_lite <http://wiki.ros.org/rosdoc_lite>`__ ROS package using the following bash command[#]_ in a terminal.
    
    .. code-block:: console

       sudo apt-get install ros-melodic-rosdoc-lite
  
  * Install sphinx_rtd_theme, the sphinx extension use for this documentation.
  
    .. code-block:: console
       
       python2 -m pip install --user sphinx-rtd-theme     

  * Install catkin-sphinx, a sphinx extension that provides a ROS theme for documentation.
  
    .. code-block:: console
       
       python2 -m pip install --user catkin-sphinx


.. rubric:: Footnotes

.. [#] Subsequent commands on this page, presented in the same looking highlighted boxes are to be run in the terminal as well.
.. [#] To learn about using these packages and to play around with these robots in simulation, check out the tutorials by Clearpath Robotics for `Ridgeback <http://www.clearpathrobotics.com/assets/guides/kinetic/ridgeback/simulation.html>`__ and `Husky <http://www.clearpathrobotics.com/assets/guides/kinetic/husky/SimulatingHusky.html>`__.
.. [#] This command automatically installs doxygen, sphinx and epydoc as well.

       
