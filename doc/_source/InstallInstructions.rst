*************************
Installation Instructions
*************************

* Install ridgeback and multi-ridgeback packages:
  
  * Install the ridgeback simulation and navigation packages by clearpath robotics using the bash commands in the terminal. 
    
    .. code-block:: console
       
       sudo apt-get install ros-melodic-ridgeback-simulator ros-melodic-ridgeback-desktop ros-melodic-ridgeback-navigation
    
    To learn about using these packages check out the `tutorials`_ by Clearpath Robotics.

    .. _tutorials: <http://www.clearpathrobotics.com/assets/guides/kinetic/ridgeback/simulation.html>

  * Install rospackage for teleoperation from terminal.

       .. |rviz_toolp1| image:: ../_static/images/set_rviz_tool_properties_1.png
          :alt: alternate text
          :width: 150

       .. |rviz_toolp2| image:: ../_static/images/set_rviz_tool_properties_2.png
          :alt: alternate text
          :width: 150

       .. |rviz_toolp3| image:: ../_static/images/set_rviz_tool_properties_3.png
          :alt: alternate text
          :width: 150      

       .. list-table:: setting 2D Nav Goal topic
          :widths: 50 50 50
          :header-rows: 0

  * clone multi_ridgeback_nav in the src directory of your catkin_ws.
          
* Install SFM MPDM package including dependencies

  * Installing dependency libraries
  
    .. code-block:: console

       sudo apt-get install libopenblas-dev liblapack-dev libarpack2-dev libsuperlu-dev
    
  * Installing Armadillo C++ library

  * Clone obstacle_detector package into catkin workspace and catkin_make.

  * Install pygame

    .. code-block:: console
       
       sudo apt-get install python-pygame

  * Clone sfm_mpdm ROS package in the src directory of your workspace.




* Install multi_robot_sim
* Install PyQt4
  
  .. code-block:: console
     
     sudo apt-get install pyqt4-dev-tools pyqt4.qsci-dev libqt4-dev python-qt4

* Install the documentation generation tools

  * Install `rosdoc_lite`_ ROS package using the following bash command in a terminal.
  
    .. _rosdoc_lite: http://wiki.ros.org/rosdoc_lite

    .. code-block:: console

       sudo apt-get install ros-melodic-rosdoc-lite

  * Install catkin-sphinx which is a sphinx extension that provides ROS theme for documentation.
  
    .. code-block:: console
       
       python2 -m pip install --user catkin-sphinx

.. * Install catkin-sphinx which is a sphinx extension that provides ROS theme for documentation.
  
..   .. code-block:: console
       
..      python2 -m pip install --user sphinx