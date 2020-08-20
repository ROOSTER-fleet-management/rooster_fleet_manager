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

  * clone multi_ridgeback_gazebo in the src directory of your catkin_ws

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