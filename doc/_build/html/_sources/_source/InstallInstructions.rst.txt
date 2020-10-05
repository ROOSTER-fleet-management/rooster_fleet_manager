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

  * Clone the multi_ridgeback_gazebo ROS package into the src directory of your catkin_ws and run catkin_make from the root of your catkin_ws.

    .. code-block:: console
       
       mkdir -p ~/catkin_ws/src
       cd ~/catkin_ws/src
       git clone https://contact/n.n.nagda@tudelft.nl/for/access.git 
       cd ..
       catkin_make

  * .. note::
       
       To confirm the installation uptil this point, run the following roslaunch command in a terminal. 
       
       .. code-block:: console
          
          roslaunch multi_ridgeback_gazebo multi_ridgeback_world.launch

       This should launch a gazebo simulation with 2 ridgebacks that can be teleoperated using the teleop nodes running in the launched terminals. 
       Also, an instance of Rviz is launched, wherein you can visualize their laserscans and the TF tree.   

  * Clone the multi_ridgeback_nav ROS package into the src directory of your catkin_ws and run catkin_make from the root of your catkin_ws.
    
    .. code-block:: console
       
       cd ~/catkin_ws/src
       git clone https://contact/n.n.nagda@tudelft.nl/for/access.git 
       cd ..
       catkin_make

  * .. note::
       
       To confirm the installation uptil this point, run the following roslaunch command in a terminal. 
       
       .. code-block:: console
          
          roslaunch multi_ridgeback_nav multi_ridgeback_nav.launch

       This should launch a gazebo simulation with 3 ridgebacks and an instance of Rviz to issue navigation goals. 
       Additionally terminals are also launched to manually operate the robot if it is stuck. 
       The 3 ridgebacks have IDs as rdg01 rdg02 and rdg03 respectively and their nodes and topics are live under those namespaces respectively.
       The 2d Nav Goal tool from the tool ribbon can be used to issue navigation goals to the respective robot by setting the current topic name for it in the tool properties.


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

          * - |rviz_toolp1|        
            - |rviz_toolp2|
            - |rviz_toolp3|
          * - Right click on 2D Nav Goal tool and click on tool properties
            - Click the 2D Nav Goal topic
            - Set the correct robot ID in the 2D Nav Goal topic base namespace 

* Install sfm_pmdm package including dependencies see `SFM-MPDM`_.

 .. _SFM-MPDM: https://git.tu-delft.ne-kloud.de/patrick.keesmaat/sfm_mpdm.
 
  * Installing dependency libraries
  
    .. code-block:: console

       sudo apt-get install libopenblas-dev liblapack-dev libarpack2-dev libsuperlu-dev
    
  * Installing `Armadillo C++`_ library

    .. _Armadillo C++: http://arma.sourceforge.net/

  * Clone `obstacle_detector`_ package into catkin workspace and catkin_make.

    .. _obstacle_detector: https://github.com/tysik/obstacle_detector

  * Install pygame

    .. code-block:: console
       
       sudo apt-get install python-pygame

  * Clone sfm_mpdm ROS package in the src directory of your workspace.




* Build the multi_robot_sim ROS package.
  
  * Clone the repository into the src folder of your catkin_ws.

  * Install PyQt4 (Python bindings for Qt) by running the following bash command in a terminal.
  
    .. code-block:: console
     
       sudo apt-get install pyqt4-dev-tools pyqt4.qsci-dev libqt4-dev python-qt4

* Install the documentation generation tools

  * Install `rosdoc_lite`_ ROS package using the following bash command in a terminal.
  
    .. _rosdoc_lite: http://wiki.ros.org/rosdoc_lite

    .. code-block:: console

       sudo apt-get install ros-melodic-rosdoc-lite

    This automatically installs doxygen, sphinx and epydoc.  

  * Install sphinx-rtd-theme, a sphinx extension that provides a beautiful theme for documentation.
  
    .. code-block:: console
       
       python2 -m pip install --user sphinx-rtd-theme     

  * Install catkin-sphinx, a sphinx extension that provides a ROS theme for documentation.
  
    .. code-block:: console
       
       python2 -m pip install --user catkin-sphinx





       
