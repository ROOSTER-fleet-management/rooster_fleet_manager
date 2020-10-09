mex\_sentinel ROS node
======================

The MEx Sentinel is a ROS node that holds the latest information about Mobile Executors (MExs). It hosts services for retrieving and editing the MEx information:

 - get_mex_list
 - assign_job_to_mex
 - unassign_job_from_mex
 - get_mex_status
 - change_mex_status

Note: In it current state the MEx information is only stored in memory and no data is logged. It is however envision that the MEx Sentinel will interact with (reads/writes from/to) a database for maintaining long term records 
(i.e. acts as front end to a database server like SQLite or MySQL). 

mex\_sentinel script
####################

.. automodule:: mex_sentinel
    :members:
    :undoc-members:
    :show-inheritance:
