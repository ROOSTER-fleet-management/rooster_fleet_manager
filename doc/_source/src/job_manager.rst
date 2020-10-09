job\_manager ROS node
=====================

The Job Manager takes in orders from the Fleet Manager Front and appends them to a order list.

Every second the Job Manager then checks this Order List and builds Jobs with Tasks from the orders, sorting them based on priority and placing them in the Pending Job list.

Every few seconds the Job Manager will check this Pending Job list and pick the first Pending Job (highest priority first, then within priotity First In First Out). It then allocates this Job to an available (and if local is applicable; closest) MEx, passing the Job on to the Active Jobs list.

Active Jobs keep track of their own Tasks and updates the Mobile Executor (MEx) on the MEx Sentinel. Upon Job completion (succes, cancel, abort) the Job is removed from the Active Jobs list and the MEx is freed and set to STANDBY in the MEx Sentinel.

.. JOB MANAGER DOES NOT PUBLISH / SUBSCRIBE 
.. .. list-table:: Publish / subscribe information      
..    :widths: 33 33 34
..    :header-rows: 1

..    * - Publishes to / Subscribes to
..      - Topic name
..      - Msg Type
..    * - Publishes to
..      - TO DO
..      - TO DO
..    * - Subscribes to
..      - TO DO
..      - TO DO

The Job Manager provides callable services locally for

* placing Orders (:ref:`PlaceOrder-srv-label`) 
* retrieving the Pending Jobs list (:ref:`GetPendingJobs-srv-label`) 
* retrieving the Active Jobs list (:ref:`GetActiveJobs-srv-label`) 
* retrieving detailed Job Information (:ref:`GetJobInfo-srv-label`)

job\_manager script
###################

.. automodule:: job_manager
    :members:
    :undoc-members:
    :show-inheritance:
