AssignJobToMex srv
==================

The `AssignJobToMex.srv`_ is a service message type which takes in a 'MEx ID' string and a 'Job ID' string and returns a boolean 'succes'. 

The service message is used to assign a Job to a Mobile Executor in the MEx Sentinel, which is hosting the service. It is called by the Job Manager during Job Allocation.

.. _AssignJobToMex.srv: ../srv/AssignJobToMex.html