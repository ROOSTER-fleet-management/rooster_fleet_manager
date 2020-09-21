ActiveJob msg
=============

The `ActiveJob.msg`_ message type is used to send information about a specific Active Job, Active meaning a MobileExecutor is assigned and the first Task of the Job has been started.

It contains information such as the Job ID, it's status, priority, linked MobileExecutor ID, the number of tasks it has, the current task index, and a keyword signifying the type of Job.

.. _ActiveJob.msg: ../../msg/ActiveJob.html