JobInfo msg
=============

The `JobInfo.msg`_ message type is used to send information about a specific Job including the Job's Tasks.

It contains information such as the Job ID, it's status, priority, linked MobileExecutor ID, the number of tasks it has, the current task index, a list of TaskInfo.msg with status and type of each Task for this Job, and a keyword signifying the type of Job.

.. _JobInfo.msg: ../../msg/JobInfo.html