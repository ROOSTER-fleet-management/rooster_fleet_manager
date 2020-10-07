ChangeMexStatus srv
===================

The `ChangeMexStatus.srv`_ is a service message type which takes in a 'MEx ID' string and a MEx Status integer. It returns a boolean 'succes'.

The service message is used to change a Mobile Executors status in the MEx Sentinel. The service is hosted by the MEx Sentinel and called by the Job Manager.

.. _ChangeMexStatus.srv: ../../srv/ChangeMexStatus.html