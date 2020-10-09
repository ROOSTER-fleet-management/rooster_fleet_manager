marker\_array ROS node
=====================

The Marker Array ROS node reads location information from the ROS parameter server. It then sets up the matching sphere and text markers and publishes these on the 'visualization_marker_array' topic.

.. list-table:: Publish / subscribe information
   :widths: 33 33 34
   :header-rows: 1

   * - Publishes to / Subscribes to
     - Topic name
     - Msg Type
   * - Publishes to
     - /visualization_marker_array
     - MarkerArray
   * - Subscribes to
     - none
     - none

marker\_array module
####################

.. automodule:: marker_array
    :members:
    :undoc-members:
    :show-inheritance:
