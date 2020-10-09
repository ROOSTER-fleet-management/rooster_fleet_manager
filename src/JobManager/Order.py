#! /usr/bin/env python

"""
The Order module contains definitions for several enumerator types; the Order Response status, the Order keyword and the Order argument count.

Note: The 'FOLLOW' OrderKeyword and OrderTypeArgCount are not actively implemented.
"""

from enum import Enum

class OrderResponseStatus(Enum):
    """ Class that acts as Enumerator for Order Response status. """
    # 0 = SUCCES, 1 = ERROR
    SUCCES = 0
    ERROR = 1

class OrderKeyword(Enum):
    """ Class that acts as Enumerator for Order keywords. """
    TRANSPORT = 0
    MOVE = 1
    FOLLOW = 2
    LOAD = 3
    UNLOAD = 4

class OrderTypeArgCount(Enum):
    """ Class that acts as Constants for Order types and the number of arguments associated with them. """
    # Example incoming order: TRANSPORT, priority, [from_location, to_location]
    # Example incoming order: MOVE, priority, [to_location]
    # Example incoming order: FOLLOW, priority, [leader_id]
    # Example incoming order: LOAD, priority, []
    # Example incoming order: UNLOAD, priority, []
    TRANSPORT = 2
    MOVE = 1
    FOLLOW = 1
    LOAD = 0
    UNLOAD = 0