#! /usr/bin/env python

class OrderResponseStatus:
    """ Class that acts as Enumerator for Order Response status. """
    # 0 = SUCCES, 1 = ERROR
    SUCCES = 0
    ERROR = 1

class OrderKeyword:
    """ Class that acts as Enumerator for Order keywords. """
    TRANSPORT = 0
    MOVE = 1
    FOLLOW = 2

class OrderTypeArgCount:
    """ Class that acts as Constants for Order types and the number of arguments associated with them. """
    # Example incoming order: [transport, priority, from_location, to_location]
    # Example incoming order: [move, priority, to_location]
    # Example incoming order: [follow, priority, leader_id]
    TRANSPORT = 4
    MOVE = 3
    FOLLOW = 3