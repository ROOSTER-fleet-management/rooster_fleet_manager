#! /usr/bin/env python

"""
TO DO: ENTER THE DESCRIPTION OF THIS MODULE HERE (IN THE SOURCE CODE DOC STRING)!!!
"""

import rospy

NAME = "[Location.py] "

class Location:
    """ Class with location information (name, position, orientation) based on map reference frame. """
    def __init__(self, id, name, x_coordinate, y_coordinate, theta):
        self.id = id                # Unique identifier for this location, e.g. "loc01"
        self.name = name            # Name of the location, string
        self.x = x_coordinate       # X position in map frame in meters, float
        self.y = y_coordinate       # Y position in map frame in meters, float
        self.theta = theta          # Orientation (yaw angle) in map frame in radians, float

    def info(self):
        """ Method which prints out general Location information to the console. """
        print(NAME + 
            "Location info [" + self.name + "]: x, y, theta = " + str(self.x) + 
            ", " + str(self.y) + ", " + str(self.theta) )


def make_location_dict():
    """ Function to read locations info from ROS parameter server and turn them into a dictionary. """

    class my_dictionary(dict):
        """ Class for location_dict to inherit the 'add' method from in order to append locations. """
        def __init__(self): 
            self = dict() 
          
        def add(self, key, value):
            """ Method to add key:value to instance dictionary. """
            self[key] = value 

    location_dict = my_dictionary() # Create location object
    temp_dictionary = rospy.get_param("/locations") # Read parameters from ROS parameter server, stores in temporary dictionary to later remove.
    for loc in temp_dictionary: # Turning parameters to a dictionary items
        location_dict.add(loc, Location(temp_dictionary[loc][0], temp_dictionary[loc][1], float(temp_dictionary[loc][2]), float(temp_dictionary[loc][3]), float(temp_dictionary[loc][4])))
    temp_dictionary.clear()
    return location_dict