#! /usr/bin/env python

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
        print(NAME + 
            "Location info [" + self.name + "]: x, y, theta = " + str(self.x) + 
            ", " + str(self.y) + ", " + str(self.theta) )

#function to read locations info from ROS parameter server and turn them into the dictionary
def make_location_dict():
    class my_dictionary(dict): #class for location_dict to inherit add function to append locations
  
        # __init__ function 
        def __init__(self): 
            self = dict() 
          
        # Function to add key:value 
        def add(self, key, value): 
            self[key] = value 

    location_dict = my_dictionary() #create location object
    temp_dictionary = rospy.get_param("/locations") #read parameters from ROS parameter server
    for loc in temp_dictionary: #turning parameters to a dictionary items
        location_dict.add(loc, Location(temp_dictionary[loc][0], temp_dictionary[loc][1], float(temp_dictionary[loc][2]), float(temp_dictionary[loc][3]), float(temp_dictionary[loc][4])))
    temp_dictionary.clear()
    return location_dict