#! /usr/bin/env python

class Location:
    """ Class with location information (name, position, orientation) based on map reference frame. """
    def __init__(self, id, name, x_coordinate, y_coordinate, theta):
        self.id = id                # Unique identifier for this location, e.g. "loc01"
        self.name = name            # Name of the location, string
        self.x = x_coordinate       # X position in map frame in meters, float
        self.y = y_coordinate       # Y position in map frame in meters, float
        self.theta = theta          # Orientation (yaw angle) in map frame in radians, float

    def info(self):
        print(
            "Location info [" + self.name + "]: x, y, theta = " + str(self.x) + 
            ", " + str(self.y) + ", " + str(self.theta) )