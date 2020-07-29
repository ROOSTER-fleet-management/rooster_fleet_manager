#! /usr/bin/env python
import json         # Used for reading JSON files (loading jobs to JobQueue)
import os           # Used to get base filename and file and directory handling
from JobManager.Location import Location
import rospy

#function to read locations from JSON file (that is list of dictionaries) and upload to ROS parameter server as dictionaries
def set_up_locations():
    with open('locations.JSON') as json_file:
        data = json.load(json_file)
    for loc in data: #for each item (dictionary) in JSON file (list) 
        loc_params = [loc["id"], loc["name"], loc["x_coordinate"], loc["y_coordinate"], loc["orientation"]] #prepare list of a location parameters
        rospy.set_param('locations/'+str(loc["id"]), loc_params) #set a location parameters

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

set_up_locations()