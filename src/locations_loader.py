#! /usr/bin/env python
import json         # Used for reading JSON files (loading jobs to JobQueue)
import os           # Used to get base filename and file and directory handling
import rospy

#function to read locations from JSON file (that is list of dictionaries) and upload to ROS parameter server as dictionaries
def set_up_locations():
    with open('locations.JSON') as json_file:
        data = json.load(json_file)
    for loc in data: #for each item (dictionary) in JSON file (list) 
        loc_params = [loc["id"], loc["name"], loc["x_coordinate"], loc["y_coordinate"], loc["orientation"]] #prepare list of a location parameters
        rospy.set_param('locations/'+str(loc["id"]), loc_params) #set a location parameters

set_up_locations()