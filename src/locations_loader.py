#! /usr/bin/env python
import json         # Used for reading JSON files (loading jobs to JobQueue)
import os           # Used to get base filename and file and directory handling
from JobManager.Location import Location

#--------------- for locations as a list (locations.JSON file)-------------------
#locations = []
#with open('locations.JSON') as json_file:
#    data = json.load(json_file)
    #for loc in data:
    #    location = Location(loc['id'],loc['name'], loc['x_coordinate'], loc['y_coordinate'], loc['orientation'])
    #    locations.append(location)
#for i in locations:
#    i.info()


#--------------- for locations as a dictionary (locations2.JSON file)-------------------
class my_dictionary(dict): 
  
    # __init__ function 
    def __init__(self): 
        self = dict() 
          
    # Function to add key:value 
    def add(self, key, value): 
        self[key] = value 

location_dict = my_dictionary()
with open('locations2.JSON') as json_file:
    data = json.load(json_file) #data is dictionary
    for loc in data: #loc is unicode
        location_dict.add(str(loc), Location(data[loc]["id"], data[loc]["name"], data[loc]["x_coordinate"], data[loc]["y_coordinate"], data[loc]["orientation"]))
#print(location_dict)
#print(location_dict.items())
for i in location_dict.items():
    print('location: ' + i[0])
    i[1].info()
    #print(i[1].theta)

#location_dict["loc04"].info()