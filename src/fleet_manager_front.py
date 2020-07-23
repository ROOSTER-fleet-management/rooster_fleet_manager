#! /usr/bin/env python
import json         # Used for reading JSON files (loading jobs to JobQueue)
import os           # Used to get base filename and file and directory handling

import rospy
from simple_sim.srv import PlaceOrder, PlaceOrderRequest

#region ################################### TO DO LIST #####################################
# DONE 1. Perform simple call /job_manager/place_order service.
# DONE 2. Load list of orders from JSON.
# DONE 3. Call /job_manager/place_order(s?) service with the list of orders.
#endregion #################################################################################

def add_orders_from_dict(order_dict):
    """ Add multile orders to the order_list from a dictionary. """
    # The order of the orders matter, so don't just loop over the dictionary, but check size and loop over order id's
    order_dict_length = len(order_dict)
    rospy.wait_for_service('/job_manager/place_order')
    try:
        place_order = rospy.ServiceProxy('/job_manager/place_order', PlaceOrder)
        req = PlaceOrderRequest()
        for order_id in range(order_dict_length):
            order_info_dict = order_dict[str(order_id)]
            req.keyword = order_info_dict["keyword"]
            req.priority = order_info_dict["priority"]
            req.order_args = order_info_dict["order_args"]
            resp = place_order(req)
            print("Response: ", resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def load_orders_from_JSON(filepath):
    """ Add one or multiple orders from a JSON file. """
    # Load JSON file into dictionary
    loaddata_dict = None
    with open(filepath) as json_loadfile:
        loaddata_dict = json.load(json_loadfile)
    
    # Add orders from loaded JSON dictionary to the order_list.
    add_orders_from_dict(loaddata_dict)

if __name__ == '__main__':
    try:     
        # Initialize the node.
        rospy.init_node('fleet_manager_front')

        filename = "example_orders.JSON"
        if not os.path.isfile(filename):
            # File does not exist yet (first time usage). Create it first from dictionary.
            order_data = {
                "0" : {
                    "keyword" : "MOVE",
                    "priority" : "LOW",
                    "order_args" : ["loc03"]
                },
                "1" : {
                    "keyword" : "MOVE",
                    "priority" : "MEDIUM",
                    "order_args" : ["loc02"]
                },
                "2" : {
                    "keyword" : "TRANSPORT",
                    "priority" : "LOW",
                    "order_args" : ["loc01", "loc02"]
                },
                "3" : {
                    "keyword" : "LOAD",
                    "priority" : "MEDIUM",
                    "order_args" : []
                },
                "4" : {
                    "keyword" : "UNLOAD",
                    "priority" : "HIGH",
                    "order_args" : []
                },
                "5" : {
                    "keyword" : "TRANSPORT",
                    "priority" : "CRITICAL",
                    "order_args" : ["loc04", "loc01"]
                }
            }
            with open(filename, 'w') as outfile:
                json.dump(order_data, outfile, indent=4)
            pass
        load_orders_from_JSON(filename)

        # Keep node running.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass