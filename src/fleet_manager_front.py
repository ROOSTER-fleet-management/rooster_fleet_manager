#! /usr/bin/env python
import json         # Used for reading JSON files (loading jobs to JobQueue)
import os           # Used to get base filename and file and directory handling
import sys

import rospy
from PyQt4 import QtGui, QtCore

from simple_sim.srv import PlaceOrder, PlaceOrderRequest
from ui import fleet_manager_ui

#region ################################### TO DO LIST #####################################
# DONE 1. Perform simple call /job_manager/place_order service.
# DONE 2. Load list of orders from JSON.
# DONE 3. Call /job_manager/place_order(s?) service with the list of orders.
# DONE 4. Add visually correct but non-functional GUI to node.
# TODO 5. Replace JSON order list call and instead visualize it in the Order tab.
# TODO 6. Connect MEx Sentinel to Mobile Executors treeWidget view.
#endregion #################################################################################


VERSION = "1.0"
APPLICATION_TITLE = "Fleet Manager"
print(APPLICATION_TITLE + ". Version: "+VERSION)


#region         ### PyQt GUI ###
class GuiMainWindow(fleet_manager_ui.Ui_MainWindow, QtGui.QMainWindow):
    def __init__(self):
        """
        Initialise the ui widgets, items and varibles.
        Connect up all UI interactions to their methods. Define status tips.
        """
        super(GuiMainWindow, self).__init__()
        self.setWindowTitle(APPLICATION_TITLE)  #self.filename + " - " + 

        # Set up gui
        self.setupUi(self)
    
    def close_application(self):
        """Prompts the user if they are sure they which to quit the application before quitting."""
        choice = QtGui.QMessageBox.question(self, 
                                            'Quit application?',
                                            "Are you sure you want to quit? Any unsaved changed will be lost!", 
                                            QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
        
        if choice == QtGui.QMessageBox.Yes:
            print("Closing Fleet Manager node...")
            QtCore.QCoreApplication.instance().quit()
        else:
            pass
    
    def closeEvent(self, event):
        """Takes control of the close event, making sure the user cannot close the application before prompting them."""
        event.ignore()
        self.close_application()
    
    def about(self):
        """Display a MessageBox with the application title, version number and general information."""
        text = "<center>" \
            "<h2>"+APPLICATION_TITLE+"</h2>" \
            "</center>" \
            "The ROS package simple_sim is created by the Human " \
            "Robot Coproduction research group at the Industrial Design " \
            "Engineering faculty of the Delft University of Technology." \
            "<p>Version: "+VERSION+"<br/>" \
            "License: Apache License version 2.0</p>"
        QtGui.QMessageBox.about(self, "About - " + APPLICATION_TITLE + ".", text)

#endregion      ### PyQt GUI ###


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
    # add_orders_from_dict(loaddata_dict)

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

        #region --- GUI ---
        app = QtGui.QApplication(sys.argv)
        appGui = GuiMainWindow()
        #windowIcon = QtGui.QIcon()
        #windowIcon.addPixmap(QtGui.QPixmap(":/icons/Launch Icon Multi.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        #appGui.setWindowIcon(windowIcon)
        appGui.show()
        app.exec_()
        #endregion

        # Keep node running.
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass