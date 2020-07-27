#! /usr/bin/env python
import json         # Used for reading JSON files (loading jobs to JobQueue)
import os           # Used to get base filename and file and directory handling
import sys

import rospy
from PyQt4 import QtGui, QtCore

from simple_sim.srv import PlaceOrder, PlaceOrderRequest
from ui import fleet_manager_ui
from JobManager.Order import *

#region ################################### TO DO LIST #####################################
# DONE 1.  Perform simple call /job_manager/place_order service.
# DONE 2.  Load list of orders from JSON.
# DONE 3.  Call /job_manager/place_order(s?) service with the list of orders.
# DONE 4.  Add visually correct but non-functional GUI to node.
# DONE 5.  Replace JSON order list call and instead visualize it in the Order tab.
# TODO 6.  Connect MEx Sentinel to Mobile Executors treeWidget view.
# DONE 7.  Check if the order is viable before adding to order list, notify user if not.
# DONE 8.  Remove orders from Order list if they were placed succesfully.
# DONE 9.  Put in Placeholder text in the arguments field based on the order keyword.
# TODO 10. Connect Job Manager to the Jobs treeWidget view.
# DONE 11. Add the deletion of individual orders from the order list.
# TODO 12. Replace placeholders in the FILE ACTION MENU.
# TODO 13. Add logo in same style as GUI launcher.
# TODO 
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

        #region FILE ACTION MENU
        self.actionAbout.triggered.connect(self.about)
        self.actionQuit_application.triggered.connect(self.close_application)
        #endregion

        #region ORDERS TAB
        self.pushButtonAddOrder.clicked.connect(self.add_order)
        self.pushButtonClearList.clicked.connect(self.clear_order_list)
        self.pushButtonPlaceOrder.clicked.connect(self.place_order_list)
        self.comboBoxKeyword.currentIndexChanged.connect(self.update_order_arguments_placeholder_text)
        self.lineEditArguments.setPlaceholderText("location")

        # TreeWidget context menu
        self.treeWidgetOrders.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.treeWidgetOrders.customContextMenuRequested.connect(self.open_context_menu)  

        self.treeMenu = QtGui.QMenu('Menu', self)
        deleteItem = QtGui.QAction("&Delete", self)
        deleteItem.setStatusTip("Delete item from Order list")
        deleteItem.triggered.connect(self.delete_orders_tree_item)
        deleteIcon = QtGui.QIcon()
        deleteIcon.addPixmap(QtGui.QPixmap(":/icons/Close.png"))
        deleteItem.setIcon(deleteIcon)
        self.treeMenu.addAction(deleteItem)

        #endregion

    def open_context_menu(self):
        """Opens the Right-Mouse-Button context menu, showing an option to delete the order tree item."""
        self.treeMenu.exec_(QtGui.QCursor.pos())

    def delete_orders_tree_item(self, item):
        """Deletes the currently selected item from the orders treeWidget."""
        index = self.treeWidgetOrders.currentIndex()
        self.treeWidgetOrders.takeTopLevelItem(index.row())

    def update_order_arguments_placeholder_text(self):
        """
        Updates the light gray placeholder text of the Order arguments lineEdit
        input field to match with the new keyword.
        """
        keyword = str(self.comboBoxKeyword.currentText())
        if keyword == OrderKeyword.LOAD.name or keyword == OrderKeyword.UNLOAD.name:
            self.lineEditArguments.setPlaceholderText("No Arguments!")
        elif keyword == OrderKeyword.MOVE.name:
            self.lineEditArguments.setPlaceholderText("location")
        elif keyword == OrderKeyword.TRANSPORT.name:
            self.lineEditArguments.setPlaceholderText("location1 location2")
        
    def place_order_list(self):
        """ Place multiple orders from the Order tab Order list to the Job Manager. """
        # The order of the orders matter, so don't just loop over the dictionary, but check size and loop over order id's
        self.order_list = []        # Empty order list
        indices_to_remove = []      # Empty list for treeWidgetOrders indices.

        # Iterate over all the existing (top level) items (a.k.a. orders) in the treeWidgetOrders and add as order.
        root = self.treeWidgetOrders.invisibleRootItem()
        child_count = root.childCount()
        for i in range(child_count):
            item = root.child(i)
            order_keyword = str(item.text(0))
            order_priority = str(item.text(1))
            order_arguments = str(item.text(2)).split()
            order = [order_keyword, order_priority, order_arguments, i]
            self.order_list.append(order)
        
        if len(self.order_list) != 0:
            rospy.wait_for_service('/job_manager/place_order')
            try:
                place_order = rospy.ServiceProxy('/job_manager/place_order', PlaceOrder)
                req = PlaceOrderRequest()
                for order in self.order_list:
                    req.keyword = order[0]
                    req.priority = order[1]
                    req.order_args = order[2]
                    resp = place_order(req)
                    print("Response: ", resp)
                    if resp.error_status == OrderResponseStatus.SUCCES.name:
                        # The placement of this order was succesful, remove from Order list
                        indices_to_remove.append(order[3])
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        
        if len(indices_to_remove) != 0:
            # Sort the indices list in descending order (from highest index to lowest index).
            indices_to_remove.sort(reverse=True)
            # Iterate over the sorted list, removing items from the treeWidgetOrders
            for index in indices_to_remove:
                self.treeWidgetOrders.takeTopLevelItem(index)
        
        if len(indices_to_remove) != len(self.order_list):
            # Show a notification box alerting the user not all orders were placed succesfully.
            QtGui.QMessageBox.warning(self, "Not all orders could be placed succefully!", "Out of the " + str(len(self.order_list)) + " orders, " + str(len(self.order_list) - len(indices_to_remove)) + " could not be placed succesfully. These orders have been kept in the order list, succesful orders have been removed.")
        else:
            # Show a notification informing the user that all orders were placed succesfully.
            QtGui.QMessageBox.information(self, "All orders placed succesfully!", "All " + str(len(self.order_list)) + " order(s) have been placed succesfully and have been removed from the order list.")


    def clear_order_list(self):
        """Clears the Order tab order list."""
        root = self.treeWidgetOrders.invisibleRootItem()
        child_count = root.childCount()
        if child_count > 0:
            self.treeWidgetOrders.clear()

    def add_order(self):
        """
        Add a order to the Order List based on the Order tab input field values.
        Before adding, a check is performaned to make sure the supplied fields are set correctly.
        If this is not the case the user is notified with a MessageBox.
        """
        order_keyword = self.comboBoxKeyword.currentText()
        order_priority = self.comboBoxPriority.currentText()
        order_arguments = self.lineEditArguments.text()
        
        # Check if the number of supplied arguments 
        supplied_args = len(str(order_arguments).split())
        expected_args = OrderTypeArgCount[str(order_keyword)].value

        if supplied_args == expected_args:
            # Add the order to the order list.
            self.lineEditArguments.clear()
            order_item = QtGui.QTreeWidgetItem([order_keyword, order_priority, order_arguments])
            self.treeWidgetOrders.addTopLevelItem(order_item)
        else:
            # Show a notification informing the user that the order is incorrect.
            QtGui.QMessageBox.information(self, "Incorrect number of order arguments!", "Incorrect number of arguments. Supplied " + str(supplied_args) + " argument(s) (" + order_arguments + "). Expected " + str(expected_args) + " argument(s).")


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
            "Robot Co-production research group at the Industrial Design " \
            "Engineering faculty of the Delft University of Technology." \
            "<p>Version: "+VERSION+"<br/>" \
            "License: Apache License version 2.0</p>"
        QtGui.QMessageBox.about(self, "About - " + APPLICATION_TITLE + ".", text)

#endregion      ### PyQt GUI ###





def load_orders_from_JSON(filepath):
    """ Add one or multiple orders from a JSON file. """
    # Load JSON file into dictionary
    loaddata_dict = None
    with open(filepath) as json_loadfile:
        loaddata_dict = json.load(json_loadfile)
    
    # Add orders from loaded JSON dictionary to the Order tab Order list.
    for order_id in range(len(loaddata_dict)):
        order_info_dict = loaddata_dict[str(order_id)]
        item_keyword = order_info_dict["keyword"]
        item_priority = order_info_dict["priority"]
        item_order_args = order_info_dict["order_args"]
        item_arguments = ""
        for arg in item_order_args:
            item_arguments = item_arguments + " " + arg
        order_item = QtGui.QTreeWidgetItem([item_keyword, item_priority, item_arguments])
        appGui.treeWidgetOrders.addTopLevelItem(order_item)





if __name__ == '__main__':
    try:     
        # Initialize the node.
        rospy.init_node('fleet_manager_front')

        #region --- GUI ---
        app = QtGui.QApplication(sys.argv)
        appGui = GuiMainWindow()
        #windowIcon = QtGui.QIcon()
        #windowIcon.addPixmap(QtGui.QPixmap(":/icons/Launch Icon Multi.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        #appGui.setWindowIcon(windowIcon)

        # Add orders from example_orders.JSON
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

        appGui.show()
        app.exec_()
        #endregion

        # Keep node running.
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass