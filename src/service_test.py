#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
# import time

def my_callback_1(request):
   print("first service has been called")
   i = 1
   while i < 11:
       print('first service is executing '+str(i)+' seconds')
       rospy.sleep(1)
       i = i + 1
   print("first service has been done")
   return EmptyResponse() # the service Response class, in this case EmptyResponse
   #return MyServiceResponse(len(request.words.split()))

def my_callback_2(request):
   print("second service has been called")
   i = 1
   while i < 11:
       print('second service is executing '+str(i)+' seconds')
       rospy.sleep(1)
       i = i + 1
   print("second service has been done")
   return EmptyResponse() # the service Response class, in this case EmptyResponse

def my_callback_3(request):
   print("third service has been called")
   i = 1
   while i < 11:
       print('third service is executing '+str(i)+' seconds')
       rospy.sleep(1)
       i = i + 1
   print("third service has been done")
   return EmptyResponse() # the service Response class, in this case EmptyResponse   

rospy.init_node('service_server')

my_service = rospy.Service('/my_service_1', Empty , my_callback_1) # create the Service called my_service with the defined callback
my_service = rospy.Service('/my_service_2', Empty , my_callback_2) # create the Service called my_service with the defined callback
my_service = rospy.Service('/my_service_3', Empty , my_callback_3) # create the Service called my_service with the defined callback

rospy.spin() # maintain the service open.