#! /usr/bin/env python

import subprocess
import time
import os
import sys

""" print('start of the scripts')

#os.system("python test1.py")
process = subprocess.Popen("gnome-terminal -x python test1.py", stdout=subprocess.PIPE, stderr=None,shell=True)
print('#1 is running')


print('sleeping')

time.sleep(5)

process = subprocess.Popen("gnome-terminal -x python test2.py", stdout=subprocess.PIPE, stderr=None,shell=True)
#os.system("gnome-terminal -x python test2.py")
print('#2 is running')

print('finish of the scripts') """

from multiprocessing import Process
import sys

rocket = 0

def func1():
    global rocket
    print ('start func1')
    while rocket < 20:
        rocket += 1
        time.sleep(1)
    print ('end func1')

def func2():
    global rocket
    print ('start func2')
    while rocket < 5:
        rocket += 1
        time.sleep(1)  
    #input('Press any key to finish func2')
    print ('end func2')

def count(how_long_count):
    i = 1
    print("Counting")
    while i < how_long_count:
        print(i)
        i=i+1

if __name__=='__main__':
    p1 = Process(target=count(10))
    p1.start()
    print("started process p1")
    p2 = Process(target=count(10))
    p2.start()
    print("started process p2")