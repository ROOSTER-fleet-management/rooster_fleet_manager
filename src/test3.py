#! /usr/bin/env python

import time

def print_several_times():
    i = 0
    while i < 5:
        print(i)
        time.sleep(1)
        i = i+1

print_several_times()