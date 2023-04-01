#! /usr/bin/env python3
import rospy
from mpc_Town04 import mpc_core

if __name__ == '__main__':
    mp = mpc_core() 
    mp.cal_vel()
