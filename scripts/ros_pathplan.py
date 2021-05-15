#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.patches as ptch
import matplotlib
from matplotlib import cm
import numpy as np
import math
import time
import csv
from termcolor import colored

import sys  
sys.path.append('/home/chihunen/catkin_ws/src/ros_pathplan/src/Python-RVO2-3D') 
import rvo23d
# # sys.path.insert(0,'..')
# # from Config import Config
# from Config_ORCA import Config
# import benchmark_3D_ORCA

# import os
# path = os.path.dirname(os.path.realpath('__file__'))
# address = path + '/ORCA_result' + Config.Data_file

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from outdoor_gcs.msg import PathPlan


class O_PathPlan(object):

    def __init__(self, dt):

        self.dt = dt
        self.change_time = rospy.Time.now()

        self.cur_pos = np.zeros((15)) # max 5 drones, each with xyz
        self.des_pos = np.zeros((15))
        self._dist_to_goal = {}

        self.start_sim = False
        self.agents_num = 0
        self.uavs_id = [False,False,False,False,False]

        self.radius = 0.8
        self.MaxVelo = 2.0 #10.0
        # self.MaxAcc = 10.0
        # self.d_star = self.radius
        # self.c1 = 7*4.5
        # self.c2 = 9*4.5
        # self.RepulsiveGradient = 7.5*100
        self.neighborDist, self.timeHorizon, self.velocity = 3.0, 2, (0,0,0)

        self.sim = rvo23d.PyRVOSimulator(self.dt, self.neighborDist, self.agents_num,
                        self.timeHorizon, self.radius, self.MaxVelo, self.velocity)

        self.pathplan_pub_msg, self.pathplan_pub_msg_nxt = PathPlan(), PathPlan()
        rospy.Subscriber("/uavs/pathplan", PathPlan, self.pathplan_callback)
        self.pathplan_pub = rospy.Publisher("/uavs/pathplan_nxt", PathPlan, queue_size=1)


    def update_nxtpos(self):
        self.pathplan_pub_msg_nxt.header.stamp = rospy.Time.now()
        nxt, j = [], 0
        for i in range(len(self.uavs_id)):
            if (self.uavs_id[i]):
                nxt.extend([self.sim.getAgentPosition(j)[0], self.sim.getAgentPosition(j)[1], self.sim.getAgentPosition(j)[2]])
                j+=1
            else:
                nxt.extend([0,0,0])
        self.pathplan_pub_msg_nxt.nxt_position = list(nxt)
        # print(self.pathplan_pub_msg.nxt_position)

    def publish_msg(self):
        self.pathplan_pub.publish(self.pathplan_pub_msg_nxt)

    def iteration(self, event):
        if (self.start_sim and rospy.Time.now()-self.change_time > rospy.Duration(secs=5)):
            self.change_time = rospy.Time.now()
            self.pathplan_pub_msg_nxt.start, self.start_sim = False, False
            for i in range(len(self.uavs_id)):
                # print(self.agents_num, self.sim.getNumAgents())
                # print(self.sim.getNumAgents())
                # print('\n')
                if (self.uavs_id[i] and self.sim.getNumAgents()!=self.agents_num):
                    self.sim.addAgent((self.cur_pos[i*3], self.cur_pos[i*3+1], self.cur_pos[i*3+2]), 
                        self.neighborDist, self.agents_num, self.timeHorizon, self.radius, self.MaxVelo, self.velocity)
        if (self.agents_num != 0):
            j = 0
            # print(self.sim.getNumAgents())
            for i in range(len(self.uavs_id)):
                if (self.uavs_id[i]):
                    self.sim.setAgentPosition(j, (self.cur_pos[i*3], self.cur_pos[i*3+1], self.cur_pos[i*3+2]))
                    vel = self.des_pos[i*3:i*3+3] - self.cur_pos[i*3:i*3+3]
                    nor = np.linalg.norm(vel)
                    if nor < 10**-5:
                        prefV[0], prefV[1], prefV[2] = 0.0, 0.0, 0.0
                    else:
                        prefV = vel/nor*self.MaxVelo
                    self.sim.setAgentPrefVelocity(j, (prefV[0], prefV[1], prefV[2]))
                    j+=1
            self.sim.doStep()
            self.update_nxtpos()
            self.publish_msg()
    
    def pathplan_callback(self, msg):
        self.pathplan_pub_msg = msg
        self.start_sim = msg.start
        self.agents_num = msg.num
        self.uavs_id = msg.uavs_id
        self.cur_pos = np.asarray(msg.cur_position)
        self.des_pos = np.asarray(msg.des_position)
        self.timeHorizon = msg.params[0]
        self.MaxVelo = msg.params[1]
        self.radius = msg.params[2]
        self.neighborDist = msg.params[4]


if __name__ == '__main__':

      rospy.init_node('ros_pathplan', anonymous=True)
      dt = 1.0/15
      pathplan_run = O_PathPlan(dt)
      rospy.Timer(rospy.Duration(dt), pathplan_run.iteration)
      rospy.spin()


     


