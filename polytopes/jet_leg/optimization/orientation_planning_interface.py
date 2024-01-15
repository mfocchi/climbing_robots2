# -*- coding: utf-8 -*-
"""
Created on Wed Dec 19 09:44:02 2018

@author: Abdelrahman Abdalla
"""

import copy
import numpy as np
import os

import rospy as ros
import sys
import time
import threading

from copy import deepcopy

stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class OrientationPlanningInterface:
	def __init__(self):

		self.stride = 3

		# Default orientation to optimize around
		self.default_orientation = np.array([])  # used only for foothold planning
		# Target CoM based on reachability heuristic
		self.target_CoM_WF = [0.,0.,0.]


		self.orientation0 = [0., 0., 0.]
		self.orientation1 = [0., 0., 0.]
		self.orientation2 = [0., 0., 0.]
		self.orientation3 = [0., 0., 0.]
		self.orientation4 = [0., 0., 0.]
		self.orientationOptions = np.array([self.orientation0,
											self.orientation1,
											self.orientation2,
											self.orientation3,
											self.orientation4])

		# For each orientation axis. Total options = (no_of_angle_choices + 1) * 2
		self.no_of_angle_choices = 0

		self.orient_optimization_started = False

		# outputs
		self.option_index = 0
		self.ack_optimization_done = False

	def get_default_orientation(self):
		return self.default_orientation

	def get_target_CoM_WF(self):
		return self.target_CoM_WF

	def no_of_angle_choices(self):
		return self.no_of_angle_choices

	def getParamsFromRosDebugTopic(self, received_data):

		self.default_orientation = np.array(received_data.default_orientation)

		for dir in range(0, 3):
			self.target_CoM_WF[dir] = received_data.target_CoM_WF[dir]

		self.no_of_angle_choices = received_data.no_of_angle_choices

		self.orient_optimization_started = received_data.orient_optimization_started
