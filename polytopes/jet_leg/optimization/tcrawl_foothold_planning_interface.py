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

from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.maths.computational_geometry import ComputationalGeometry
from jet_leg.maths.math_tools import Math

stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class TCrawlFootholdPlanningInterface:
	def __init__(self):

		# Foothold planning

		# Future support triangle for the ipsilateral leg (without the current swing foot)
		# self.future_support_triangle = np.array([[0.,0.,0.],
		# 										 [0.,0.,0.],
		# 										 [0.,0.,0.]])
		self.future_support_triangle = np.zeros((3, 3))
		self.supportFeet = [0, 0, 0, 0]
		# Index of the swing leg from all the 4 legs
		self.swing_leg_index = 3
		# First or second leg to swing flag. 0 for first leg and 1 for second leg
		self.leg_sequence_index = 0
		# Instantaneous capture point
		self.icp = np.zeros(3)
		# Step direction to search along for optimal foothold
		self.step_direction_wf = np.zeros(3)
		# Increment to search with along search direction
		self.search_step = 0
		# Maximum iterations for performing foothold optimization
		self.max_iterations = 0
		# Request for new optimization from framework
		self.tcrawl_optimization_start = False

		# self.sample_contacts = np.zeros((4, 3))

		self.numberOfContacts = 0

		# self.minRadius = 0.

		# outputs
		# self.option_index = 0
		self.ack_optimization_done = False

	def getSupportFeet(self):
		return self.supportFeet

	def getParamsFromRosDebugTopic(self, received_data):

		# Foothold planning
		stride = received_data.tcrawl_future_support_triangle.layout.dim[1].stride
		for foot in range(0, 3):
			self.future_support_triangle[foot] = received_data.tcrawl_future_support_triangle.data[stride * foot : stride * foot + 3]

		self.swing_leg_index = received_data.tcrawl_swing_leg_index;

		self.leg_sequence_index = received_data.tcrawl_leg_sequence_index

		for dir in range(0, 3):
			self.icp[dir] = received_data.icp[dir]

		for dir in range(0, 3):
			self.step_direction_wf[dir] = received_data.tcrawl_step_direction_wf[dir]

		self.search_step = received_data.tcrawl_search_step

		self.max_iterations = received_data.tcrawl_max_iterations

		self.tcrawl_optimization_start = received_data.tcrawl_optimization_started

	def getFutureSupportFeetFlags(self, received_data):

		no_of_legs = 4
		# Future 'support triangle ipsilateral leg'
		for leg in range(0, no_of_legs):
			self.supportFeet[leg] = int(received_data.tcrawl_future_support_feet_flag[leg])

		self.numberOfContacts = np.sum(self.supportFeet)
