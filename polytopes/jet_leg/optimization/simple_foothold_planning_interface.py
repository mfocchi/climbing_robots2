# -*- coding: utf-8 -*-
"""
Created on Wed Dec 19 09:44:02 2018

@author: rorsolino
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


class FootholdPlanningInterface:
	def __init__(self):

		self.stride = 3

		self.com_position_to_validateW = [0., 0., 0.]  # used only for foothold planning

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

		# foothold planning
		self.footOption0 = [0., 0., 0.]
		self.footOption1 = [0., 0., 0.]
		self.footOption2 = [0., 0., 0.]
		self.footOption3 = [0., 0., 0.]
		self.footOption4 = [0., 0., 0.]
		self.footOption5 = [0., 0., 0.]
		self.footOption6 = [0., 0., 0.]
		self.footOption7 = [0., 0., 0.]
		self.footOption8 = [0., 0., 0.]
		self.footOptions = np.array([self.footOption0,
									 self.footOption1,
									 self.footOption2,
									 self.footOption3,
									 self.footOption4,
									 self.footOption5,
									 self.footOption6,
									 self.footOption7,
									 self.footOption8])

		self.numberOfFeetOptions = 0
		self.sample_contacts = np.zeros((4, 3))

		self.minRadius = 0.
		self.foothold_optimization_started = False
		# outputs
		self.option_index = 0
		self.ack_optimization_done = False
		self.TOL = 0.001

	def getParamsFromRosDebugTopic(self, received_data):

		for dir in range(0, 3):
			self.com_position_to_validateW[dir] = received_data.com_position_to_validateW[dir]

		self.numberOfFeetOptions = received_data.no_of_foothold_options

		for option in range(0,self.numberOfFeetOptions):
			self.footOptions[option] = received_data.foothold_options.data[self.stride * option: self.stride * option + 3]

		# print self.footOptions

		self.foothold_optimization_started = received_data.foothold_optimization_started

		self.minRadius = received_data.min_radius
