#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  3 13:38:45 2018

@author: Abdelrahman Abdalla
"""

import copy
import numpy as np
import math as pymath
import os

import rospy as ros
import sys
import time
import threading

from copy import deepcopy

# from context import jet_leg
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.maths.computational_geometry import ComputationalGeometry
from jet_leg.maths.geometry import Geometry
from jet_leg.optimization.foothold_planning_interface import FootholdPlanningInterface
from jet_leg.maths.math_tools import Math
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics

stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr

# class Leg(Enum):
# 	LF = 0
# 	RF = 1
# 	LH = 2
# 	RH = 3
#
# class Quadrant(Enum):
# 	FRONT = 0
# 	BACK = 1
# 	LEFT = 2
# 	RIGHT = 3

class LineCoeff2d:
	def __init__(self):
		self.p = 0.0
		self.q = 0.0
		self.r = 0.0

class TCrawlFootHoldPlanning:
	def __init__(self, robot_name):
		self.robotName = robot_name
		self.compGeo = ComputationalGeometry()
		self.geometry = Geometry()
		self.compDyn = ComputationalDynamics(self.robotName)
		self.footPlanning = FootholdPlanningInterface()
		self.math = Math()
		self.dog = DogInterface()
		self.rbd = RigidBodyDynamics()

	def mapToWorld(self, R, input_B, actual_base_x):
		return np.transpose(R.transpose) * input_B + actual_base_x

	"""
	Computes the halfplane description of the polygon
	
	Parameters
    ----------
    vertices: numpy array of 3D vertices
    
    Returns
    ----------
    A, b: Half space descriprion of the polygon (assuming created with vertex sorted in CCWise order)
	"""
	def compute_half_plane_description(self, vertices):

		number_of_constraints = np.size(vertices,0)
		# vertices_ccwise_sorted = np.zeros((number_of_constraints, 3-1))
		A = np.zeros((number_of_constraints, 3))
		b = np.zeros(number_of_constraints)

		# for vertix in range(0, number_of_constraints):
		# 	vertices_ccwise_sorted[vertix] = [vertices[vertix][0], vertices[vertix][1]]
		vertices_ccwise_sorted = vertices

		# Sort feet positions
		self.geometry.counter_clockwise_sort(vertices_ccwise_sorted)

		# cycle along the ordered vertices to compute the line coeff p*xcp + q*ycp  +r  > + stability_margin
		for vertix in range(0, number_of_constraints):
			# Compute the coeffs of the line between two vertices (normal p,q pointing on the left of (P1 - P0) vector
			line_coeff = self.compute_line_coeff(vertices_ccwise_sorted[vertix],
									vertices_ccwise_sorted[(vertix + 1) % number_of_constraints])
			if (not np.isfinite(line_coeff.p)) or (not np.isfinite(line_coeff.q)):
				print "There are two coincident vertices in the polygon, there could be NaNs in the HP description matrix"

			A[vertix, 0] = line_coeff.p
			A[vertix, 1] = line_coeff.q
			A[vertix, 2] = 0.0 # Z component is not considered
			b[vertix] = line_coeff.r

		return A, b

	"""
	Compute the coefficients p,q of the line p*x + q*y + r = 0 (in 2D) passing through pt0 and pt1
	"""
	def compute_line_coeff(self, pt0, pt1):
		ret = LineCoeff2d()
		ret.p = pt0[1] - pt1[1]
		ret.q = pt1[0] - pt0[0]
		ret.r = -ret.p * pt0[0] - ret.q * pt0[1]

		# Normalize the equation in order to intuitively use stability margins (?)
		norm = pymath.hypot(ret.p, ret.q)
		ret.p /= norm
		ret.q /= norm
		ret.r /= norm

		return ret

	# def getIpsilateralLegIDinQuadrant(self, leg, quadrant):
	#
	# 	if leg == Leg.RH.value:
	# 		if quadrant == Quadrant.FRONT.value: ipsilateral_leg = Leg.RH.value
	# 		if quadrant == Quadrant.LEFT.value: ipsilateral_leg = Leg.LH.value
	# 		return ipsilateral_leg
	# 	if leg == Leg.LH.value:
	# 		if quadrant == Quadrant.FRONT.value: ipsilateral_leg = Leg.LF.value
	# 		if quadrant == Quadrant.RIGHT.value: ipsilateral_leg = Leg.RH.value
	# 		return ipsilateral_leg
	# 	if leg == Leg.LF.value:
	# 		if quadrant == Quadrant.BACK.value: ipsilateral_leg = Leg.LH.value
	# 		if quadrant == Quadrant.RIGHT.value: ipsilateral_leg = Leg.RF.value
	# 		return ipsilateral_leg
	# 	if leg == Leg.RF.value:
	# 		if quadrant == Quadrant.BACK.value: ipsilateral_leg = Leg.RH.value
	# 		if quadrant == Quadrant.LEFT.value: ipsilateral_leg = Leg.LF.value
	# 		return ipsilateral_leg

	def compute_optimal_foothold(self, foothold_params, params):

		ng = 4
		params.setConstraintModes(['ONLY_FRICTION',
								   'ONLY_FRICTION',
								   'ONLY_FRICTION',
								   'ONLY_FRICTION'])
		params.setNumberOfFrictionConesEdges(ng)

		# future_support_triangle = copy.deepcopy(foothold_params.future_support_triangle)
		future_support_triangle = foothold_params.future_support_triangle

		icp = foothold_params.icp
		swing_leg_index = foothold_params.swing_leg_index
		step_direction_wf = foothold_params.step_direction_wf
		search_step = foothold_params.search_step
		max_iterations = foothold_params.max_iterations
		alpha = 0.0
		iteration = 0

		# Set support feet for iterative projection of future support triangle
		params.stanceFeet = foothold_params.supportFeet

		# Compute the contacts in base frame
		current_contactsWF = params.getContactsPosWF().copy()
		comPositionWF = params.getCoMPosWF()
		comPositionBF = params.getCoMPosBF()
		rpy = params.getOrientation()
		contactsBF = np.zeros((4, 3))
		for j in np.arange(0, 4):
			j = int(j)
			contactsBF[j, :] = np.add(
				np.dot(self.math.rpyToRot(rpy[0], rpy[1], rpy[2]), (current_contactsWF[j, :] - comPositionWF)), comPositionBF)

		while True:

			first_time = time.time()
			alpha += search_step
			delta_step = step_direction_wf * alpha  # iterate along step direction with search step
			# Fill the support triangle with the swing foot candidate position
			# future_support_triangle[-1] = current_contactsWF[swing_leg_index] + delta_step

			# TODO compute support_triangle using projection
			# Set new candidate foot position for swing leg
			params.contactsWF[swing_leg_index] = current_contactsWF[swing_leg_index] + delta_step
			# Compute candidate future support triangle
			future_support_triangle, a_p, IP_computation_time = self.compDyn.try_iterative_projection_bretl(params)


			half_plane_time = time.time()
			A_pol, b_pol = self.compute_half_plane_description(future_support_triangle[:-1]) # Remove repeated last vertix
			# A_pol, b_pol = self.compute_half_plane_description(future_support_triangle)
			# print "half_plane_time_mirco: ", (time.time() - half_plane_time)* 1000.0 * 1000.0


			if iteration > max_iterations:
				print "icp: ", foothold_params.icp
				optimization_success = False
				print "Took too long. Using default values for stepping deltaStep"
				return delta_step, optimization_success

			condition_time = time.time()
			if self.math.is_point_inside_polygon(icp, A_pol, b_pol):
				print "icp: ", foothold_params.icp
				print "iterations: ", iteration
				optimization_success = True
				print "Success! Target footstep: ", contactsBF[swing_leg_index] + delta_step
				print "alpha: ", alpha
				return (contactsBF[swing_leg_index] + delta_step), optimization_success
			# print "condition_time_micro: ", (time.time() - condition_time) * 1000.0 * 1000.0
			# print "final_time_micro: ", (time.time() - first_time) * 1000.0 * 1000.0
			iteration += 1