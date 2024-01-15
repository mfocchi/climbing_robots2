#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  3 13:38:45 2018

@author: Abdelrahman Abdalla
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

# import copy
import numpy as np
import os

# import rospy as ros
import sys
# import time
# import threading
#
from copy import deepcopy

from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.optimization.nonlinear_projection import NonlinearProjectionBretl
from jet_leg.maths.computational_geometry import ComputationalGeometry
from jet_leg.optimization.foothold_planning_interface import FootholdPlanningInterface
from jet_leg.maths.math_tools import Math
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from shapely.geometry import Polygon, LineString, LinearRing
from shapely.geos import TopologicalError

stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class OrientationPlanning:
	def __init__(self, robot_name):
		self.robotName = robot_name
		self.compGeo = ComputationalGeometry()
		self.compDyn = ComputationalDynamics(self.robotName)
		self.kinProj = NonlinearProjectionBretl(self.robotName)
		self.footPlanning = FootholdPlanningInterface()
		self.math = Math()
		self.dog = DogInterface()
		self.rbd = RigidBodyDynamics()

	def optimize_orientation(self, orient_plan_params, params):
		ng = 4
		params.setConstraintModes(['FRICTION_AND_ACTUATION',
								   'FRICTION_AND_ACTUATION',
								   'FRICTION_AND_ACTUATION',
								   'FRICTION_AND_ACTUATION'])

		params.setNumberOfFrictionConesEdges(ng)
		#
		# params.setCoMPosWF(footPlanningParams.com_position_to_validateW)

		#        print numberOfFeetOptions

		delta_angle = 0.0872665 # Increment of 5 degrees
		target_CoM_WF = orient_plan_params.get_target_CoM_WF()

		# Initialize return variables
		optimal_orientation = default_orientation = orient_plan_params.get_default_orientation()
		optimal_distance = old_distance = -1
		old_area = -1
		optimal_index = -1
		feasible_regions = []
		min_distances = []
		max_areas= []

		first_roll = -0.174533  # -10 degrees
		first_pitch = -0.174533  # -10 degrees

		roll_list = [first_roll + i*delta_angle for i in range(orient_plan_params.no_of_angle_choices+1)]
		pitch_list = [first_pitch + i*delta_angle for i in range(orient_plan_params.no_of_angle_choices+1)]

		for new_roll in roll_list:
			for new_pitch in pitch_list:

				new_orientation = default_orientation + np.array([new_roll, new_pitch, 0])
				# This is all to get the contactsBF and use it then in joint projection (to find contactsWF)
				params.setOrientation(new_orientation)

				# Compute feasible region, reachable region, and intersection
				try:
					feasible_region, actuation_polygons, computation_time = self.compDyn.try_iterative_projection_bretl(params)
				except ValueError as e:
					feasible_region = False
					if hasattr(e, 'message'):
						print((e.message))
					else:
						print(e)
				if feasible_region is False:
					feasible_regions.append(False)
					min_distances.append(False)
					max_areas.append("FR False")
					# print "Feasible region is false"
					continue

				reachability_polygon, computation_time_joint = self.kinProj.project_polytope(params, target_CoM_WF, 25 * np.pi / 180, 0.03)
				if reachability_polygon.size <= 0:
					feasible_regions.append(False)
					min_distances.append(False)
					max_areas.append("Reach False")
					# print "Kinematic reach region is false"
					continue

				extended_feasible_region = Polygon(feasible_region)
				p_reachability_polygon = Polygon(reachability_polygon)
				try:
					reachable_feasible_polygon = extended_feasible_region.intersection(p_reachability_polygon)
				except TopologicalError:
					feasible_regions.append(False)
					min_distances.append(False)
					max_areas.append("Intersec Fail")
					continue

				area = reachable_feasible_polygon.area

				# Exclude if path from current CoM to target one lies outside region
				l_CoM_path = LineString([params.getCoMPosWF(), target_CoM_WF])
				try:
					if not l_CoM_path.within(reachable_feasible_polygon):
						feasible_regions.append(reachable_feasible_polygon)
						min_distances.append(-1)
						max_areas.append(area)
						# print "Path outside of region"
						continue
				except TopologicalError:
					feasible_regions.append(False)
					min_distances.append(False)
					max_areas.append("Extended !=Polygon")
					continue


				# Compute minimum distance between path and polygon
				r_reachable_feasible_polygon = LinearRing(reachable_feasible_polygon.exterior.coords)
				distance = r_reachable_feasible_polygon.distance(l_CoM_path)

				feasible_regions.append(reachable_feasible_polygon)
				min_distances.append(distance)
				max_areas.append(area)

				# Update optimal distance
				if distance - old_distance > 0.02 or (abs(distance - old_distance) <= 0.02 and area > old_area):
					optimal_orientation = new_orientation
					optimal_index = len(feasible_regions) - 1
					old_distance = distance
					old_area = area

					# optimal_orientation = new_orientation
					# optimal_index = len(feasible_regions) - 1
					# print "optimal_index: ", optimal_index
					# print "better_orientation: ", new_orientation, ", better_distance: ", distance
					# print "better_region: ", np.array(reachable_feasible_polygon.exterior.coords)

		return feasible_regions, min_distances, max_areas, optimal_orientation, optimal_index
		# newArea = self.compGeo.computePolygonArea(IAR)