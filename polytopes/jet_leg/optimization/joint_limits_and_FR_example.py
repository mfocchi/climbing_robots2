# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Abdelrahman Abdalla
"""

import numpy as np

from jet_leg.plotting.plotting_tools import Plotter
import random
from jet_leg.maths.math_tools import Math
from jet_leg.kinematics import kinematics_interface
from jet_leg.maths.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization import nonlinear_projection
from shapely.geometry import Polygon
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics

import matplotlib.pyplot as plt

plt.close('all')

robot_name = 'hyq'
projection = nonlinear_projection.NonlinearProjectionBretl(robot_name)
math = Math()


trunk_mass = 45.
mu = 0.5

# comWF = np.array([.55, 0.005, 0.5])  # pos of COM in world frame
comWF = np.array([0.0107, 0.0003, 0.5])
comBF = np.array([0.0094, 0.0002, -0.0433])  # pos of COM in body frame
rpy = np.array([0.00012, 0.00601, 3.6e-05])  # orientation of body frame

""" contact points in the World Frame"""
# LF_foot = np.array([0.9, 0.3, 0.02])
# RF_foot = np.array([1., -0.3, 0.02])
# LH_foot = np.array([0.1, 0.3, 0.02])
# RH_foot = np.array([0.2, -0.3, 0.02])
LF_foot = np.array([0.36, 0.32, 0.02])  # Starting configuration
RF_foot = np.array([0.36, -0.32, 0.02])
LH_foot = np.array([-0.36, 0.32, 0.02])
RH_foot = np.array([-0.36, -0.32, 0.02])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

''' stanceFeet vector contains 1 if the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1, 1, 1, 1]

tripleStance = False
randomSwingLeg = random.randint(0, 3)  # if you want you can define a swing leg using this variable

if tripleStance:
    print 'Swing leg', randomSwingLeg
    stanceFeet[randomSwingLeg] = 0
print 'stanceLegs ', stanceFeet

constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION']

''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
axisZ= np.array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # LF
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # RF
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # LH
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # RH
normals = np.vstack([n1, n2, n3, n4])

''' number of generators, i.e. rays/edges used to linearize the friction cone '''
ng = 4

''' torque limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
HAA = Hip Abduction Adduction
HFE = Hip Flextion Extension
KFE = Knee Flextion Extension
'''
LF_tau_lim = [60.0, 60.0, 60.0] # HAA, HFE, KFE
RF_tau_lim = [60.0, 60.0, 60.0] # HAA, HFE, KFE
LH_tau_lim = [60.0, 60.0, 60.0] # HAA, HFE, KFE
RH_tau_lim = [60.0, 60.0, 60.0] # HAA, HFE, KFE
torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForceW = np.array([0.0, 0.0, 0.0]) # units are Nm

''' joint position limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
HAA = Hip Abduction Adduction
HFE = Hip Flextion Extension
KFE = Knee Flextion Extension
'''
LF_q_lim_max = [0.44, 1.2217, -0.3491]  # HAA, HFE, KFE
LF_q_lim_min = [-1.22, -0.8727, -2.4435]  # HAA, HFE, KFE
RF_q_lim_max = [0.44, 1.2217, -0.3491]  # HAA, HFE, KFE
RF_q_lim_min = [-1.22, -0.8727, -2.4435]  # HAA, HFE, KFE
LH_q_lim_max = [0.44, 0.8727, 2.4435]  # HAA, HFE, KFE
LH_q_lim_min = [-1.22, -1.2217, 0.3491]  # HAA, HFE, KFE
RH_q_lim_max = [0.44, 0.8727, 2.4435]  # HAA, HFE, KFE
RH_q_lim_min = [-1.22, -1.2217, 0.3491]  # HAA, HFE, KFE
joint_limits_max = np.array([LF_q_lim_max, RF_q_lim_max, LH_q_lim_max, RH_q_lim_max])
joint_limits_min = np.array([LF_q_lim_min, RF_q_lim_min, LH_q_lim_min, RH_q_lim_min])

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters()
comp_dyn = ComputationalDynamics(robot_name)

params.setContactsPosWF(contactsWF)
params.setCoMPosWF(comWF)
params.setCoMPosBF(comBF)
params.setOrientation(rpy)
params.setJointLimsMax(joint_limits_max)
params.setJointLimsMin(joint_limits_min)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(trunk_mass)
params.externalForceWF = extForceW  # params.externalForceWF is actually used anywhere at the moment

'''-----------------------------------------------------------------------------------------'''


com_check = np.array([0.3, -0.1, 0.5])
# polygon, computation_time = projection.project_polytope(params, com_check)
# polygon, computation_time = projection.project_polytope(params, com_check, 20. * np.pi / 180)
polygon, computation_time = projection.project_polytope(params)
print "vertices", polygon
print "Computation Time: ", computation_time, " seconds"

''' compute iterative projection 
Outputs of "iterative_projection_bretl" are:
IP_points = resulting 2D vertices
actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
computation_time = how long it took to compute the iterative projection
'''
FR_polygon, force_polytopes, FR_computation_time = comp_dyn.iterative_projection_bretl(params)

joint_lim_polygon = Polygon(polygon)
# test_polygon = Polygon([(-0.215, -0.34), (0.225, -0.34), (0.225, 0.34), (-0.215, 0.34)])
pFR_polygon = Polygon(FR_polygon)
# final = joint_lim_polygon.intersection(test_polygon)
final = joint_lim_polygon.intersection(pFR_polygon)


# test_polygon = np.array(test_polygon.exterior.coords)
FR_polygon = np.array(pFR_polygon.exterior.coords)
final = np.array(final.exterior.coords)
print "final: ", final

plt.close()
h1 = plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plotter = Plotter()
plt.legend()


nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)

for j in range(0, nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    h1 = plt.plot(contactsWF[idx, 0], contactsWF[idx, 1], 'ko', markersize=15, label='stance feet')
h2 = plotter.plot_polygon(np.transpose(polygon), '--b', 'Iterative Projection')


for j in range(0, nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    h3 = plt.plot(contactsWF[idx, 0], contactsWF[idx, 1], 'ko', markersize=15, label='stance feet')
h4 = plotter.plot_polygon(np.transpose(FR_polygon), '--g', 'Iterative Projection')

plt.figure()

for j in range(0, nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    h5 = plt.plot(contactsWF[idx, 0], contactsWF[idx, 1], 'ko', markersize=15, label='stance feet')
h6 = plotter.plot_polygon(np.transpose(final), '--b', 'Iterative Projection')

plt.show()
