# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np

robot_params = {}


robot_params['ur5'] ={'dt': 0.001, 
                       'kp': np.array([300, 300, 300,30,30,1]),
                       'kd':  np.array([20,20,20,5, 5,0.5]),
                       #'q_0':  np.array([ 0.3, -1.3, 1.0, -0.7, 0.7, 0.5]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                       'q_0':  np.array([ -0.3223527113543909,-0.7805794638446351, -2.48,-1.6347843609251917, -1.5715253988849085, -1.0017417112933558]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                       'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
                       'ee_frame': 'tool0',
                       'control_mode': 'point', # 'trajectory','point'
                       'real_robot': False,
                       'control_type': 'position', # 'position', 'torque'
                       'gripper_sim': True,
                       'spawn_x' : 0.5,
                       'spawn_y' : 0.35,
                       'spawn_z' : 1.75,
                       'buffer_size': 50000} # note the frames are all aligned with base for joints = 0

robot_params['climbingrobot2'] ={'dt': 0.001,
                       'kp': np.array([0 ,    0,    400,  40,    40,   40,
                                       0 ,    0,    400,  40,    40,   40,
                                       50, 30, 60]),
                       'kd':  np.array([20,    20,    100,   20,     20,   20,
                                        20,    20,    100,   20,     20,   20,
                                        10,   10, 10  ]),
                        # this corresposnds to p = [0.03, 2.5, -6] from matlab WF  which is located in anchor_pos1
                       'q_0':  np.array([ 0.0,    1.17  ,  4.0000   ,      0. ,   -1.17 ,        0.,
                                          0.0,  -1.17,    4.0000  ,       0. ,  1.17,           0.,
                                          -1.57, 0.0, -0.2 ]),
                       'Ko': np.array([10, 10, 10]),
                       'Do': np.array([0.1, 0.01, 0.01]),
                       'joint_names': ['mountain_wire_pitch_r', 'mountain_wire_roll_r',  'wire_base_prismatic_r',
                                       'wire_base_pitch_r', 'wire_base_roll_r','wire_base_yaw_r',
                                        'mountain_wire_pitch_l', 'mountain_wire_roll_l',  'wire_base_prismatic_l',
                                       'wire_base_pitch_l', 'wire_base_roll_l','wire_base_yaw_l',
                                       'hip_pitch', 'hip_roll', 'knee'],
                       'ee_frame': 'foot',
                       'spawn_x' : 0.2,
                       'spawn_y' : 0.0,
                       'spawn_z' : 20.0,
                       'spawn_2x': 0.2,  # should be the same as spawn_z
                       'spawn_2y': 5.0,
                       'spawn_2z': 20.0, # should be the same as spawn_z
                       'buffer_size': 10000} # note the frames are all aligned with base for joints = 0

robot_params['climbingrobot2landing'] ={'dt': 0.001,
                       'kp': np.array([0 ,    0,    1200,  40,    40,   40,
                                       0 ,    0,    1200,  40,    40,   40,
                                       50, 30, 80,
                                       60, 0., 60,0.
                                       ]),
                       'kd':  np.array([20,    20,    400,   20,     20,   20,
                                        20,    20,    400,   20,     20,   20,
                                        10,   10, 30,
                                        10, 3, 10 ,3
                                        ]),
                       # this corresposnds to p = [0.03, 2.5, -6] from matlab WF  which is located in anchor_pos1
                       'q_0':  np.array([ 0.0,    1.17  ,  4.0000   ,      0. ,   -1.17 ,        0.,
                                          0.0,  -1.17,    4.0000  ,       0. ,  1.17,           0.,
                                          -1.57, 0.0, 0.0,
                                          0., -0, 0.,0.
                                          ]),
                       'Ko': np.array([10, 10, 10]),
                       'Do': np.array([1, 1, 1]),
                       'joint_names': ['mountain_wire_pitch_r', 'mountain_wire_roll_r',  'wire_base_prismatic_r',
                                       'wire_base_pitch_r', 'wire_base_roll_r','wire_base_yaw_r',
                                        'mountain_wire_pitch_l', 'mountain_wire_roll_l',  'wire_base_prismatic_l',
                                       'wire_base_pitch_l', 'wire_base_roll_l','wire_base_yaw_l',
                                       'hip_pitch', 'hip_roll', 'knee',
                                       'hip_yaw_landing_l', 'wheel_joint_l', 'hip_yaw_landing_r', 'wheel_joint_r'],
                       'ee_frame': 'foot',
                       'spawn_x' : 0.2,
                       'spawn_y' : 0.0,
                       'spawn_z' : 20.0,
                       'spawn_2x': 0.2, # should be the same as spawn_x
                       'spawn_2y': 5.0,
                       'spawn_2z': 20.0, # should be the same as spawn_z
                        'wall_inclination':0.1,
                       'buffer_size': 10000} # note the frames are all aligned with base for joints = 0



verbose = False
plotting = True


