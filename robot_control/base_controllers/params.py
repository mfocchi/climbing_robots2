# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np

robot_params = {}



robot_params['climbingrobot2'] ={'dt': 0.001,
                       'kp': np.array([0 ,    0,    400,  40,    40,   40,
                                       0 ,    0,   400,  40,    40,   40,
                                       150, 130, 60]),
                       'kd':  np.array([20,    20,    100,   20,     20,   20,
                                        20,    20,    100,   20,     20,   20,
                                        10,   10, 10  ]),
                       # this corresposnds to p = [0.03, 2.5, -6] from matlab WF  which is located in anchor_pos1 and correspods to (rope joint starts from anchor disnace /2)
                       'q_0':  np.array([ 0.0,    1.17  ,  4.0000   ,      0. ,   -1.17 ,        0.,
                                          0.0,  -1.17,    4.0000  ,       0. ,  1.17,           0.,
                                          -1.57, 0.0, -0.2 ]),
                       # 'q_0': np.array([0.0, 0, 4, 0, 0.0, 0,
                       #                    0.0, 0, 4, 0, 0, 0,
                       #                    -1.57, 0.0, 0.0]),
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
                       'spawn_2x': 0.2,
                       'spawn_2y': 5.0,
                       'spawn_2z': 20.0,
                       'wall_inclination': 0.0,
                       'buffer_size': 10000} # note the frames are all aligned with base for joints = 0

robot_params['climbingrobot2landing'] ={'dt': 0.001,
                       'kp': np.array([0 ,    0,    1200,  40,    40,   40,
                                       0 ,    0,    1200,  40,    40,   40,
                                       50, 30, 80,
                                       30, 0., 30,0.
                                       ]),
                       'kd':  np.array([20,    20,    400,   20,     20,   20,
                                        20,    20,    400,   20,     20,   20,
                                        10,   10, 30,
                                        5, 0.01, 5 ,0.01
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
                       'spawn_2x': 0.2,
                       'spawn_2y': 5.0,
                       'spawn_2z': 20.0,
                       'wall_inclination':0.1,
                       'buffer_size': 30000} # note the frames are all aligned with base for joints = 0



verbose = False
plotting = True


