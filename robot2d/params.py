#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Copyright 2019 Rodrigo da Silva Guerra

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.  '''

PPM = 15.0 # pixels per meter
FPS = 60
TIME_STEP = 1.0 / FPS
WIDTH, HEIGHT = 1024, 768
CENTER = WIDTH / (2*PPM), HEIGHT / (2*PPM)
ROBOTY = 1.65*CENTER[0] # placement of robot on the horizontal axis
COLORS = {
    'SKY': (64, 128, 128+32, 255),
    'BOX': (128+64, 128+64, 64, 255),
    'TABLE': (128+64+32, 64, 128, 255),
    'GROUND': (128, 64+32, 64+32, 255),
    'ROBOT': (64, 128+64, 64+32, 255),
    'GRIPPER': (64+16, 128+64+16, 64+32, 255),
    'JOINTSON': (64+32, 128+64+32, 64+64, 255),
    'JOINTSOFF': (64-32, 128+64-32, 64, 255),
    'RED': (255, 64, 32, 255),
    'PATHOFF': (64+32, 128+16, 128+16, 255),
    'PATHON': (64+32, 128+64, 128+64, 255),
    'TARGET': (128+64, 128+32, 128, 255),
    'CURRPOS': (64, 128+64+32, 64, 255),
}
BASE_LENGTH = 5
ROBOT_WIDTH = 1.5
LINK0_LENGTH = 10.0
LINK1_LENGTH = 8.0
LINK2_LENGTH = 1.0
LINK3_LENGTH = 0.35
LINK3_WIDTH = 2.5
FINGER_LENGTH = 1.25
FINGER_WIDTH = 0.35
JOINT_RADIUS = int(PPM*ROBOT_WIDTH)+1 # this is in PyGame pixel units
FORCE_SCALE = 35 # scale of the isDragging force
MAX_JOINT_TORQUE = 10000000
MAX_GRIPPER_FORCE = 39
BOX_RESTITUTION = 0.48
USE_WELD_GRASP = True # if true, will use welding joints for grasping

KPID = [(27.0, 0.0, 0.0), (19.0, 0.0, 1.0),  # PID gains for the arm joints
        (19.0, 0.0, 1.0),                   # PID gain for the wrist
        (2.17, 0.00, 0.63)]                  # PID gain for the gripper
#        (2.17, 6.72, 0.63)]                  # PID gain for the gripper

