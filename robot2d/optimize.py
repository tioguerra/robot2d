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
SOFTWARE.
'''

import numpy as np
from .params import *
from .utils import *
import robot2d
from .robot2d import *
import sys

def calibrate_pid_gains():
    ''' This function performs some calibration (guided random search)
    for tuning the parameters of the PID, taking the current values
    as the start point. The way it works is that is generates random
    points around the current one, starting with 1.5x value changes.
    We test these random values, pick the best one, and generate new
    values again, now in a narrower neighborhood. Then we repeat
    the process with narrower and narrower neighborhood, until we
    (hopefully) converge to a nice value.
    '''
    global KPID # We use the current PID gains as our start point
    scores = {} # performance scores for each set of parameters
    pid_gains = [] # list of sets of PID gains to be tested
    max_ik = 5 # This is the number of epochs to run
    # Below we start generating PID gains that are random variations
    # with amplitude up to 1.5x the current values
    for ik,k in enumerate(np.linspace(1.5,0.05,max_ik)):
        # For this epoch, the loop below generates 15 sets
        # of random gains to test out
        pid_gains = []
        for i in range(15*4):
            # Below we generate a set of the PID gains
            # for 4 joints (the two fingers have the same
            # gain each)
            gains = []
            for joint in range(4):
                kp,ki,kd = KPID[joint] # This is the center value
                # We generate some variation around the center
                # value for each gain, always allowing a margin
                # of +1 and -1 to ensure some variability for
                # even smaller gains. We do not allow negative
                # gains.
                kp = \
                np.max([np.random.randint(int(kp-k*kp)-1,
                                          int(kp+k*kp)+1),
                        0.0],initial=0.0)
                ki = \
                np.max([np.random.randint(int(ki-k*ki)-1,
                                          int(ki+k*ki)+1),
                        0.0],initial=0.0)
                kd = \
                np.max([np.random.randint(int(kd-k*kd)-1,
                                          int(kd+k*kd)+1),
                        0.0],initial=0.0)
                gains.append((kp,ki,kd)) # The set of 4 gains
            pid_gains.append(gains) # All sets of gains of this epoch
        i = 0 # Simulation counter
        j = 0 # PID gains set counter
        w = 0 # Show graphics of the simulation every so often
        e_sum = 0.0 # Accumulated error
        robot = Robot2D() # Create a robot
        robot.enableTorque()
        show = True # This makes the robot appear on the screen
        while robot.step(show):
            if show == True:
                w = w + 1
                if w >= 350: # We only show the robot for some frames
                    w = 0
                    show = False
            e = robot.getLastError() # Get the step error
            e_sum = e_sum + sum((x*10.0)**2 for x in e) # Dum the squared error
            e_sum = e_sum + (10.0*e[-1])**2 # extra gripper error
            if i % 20 == 0 and i != 0: # Generate random joint targets
                robot.target = [(np.pi/2)*np.random.random() for i in range(3)]
                robot.target = robot.target + [-np.random.random()] + [np.random.random()]
            if i % 100 == 0 and i != 0:
                robot.disableTorque()
                robot.enableTorque()
                j = j + 1
                scores[int(e_sum*10)] = KPID
                if j == len(pid_gains):
                    # The loop below is just trying to select only the
                    # best that does not have a zero proportional gain
                    for ibest in range(len(scores)):
                        best = scores[sorted(scores.keys())[ibest]]
                        flag = False
                        for kp,ki,kd in best:
                            if kp == 0.0:
                                flag = True
                        if not flag:
                            break
                    KPID = best
                    break
                KPID = pid_gains[j]
                robot2d.KPID = KPID
                e_sum = 0
                print('%.2f done' % ((100.0 \
                                      * (j / (len(pid_gains)-1) * (1.0/float(max_ik))\
                                      + (float(ik)/float(max_ik)))))\
                                      )
            i = i + 1
    print(best)

def calibrate_gripper_gains():
    '''
    This routine tries to calibrate the gripper gains
    and other gripper related parameters
    '''
    global KPID
    global MAX_GRIPPER_FORCE
    global BOX_RESTITUTION
    scores = {}
    n1 = 10
    n2 = 50
    for i1,k in enumerate(np.linspace(1.5,0.1,n1)):
        if i1 == 0:
            seed_gains = [list(KPID[3])+ [MAX_GRIPPER_FORCE, BOX_RESTITUTION]]
        gains = []
        for gain in seed_gains:
            for i in range(n2):
                kp,ki,kd = gain[:3]
                maxForce = gain[-2]
                boxRestitution = gain[-1]
                kp = get_random_var(kp,k)
                ki = get_random_var(ki,k)
                kd = get_random_var(kd,k)
                maxForce = get_random_var(maxForce,k)
                boxRestitution = get_random_var(boxRestitution,k)
                if kp != 0.0:
                    gains.append((kp,ki,kd,maxForce,boxRestitution)) # The set of 4 gains
        gains = gains + seed_gains
        show = True
        for i2,gain in enumerate(gains):
            KPID[3] = gain[:3]
            MAX_GRIPPER_FORCE = gain[-2]
            BOX_RESTITUTION = gain[-1]
            robot2d.KPID = KPID
            robot2d.MAX_GRIPPER_FORCE = MAX_GRIPPER_FORCE
            robot2d.BOX_RESTITUTION = BOX_RESTITUTION
            robot = Robot2D(False)
            robot.createTablesEnv()
            robot.enableTorque()
            robot.startFollowingPath()
            while robot.isPathFollowing:
                robot.step(show_graphics=show)
            #show = False
            d = distance(robot.box.position, robot.goal)
            print('%.1f: Dist=%f, Gains=%s' %
                  (100.0*(((1.0/n1)*(i2+1)/len(gains))+(i1/n1)),d,gain))
            scores[int(d*10)] = gain
        best_scores = sorted(scores.keys())
        seed_gains = [scores[best_scores[i]] for i in range(3)]
        KPID[3] = scores[best_scores[0]][:3]
        MAX_GRIPPER_FORCE = scores[best_scores[0]][-2]
        BOX_RESTITUTION = scores[best_scores[0]][-1]
        robot2d.KPID = KPID
        robot2d.MAX_GRIPPER_FORCE = MAX_GRIPPER_FORCE
        robot2d.BOX_RESTITUTION = BOX_RESTITUTION
        print("BEST SO FAR:")
        for i in range(3):
            print(best_scores[i],scores[best_scores[i]])

if __name__ == '__main__':
    if sys.argv[1] == 'gripper':
        calibrate_gripper_gains()
    elif sys.argv[1] == 'pid':
        calibrate_pid_gains()
    robot = Robot2D()
    while robot.step():
        pass

