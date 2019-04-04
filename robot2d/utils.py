#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Copyright 2019 Rodrigo da Silva Guerra

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

import numpy as np
from .params import *

def b2d_to_pygame(pos):
    """ Transform 2D coordinates from Box2D to PyGame

    Parameters
    ----------
    pos : tuple or list with two integers or floats
          representing coordinates in Box2D

    """
    return [int(pos[0]*PPM), HEIGHT - int(pos[1]*PPM)]

def pygame_to_b2d(pos):
    """ Transform 2D coordinates from PyGame to Box2D

    Parameters
    ----------
    pos : tuple or list with two integers or floats
          representing coordinates in PyGame

    """
    return [pos[0]/PPM, (HEIGHT - pos[1])/PPM]

def limit_angle(angle):
    """ This function limits the given angle between
    pi and -pi.

    Parameters
    ----------
    angle : a float value representing an angle in radians
    """
    return (angle + np.pi) % (2*np.pi) - np.pi

def interpolate_path(pos0, pos1, gripper, n=100):
    ''' This function create a path of n 2D point
    coordinates starting from pos0 all the way
    to pos1, using linear interpolation.

    Parameters
    ----------
    pos0 : 2-tuple representing the starting position
    pos1 : 2-tuple representing the finish position
    gripper : the value of the gripper for all the path
              (the gripper is a single value, same
               for all path, 1.0 for open, 0.0 close)
    n : number of points to generate (default is 100)
    '''
    x0,y0 = pos0
    x1,y1 = pos1
    x = np.linspace(x0, x1, n)
    y = np.linspace(y0, y1, n)
    g = [gripper]*n
    return list(zip(x,y,g))

def distance(pos0, pos1):
    '''
    This helper function simply computes the Euclidean
    distance between two points

    Parameters
    ----------
    pos0 : 2-tuple representing the first point
    pos1 : 2-tuple representing the second point
    '''
    x0,y0 = pos0
    x1,y1 = pos1
    d = np.sqrt((x1-x0)**2 + (y1-y0)**2)
    return d

def get_random_var(x,k):
    '''
    This function returns a random number between
    -k*x and +k*x, using an uniform distribution.
    The function does not allow numbers smaller than
    zero (this is to be used for tuning PID gains,
    and other positive parameters)
    '''
    xmin = np.max([x-k*x - 1.0, 0.0])
    xmax = np.max([x+k*x + 1.0, 0.0])
    xdelta = xmax - xmin
    return np.random.random()*xdelta + xmin

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
    scores = {}
    n1 = 5
    n2 = 15
    KPID[3] = (3.0,0.0,2.0)
    MAX_GRIPPER_FORCE = 546
    BOX_RESTITUTION = 0.8
    for i1,k in enumerate(np.linspace(1.5,0.1,n1)):
        if i1 == 0:
            gains = [list(KPID[3])+ [MAX_GRIPPER_FORCE, BOX_RESTITUTION]]
        for gain in gains:
            for i in range(n2):
                kp,ki,kd = gain[:3]
                maxForce = gain[-2]
                boxRestitution = gain[-1]
                kp = get_random_version(kp,k)
                ki = get_random_version(ki,k)
                kd = get_random_version(kd,k)
                maxForce = get_random_version(maxForce,k)
                boxRestitution = get_random_version(boxRestitution,k)
                if kp != 0.0:
                    gains.append((kp,ki,kd,maxForce,boxRestitution)) # The set of 4 gains
        show = True
        for i2,gain in enumerate(gains):
            KPID[3] = gain[:3]
            MAX_GRIPPER_FORCE = gain[-2]
            BOX_RESTITUTION = gain[-1]
            robot = Robot2D()
            robot.createTablesEnv()
            robot.enableTorque()
            robot.startFollowingPath()
            while robot.isPathFollowing:
                robot.step(show_graphics=show)
            show = False
            d = distance(robot.box.position, robot.goal)
            print('%.1f: Dist=%f, Gains=%s' %
                  (100.0*(((1.0/n1)*(i2+1)/n2)+(i1/n1)),d,gain))
            scores[int(d*10)] = gain
        best_scores = sorted(scores.keys())
        gains = [scores[best_scores[i]] for i in range(5)]
        KPID[3] = scores[best_scores[0]][:3]
        MAX_GRIPPER_FORCE = scores[best_scores[0]][-2]
        BOX_RESTITUTION = scores[best_scores[0]][-1]
    for i in range(5):
        print(best_scores[i],scores[best_scores[i]])

