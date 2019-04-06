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

import pygame
from pygame.locals import *
from Box2D.b2 import *
import numpy as np
from .params import *
from .utils import *

# Here the code that defines the robot simulation class

class Robot2D:
    """ This class defines a 2D robot simulation. To use it
    you just need to instantiate an object and call the
    step() method, in a loop. The step() method returns
    a boolean True while the simulation is running. If the
    user closes the window, then the step method returns
    False. This can be used to know when the simulation
    has finished.

    Example
    -------

    r = Robot2D

    while r.step():
        pass
    """
    def __init__(self):
        """ This constructor takes no parameters. Just initializes
        data structures related to GUI and physics simulation.
        """
        # PyGame window setup
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
        pygame.display.set_caption('Simple 2D Robot Simulator')
        # PyGame Clock
        self.clock = pygame.time.Clock()
        # Box2D World
        self.world = world(gravity=(0,-10), doSleep=True)
        # Box2D ground (static body)
        self.ground_body = self.world.CreateStaticBody(
            position=(0, 1),
            shapes=polygonShape(box=(3*CENTER[0],5)),
        )
        # A flag to tell if the simulation is still running
        self.running = True
        # This will hold the color mapping
        self.colors = {}
        self.colors[self.ground_body] = COLORS['GROUND']
        # This list holds references to all simulation bodies
        self.bodies = [self.ground_body]
        # This list will hold all joints
        self.joints = []
        # Create the robot
        self._createRobot()
        # Set torque
        self.isTorqueEnabled = False
        # Setting up GUI mouse isDragging vars and flags
        self.pos = 0,0 # used for isDragging and isDrawing mouse coordinates
        self.isDragging = False
        self.isDrawing = False
        self.gripper = False
        # Set also some dynamic interaction related  vars and flags
        self.isPathFollowing = False
        self.path_counter = 0.0
        self.draggedBodies = {}
        self.use_weld_grasp = USE_WELD_GRASP # Flag for joint based grasping
        # Some lists to hold things to be drawn on the screen
        self.dots = []
        self.lines = []
        self.path = []
    def _createRobot(self):
        ''' This method creates the robot itself, creating the bodies,
        joints, and geometries (fixtures) using Box2D. The method is
        supposed to be called from the constructor only.
        '''
        # Create the base
        self.robot_base0_body = self.world.CreateStaticBody(position=(ROBOTY,\
                                                                      3.8))
        self.robot_base0_body.CreatePolygonFixture(box=(4,3))
        self.robot_base1_body = self.world.CreateStaticBody( \
                                                position=(ROBOTY,6+BASE_LENGTH))
        self.robot_base1_body.CreatePolygonFixture(box=(ROBOT_WIDTH,LINK0_LENGTH))
        # Create the first link
        self.robot_link0_body = self.world.CreateDynamicBody(
                                                position=(ROBOTY, \
                                                          6+BASE_LENGTH+2*LINK0_LENGTH))
        self.robot_link0_body.CreatePolygonFixture(box=(ROBOT_WIDTH,LINK0_LENGTH), \
                                                density=2, friction=0.2)
        # Create the second link
        self.robot_link1_body = self.world.CreateDynamicBody( \
                                                position=(ROBOTY,\
                                                          6+BASE_LENGTH \
                                                          +3*LINK0_LENGTH+LINK1_LENGTH))
        self.robot_link1_body.CreatePolygonFixture(box=(ROBOT_WIDTH,LINK1_LENGTH), \
                                                   density=0.5, friction=0.2)
        # Create the third link (wrist)
        self.robot_link2_body = self.world.CreateDynamicBody( \
                                                position=(ROBOTY,\
                                                          6+BASE_LENGTH \
                                                          +3*LINK0_LENGTH \
                                                          +2*LINK1_LENGTH+LINK2_LENGTH))
        self.robot_link2_body.CreatePolygonFixture(box=(ROBOT_WIDTH,LINK2_LENGTH), \
                                                   density=0.125, friction=0.2)
        # This fourth link is just a bar-like shape to simulate
        # the gripper attachment. This body will be welded to the
        # third link.
        self.robot_link3_body = self.world.CreateDynamicBody( \
                                                position=(ROBOTY,\
                                                          6+BASE_LENGTH \
                                                          +3*LINK0_LENGTH \
                                                          +2*LINK1_LENGTH \
                                                          +2*LINK2_LENGTH+LINK3_LENGTH))
        self.robot_link3_body.CreatePolygonFixture(box=(LINK3_WIDTH,LINK3_LENGTH), \
                                                   density=0.01, friction=0.2)
        # This is the left finger (if you look gripper facing downwards)
        self.robot_link4_body = self.world.CreateDynamicBody( \
                                                position=(ROBOTY-FINGER_WIDTH/2.0,\
                                                          6+BASE_LENGTH \
                                                          +3*LINK0_LENGTH \
                                                          +2*LINK1_LENGTH \
                                                          +2*LINK2_LENGTH \
                                                          +2*LINK3_LENGTH \
                                                          +FINGER_LENGTH))
        self.robot_link4_body.CreatePolygonFixture(box=(FINGER_WIDTH,FINGER_LENGTH), \
                                                   density=0.01, friction=100.0)
        # This is the right finger (if you look gripper facing downwards)
        self.robot_link5_body = self.world.CreateDynamicBody( \
                                                position=(ROBOTY+FINGER_WIDTH/2.0,\
                                                          6+BASE_LENGTH \
                                                          +3*LINK0_LENGTH \
                                                          +2*LINK1_LENGTH \
                                                          +2*LINK2_LENGTH \
                                                          +2*LINK3_LENGTH \
                                                          +FINGER_LENGTH))
        self.robot_link5_body.CreatePolygonFixture(box=(FINGER_WIDTH,FINGER_LENGTH), \
                                                   density=0.01, friction=100.0)
        # This is the first actuated joint
        self.j0 = self.world.CreateRevoluteJoint(bodyA=self.robot_base1_body,
                                            bodyB=self.robot_link0_body,
                                            anchor=(ROBOTY,6+BASE_LENGTH+LINK0_LENGTH),
                                            collideConnected = False,
                                            maxMotorTorque=MAX_JOINT_TORQUE)
        # This is the second actuated joint
        self.j1 = self.world.CreateRevoluteJoint(bodyA=self.robot_link0_body,
                                            bodyB=self.robot_link1_body,
                                            anchor=(ROBOTY,
                                                    6+BASE_LENGTH+3*LINK0_LENGTH),
                                            collideConnected = False,
                                            maxMotorTorque=MAX_JOINT_TORQUE)
        # This is the third actuated joint (wrist). It may seem like the
        # wrist is just dangling by gravity, but it is actually being
        # actuated.
        self.j2 = self.world.CreateRevoluteJoint(bodyA=self.robot_link1_body,
                                            bodyB=self.robot_link2_body,
                                            anchor=(ROBOTY,
                                                    6+BASE_LENGTH+3*LINK0_LENGTH+2*LINK1_LENGTH),
                                            collideConnected = False,
                                            maxMotorTorque=MAX_JOINT_TORQUE)
        # This is a welding joint (not really a "joint", because
        # these bodies are not supposed to move relative to each
        # other)
        self.j3 = self.world.CreateWeldJoint(bodyA=self.robot_link2_body,\
                                             bodyB=self.robot_link3_body,\
                                             anchor=(ROBOTY,\
                                                     6+BASE_LENGTH\
                                                     +3*LINK0_LENGTH\
                                                     +2*LINK1_LENGTH\
                                                     +2*LINK2_LENGTH),
                                             collideConnected = False)
        # We define this propertie for later reference
        self.gripperMax = LINK3_WIDTH-1.5*FINGER_WIDTH
        # This is the left finger prismatic (linear translation) joint
        self.j4 = self.world.CreatePrismaticJoint(bodyA=self.robot_link3_body,\
                                                  bodyB=self.robot_link4_body,\
                                                  anchor=(ROBOTY,\
                                                          6+BASE_LENGTH\
                                                          +3*LINK0_LENGTH\
                                                          +2*LINK1_LENGTH\
                                                          +2*LINK2_LENGTH\
                                                          +2*LINK3_LENGTH),
                                                  axis=(1,0),
                                                  lowerTranslation=-self.gripperMax,
                                                  upperTranslation=0.0,
                                                  enableLimit = True,
                                                  collideConnected = False,
                                                  maxMotorForce=MAX_GRIPPER_FORCE)
        # This is the right finger prismatic (linear translation) joint
        self.j5 = self.world.CreatePrismaticJoint(bodyA=self.robot_link3_body,
                                                  bodyB=self.robot_link5_body,
                                                  anchor=(ROBOTY,
                                                          6+BASE_LENGTH\
                                                          +3*LINK0_LENGTH\
                                                          +2*LINK1_LENGTH\
                                                          +2*LINK2_LENGTH\
                                                          +2*LINK3_LENGTH),
                                                  axis=(1,0),
                                                  lowerTranslation=0.0,
                                                  upperTranslation=LINK3_WIDTH\
                                                                   -1.5*FINGER_WIDTH,
                                                  enableLimit = True,
                                                  collideConnected = False,
                                                  maxMotorForce=MAX_GRIPPER_FORCE)
        # Create a gear joint to make the fingers move together
        # This means that one finger will always move with the other
        self.j6 = self.world.CreateGearJoint(bodyA=self.robot_link4_body,
                                             bodyB=self.robot_link5_body,
                                             joint1=self.j4,
                                             joint2=self.j5,
                                             ratio = 1.0)
        # This is for grasping (placeholder)
        self.j7 = None
        # Append the bodies to our queue for isDrawing later
        self.robot_bodies = []
        self.robot_bodies.append(self.robot_base0_body)
        self.robot_bodies.append(self.robot_base1_body)
        self.robot_bodies.append(self.robot_link0_body)
        self.robot_bodies.append(self.robot_link1_body)
        self.robot_bodies.append(self.robot_link2_body)
        self.robot_bodies.append(self.robot_link3_body)
        self.robot_bodies.append(self.robot_link4_body)
        self.robot_bodies.append(self.robot_link5_body)
        self.bodies = self.bodies + self.robot_bodies
        # Append the revolute joints to our joints list for PID
        # control and for isDrawing later
        self.joints.append(self.j0)
        self.joints.append(self.j1)
        self.joints.append(self.j2)
        self.joints.append(self.j4)
        # Define the color of the robot parts, using the global
        # dictionary defined on the header
        self.colors[self.robot_base0_body] = COLORS['ROBOT']
        self.colors[self.robot_base1_body] = COLORS['ROBOT']
        self.colors[self.robot_link0_body] = COLORS['ROBOT']
        self.colors[self.robot_link1_body] = COLORS['ROBOT']
        self.colors[self.robot_link2_body] = COLORS['ROBOT']
        self.colors[self.robot_link3_body] = COLORS['GRIPPER']
        self.colors[self.robot_link4_body] = COLORS['GRIPPER']
        self.colors[self.robot_link5_body] = COLORS['GRIPPER']
        # Starts with disabled torque (this also initialize internal variables
        # for the PID control)
        self.disableTorque()
    def _jointPID(self):
        ''' Here we implement a simple PID controller to control
        each joint of the robot using the PID gains that were
        defined using the global variables on the header.
        This PID control controls the velocity of the joints,
        which in turn are torque/force actuated in the Box2D
        framework. This is NOT setting velocities directly.
        '''
        acc_angle = 0
        for i,joint in enumerate(self.joints):
            if i<=2: # revolute joints
                error = self.target[i] - joint.angle
            else: # prismatic joints
                error = self.target[i] - joint.translation
            # Keep the error within -np.pi and np.pi
            if error > np.pi:
                error = error - 2*np.pi
            if error < -np.pi:
                error = error + 2*np.pi
            # Compute the derivative error
            if self.error_old[i] is not None:
                derror = error - self.error_old[i]
                # We add a moving average to smooth jitter
                self.derror = 0.8 * self.derror + 0.2*derror
            else: # If this is the first call, just zero
                self.derror = 0.0
                derror = 0.0
            # Integral is approximated by a sum
            self.error_sum[i] = self.error_sum[i] + error
            # This sends the speed command to the joint,
            # for the Box2D simulation to compute the torques
            joint.motorSpeed = (KPID[i][0]*error\
                                + KPID[i][1]*self.error_sum[i]\
                                + KPID[i][2]*self.derror)
            # Saves the error for computing the derivative
            # next call
            self.error_old[i] = error
    def getLastError(self):
        ''' This method is here to give the joint errors
        (differences between target angle/pos and current
        angle/pos for each joint). These values are computed
        in the _jointPID method.
        '''
        return self.error_old
    def getMotorCommands(self):
        ''' Return the motor speeds
        '''
        return [joint.motorSpeed for joint in self.joints]
    def enableTorque(self):
        ''' Enables the torque on the joints
        '''
        self.isTorqueEnabled = True
        for joint in self.joints:
            joint.motorEnabled = True
    def disableTorque(self):
        ''' Disables the torque on the joints
        '''
        # When disabling torque we also reset
        # the PID variables, just in case.
        self.derror = 0.0
        self.target = [0.0] * len(self.joints)
        self.error_old = [None] * len(self.joints)
        self.error_sum = [0.0] * len(self.joints)
        # From here, torque off
        self.isTorqueEnabled = False
        for joint in self.joints:
            joint.motorEnabled = False
    def setJointTarget(self, target):
        [j0, j1, j2, j4, j5] = target
        self.target = target
    def getFK(self):
        ''' This method returns the current position of the gripper
        in world 2D coordinates
        '''
        x0, y0 = (ROBOTY,\
                         6+BASE_LENGTH+LINK0_LENGTH)
        # These are the link lengths
        a1 = 2*LINK0_LENGTH
        a2 = 2*LINK1_LENGTH
        a3 = 2*LINK2_LENGTH+2*LINK3_LENGTH+2*FINGER_LENGTH
        x = a1*np.cos(self.j0.angle + np.pi/2.0) \
            + a2*np.cos(self.j0.angle + np.pi/2.0 + self.j1.angle) \
            + a3*np.cos(self.j0.angle + np.pi/2.0 + self.j1.angle + self.j2.angle)
        y = a1*np.sin(self.j0.angle + np.pi/2.0) \
            + a2*np.sin(self.j0.angle + np.pi/2.0 + self.j1.angle) \
            + a3*np.sin(self.j0.angle + np.pi/2.0 + self.j1.angle + self.j2.angle)
        # Here is offset the joint coordinates
        x = x + x0
        y = y + y0
        return x,y
    def setIKTarget(self, target):
        ''' This implements the closed-form solution of the inverse kinematics
        for the 2DOF arm.

        Parameters
        ----------

        target: list or tuple, with [x, y, gripper], where gripper
                is a value between 0:closed and 1:open
        '''
        # Get the target variables
        x, y, gripper = target
        # Make sure the gripper is between 0 and 1
        gripper = np.max([np.min([gripper,1.0]),0.0])
        # This is the offset for accounting for the horizontal
        # displacement of the robot, and for the vertical displacement
        # adding the height up to first link and subtracting gripper
        # assembly height (we assume the gripper is always pointing
        # down).
        x0, y0 = (ROBOTY,\
                         6+BASE_LENGTH+LINK0_LENGTH \
                         -(2*LINK2_LENGTH+2*LINK3_LENGTH+2*FINGER_LENGTH))
        # Here is offset the joint coordinates
        x = x - x0
        y = y - y0
        # These are the link lengths
        a1 = 2*LINK0_LENGTH
        a2 = 2*LINK1_LENGTH
        # This computes the closed-form 2DoF inverse kinematics
        num = x**2 + y**2 - a1**2 - a2**2
        den = 2*a1*a2
        # We can only proceed if num/den <= 1, which means
        # the target is reachable.
        if np.abs(num/den) <= 1.0:
            q2 = np.arccos(num / den)
            # For elbow-symmetric solutions (always elbow up)
            # we inverd the signal of q2 if the target is to
            # the right of the robot. This will flip the pose
            # so that the elbow is always up. However, this
            # causes some sudden discontinuity in the movement
            # when moving in a path from the left of the robot
            # to the right. I would avoid working on the space
            # to the right of the robot. If you do not want the
            # flip, just comment the two lines marked "this" below
            if x > 0.0:  # <= this
                q2 = -q2 # <= this
            q1 = np.arctan2(y,x) \
                 - np.arctan2(a2*np.sin(q2),a1+a2*np.cos(q2)) - np.pi/2.0
            # Great, we got a new target.
            self.target = [q1, q2, np.pi-q1-q2, \
                           -gripper*self.gripperMax]
    def getGripperGoal(self):
        ''' Tells if gripper is closed (True) or opened (False)
        '''
        #gripper = - self.j4.translation / self.gripperMax
        gripper = - self.target[-1] / self.gripperMax
        if gripper > 0.25:
            return False
        else:
            return True
    def _is_grasping(self):
        ''' Returns true if welding grasping is active
        '''
        if self.j7 is None:
            return False
        else:
            return True
    def _ungrasp(self):
        ''' Undo the weld joint grasping (if any)
        '''
        if self.j7 is not None:
            self.world.DestroyJoint(self.j7)
            self.world.DestroyJoint(self.j8)
            self.world.DestroyJoint(self.j9)
            self.j7 = None
            self.j8 = None
            self.j9 = None
    def _grasp(self, body, pos=None):
        ''' This function creates a weld joint for a firm
        grasp, not letting the object go easily'''
        if pos is None:
            pos = self.getFK()
        self.j7 = self.world.CreateWeldJoint( \
                    bodyA=self.robot_link2_body,\
                    bodyB=body,\
                    anchor=pos,\
                    collideConnected = False)
        self.j8 = self.world.CreateWeldJoint( \
                    bodyA=self.robot_link4_body,\
                    bodyB=body,\
                    anchor=pos,\
                    collideConnected = False)
        self.j9 = self.world.CreateWeldJoint( \
                    bodyA=self.robot_link5_body,\
                    bodyB=body,\
                    anchor=pos,\
                    collideConnected = False)
    def startFollowingPath(self, path = None):
        ''' Tells the robot to start following a path

        Parameters
        ----------
        path : an array with 3-tuples (x,y,g) where g is
               the gripper (1 for open, 0 for closed)
        '''
        if path: # We overwrite the path one is provided
            self.path = path
        self.path_counter = 0
        self.enableTorque()
        self.isPathFollowing = True
    def stopFollowingPath(self):
        ''' Stops following a path
        '''
        self.isPathFollowing = False
    def step(self, show_graphics=True):
        ''' This is the main method for performing one step of
        simulation, updating position of objects according to
        torques, forces, constraints, gravity and so on. This
        also updates the GUI picture.

        Parameters
        ----------

        show_graphics: This is a boolean flag to define wheter
                       or not the GUI should be updated. If set
                       to False then we will not draw anything
                       on the window and we will not try to
                       throttle the framerate.
        '''
        # First we check if we should be running at all. If
        # the user closes the GUI window, then the simulation
        # is over.
        if not self.running:
            return False
        # Check PyGame's event queue
        for event in pygame.event.get():
            # Check if the application was closed
            if event.type == QUIT:
                pygame.quit()
                self.running = False
                return False
            # Handle event keys
            # Key-down is only used for the gripper. When isDragging
            # with the mouse, isDrawing a path, if the shift key is
            # pressed, then the gripper target for that point will
            # be the gripper-closed configuration.
            elif event.type == KEYDOWN:
                if event.key == K_LSHIFT or event.key == K_RSHIFT:
                    self.gripper = True
            # Key-up is the standard for handling key presses
            # so we only act after the key is released.
            elif event.type == KEYUP:
                # Pressing return (enter) toggles joint torque
                if event.key == K_RETURN:
                    if self.isTorqueEnabled:
                        self.disableTorque()
                    else:
                        self.enableTorque()
                # Pressing DEL or BACKSPACE deletes the path
                if event.key == K_DELETE or event.key == K_BACKSPACE:
                    self.path = []
                # Pressing the SPACE BAR will make the target
                # follow the defined path (if one exists)
                if event.key == K_SPACE:
                    if not self.isPathFollowing:
                        self.path_counter = 0
                        self.isPathFollowing = True
                    else:
                        self.isPathFollowing = False
                # These are some shortcuts to move the joints
                # of the robot go to specific positions,
                # or incrementing/decrementing the angles
                # by 45 deg steps
                if event.key == K_0:
                    self.target = 0.0, 0.0, 0.0, 0.0
                if event.key == K_1:
                    self.target[0] -= np.pi/4
                if event.key == K_2:
                    self.target[0] += np.pi/4
                if event.key == K_3:
                    self.target[1] -= np.pi/4
                if event.key == K_4:
                    self.target[1] += np.pi/4
                if event.key == K_5:
                    self.target[2] -= np.pi/4
                if event.key == K_6:
                    self.target[2] += np.pi/4
                if event.key == K_7:
                    self.target[3] -= 0.25
                if event.key == K_8:
                    self.target[3] += 0.25
                if event.key == K_LSHIFT or event.key == K_RSHIFT:
                    self.gripper = False
            # Check if the user has clicked an object with the mouse
            elif event.type == MOUSEBUTTONDOWN:
                self.pos = pygame_to_b2d(event.pos) # PyGame to Box2D coords
                for body in self.bodies: # Iterate over all bodies...
                    for fixture in body.fixtures: #...and their fixtures
                        if fixture.TestPoint(self.pos): # Test clicked pos
                            self.isDragging = True
                            # Convert the clicked pos into polar coordinates
                            # relative to the clicked body
                            dx = self.pos[0] - body.worldCenter[0]
                            dy = self.pos[1] - body.worldCenter[1]
                            radius = np.sqrt(dx**2 + dy**2)
                            angle = np.arctan2(dy, dx) - body.angle
                            # Save this info in a dictionary
                            self.draggedBodies[fixture.body] = radius, angle
                if not self.isDragging:
                    self.path = []
                    self.isDrawing = True
            # If the mouse moved and we are isDragging, then update the pos
            elif event.type == MOUSEMOTION and self.isDragging:
                self.pos = pygame_to_b2d(event.pos)
            # If the mouse moved and we are isDrawing, then update the pos
            elif event.type == MOUSEMOTION and self.isDrawing:
                self.pos = pygame_to_b2d(event.pos)
            # If the button was released, then stop isDragging
            # and also stop isDrawing
            elif event.type == MOUSEBUTTONUP:
                self.isDragging = False
                self.isDrawing = False
                self.draggedBodies = {}
        # Append a new target for each time step of the isDrawing
        if self.isDrawing:
            if self.gripper:
                gripper_goal = 0.0
            else:
                gripper_goal = 1.0
            self.path.append(self.pos+[gripper_goal])
        # Path following
        if self.isPathFollowing and self.path:
            self.setIKTarget(self.path[self.path_counter])
            self.path_counter = self.path_counter + 1
            if self.path_counter >= len(self.path):
                self.path_counter = 0
                self.isPathFollowing = False
        # Grasp using weld joints
        if self.use_weld_grasp:
            if not self._is_grasping() and self.getGripperGoal():
                grip_pos = self.getFK()
                for body in self.bodies: # Iterate over all bodies...
                    if not body in self.robot_bodies: # Must not belong to robot
                        for fixture in body.fixtures: # Iterate the fixtures
                            if fixture.TestPoint(grip_pos): # Test clicked pos
                                self._grasp(body,grip_pos)
                                break # pick only one
            elif self._is_grasping() and not self.getGripperGoal():
                self._ungrasp()
        # Update joint torques
        if self.isTorqueEnabled:
            self._jointPID()
        # Update forces on dragged objects
        if self.isDragging:
            # The keys of this dictionary are the dragged objects
            for body in self.draggedBodies.keys():
                # Get the relative polar coordinates of the anchor drag
                # point and convert to world coordinates
                radius, angle = self.draggedBodies[body]
                pos0 = body.worldCenter[0]+radius*np.cos(body.angle + angle), \
                       body.worldCenter[1]+radius*np.sin(body.angle + angle)
                # Append this point to be drawn later
                self.dots.append(b2d_to_pygame(pos0))
                force = FORCE_SCALE*(self.pos[0] - pos0[0]),\
                        FORCE_SCALE*(self.pos[1] - pos0[1])
                body.ApplyForce(force, pos0, True)
        # Paint the sky
        self.screen.fill(COLORS['SKY'])
        # Draw the bodies
        for body in self.bodies:
            for fixture in body.fixtures:
                shape = fixture.shape
                vertices = [b2d_to_pygame(body.transform*v) \
                            for v in shape.vertices]
                pygame.draw.polygon(self.screen, self.colors[body], vertices)
        # Draw the revolute joints
        if self.isTorqueEnabled:
            joint_color = COLORS['JOINTSON']
        else:
            joint_color = COLORS['JOINTSOFF']
        for joint in self.joints[:3]:
            pygame.draw.circle(self.screen, joint_color, \
                               b2d_to_pygame(joint.anchorA), JOINT_RADIUS)
        # Draw the dots
        for dot in self.dots:
            pygame.draw.circle(self.screen, COLORS['RED'], dot, 5)
        # Draw the path
        if self.isDrawing:
            path_radius = 2
        else:
            path_radius = 5
        for x,y,gripper in self.path:
            point = x,y
            if gripper == 1.0:
                color = COLORS['PATHOFF']
            else:
                color = COLORS['PATHON']
            pygame.draw.circle(self.screen, color,
                               b2d_to_pygame(point), path_radius)
        # If it is following a path, then draw a red circle
        if self.path and self.isPathFollowing:
            pygame.draw.circle(self.screen, COLORS['TARGET'],
                            b2d_to_pygame(self.path[self.path_counter]), 8)
        # Draw the current pos
        pygame.draw.circle(self.screen, COLORS['CURRPOS'],
                           b2d_to_pygame(self.getFK()), 9, 2)
        self.dots = []
        # Simulation step
        self.world.Step(TIME_STEP, 10, 10)
        if show_graphics:
            # Update the screen
            pygame.display.flip()
            # Pace the framerate
            self.clock.tick(FPS)
        return True
    def createBoxes(self):
        ''' Creates 25 little boxes, for interaction
        '''
        self.box_bodies = []
        for i in range(25):
            # create a random box at random horizontal positions
            # around position 25,15
            self.box_bodies.append(self.world.CreateDynamicBody( \
                                    position=(25 + np.random.random()*10, 15),
                                    angle=np.random.random()*90.0))
            # define the box perimeter fixture and its friction is high
            # to make it easier for the robot to grasp the box
            self.box_bodies[-1].CreatePolygonFixture(box=(1.25, 1.25),
                                                     density=0.125,
                                                     friction=100.0,
                                                     restitution=BOX_RESTITUTION)
        # We color all these boxes in the BOX color
        for box in self.box_bodies:
            self.colors[box] = COLORS['BOX']
        # And finally we append them to our simulation queue
        self.bodies = self.bodies + self.box_bodies
    def createTablesEnv(self):
        ''' Creates an environment with two tables and some boxes
        for manipulation tasks
        '''
        self.table1 = self.world.CreateStaticBody(position=(25,10))
        self.table1.CreatePolygonFixture(box=(3,5))
        self.table2 = self.world.CreateStaticBody(position=(40,10))
        self.table2.CreatePolygonFixture(box=(3,5))
        self.box = self.world.CreateDynamicBody(position=(25,15+1.25))
        self.box.CreatePolygonFixture(box=(1.25,1.25),
                                      density=0.125,
                                      friction=100.0,
                                      restitution=BOX_RESTITUTION)
        self.colors[self.table1] = COLORS['TABLE']
        self.colors[self.table2] = COLORS['TABLE']
        self.colors[self.box] = COLORS['BOX']
        self.bodies = self.bodies + [self.table1, self.table2, self.box]
        pos0 = (32.5,30)
        pos_above_goal1 = (25,20)
        pos_goal1 = (25,15+1.25)
        pos_above_goal2 = (40,20)
        pos_goal2 = (40,15+1.25)
        self.goal = pos_goal2
        path = []
        path = path + interpolate_path(pos0, pos_above_goal1, 1.0, 25)
        path = path + interpolate_path(pos_above_goal1, pos_goal1, 1.0, 50)
        path = path + interpolate_path(pos_goal1, pos_goal1, 0.0, 25)
        path = path + interpolate_path(pos_goal1, pos_above_goal1, 0.0)
        path = path + interpolate_path(pos_above_goal1, pos_above_goal2, 0.0)
        path = path + interpolate_path(pos_above_goal2, pos_goal2, 0.0)
        path = path + interpolate_path(pos_goal2, pos_goal2, 1.0, 25)
        path = path + interpolate_path(pos_goal2, pos_above_goal2, 1.0, 50)
        path = path + interpolate_path(pos_above_goal2, pos0, 1.0, 25)
        self.path = path

if __name__ == '__main__':
    robot = Robot2D()
    while robot.step():
        pass

