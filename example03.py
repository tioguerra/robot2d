from robot2d import Robot2D
import numpy as np

path = []

x0, y0 = 30.0,25.0
x1, y1 = 40.0,25.0
radius = 5.0
gripper = 1.0
d = x1 - x0
n = 50
repeat = 4

for angle in np.linspace(np.pi/2.0, 3.0*np.pi/2.0,n):
    x = x0 + radius*np.cos(angle)
    y = y0 + radius*np.sin(angle)
    pos = (x,y,gripper)
    path.append(pos)

for i in np.linspace(0.0,1.0,n):
    x = x0 + d*i
    y = y0 - radius*np.cos(i*np.pi)
    pos = (x,y,gripper)
    path.append(pos)

for angle in np.linspace(5*np.pi/2.0,3.0*np.pi/2.0,n):
    x = x1 + radius*np.cos(angle)
    y = y1 + radius*np.sin(angle)
    gripper = 1.0
    pos = (x,y,gripper)
    path.append(pos)

for i in np.linspace(1.0,0.0,n):
    x = x0 + d*i
    y = y0 + radius*np.cos(i*np.pi)
    pos = (x,y,gripper)
    path.append(pos)

path = path*repeat

r = Robot2D()
r.startFollowingPath(path)
while r.isPathFollowing:
    r.step()

