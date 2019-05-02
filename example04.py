from robot2d import Robot2D

r = Robot2D()
r.createSeeSawEnv()
while r.step():
    print(r.fallenBalls())

