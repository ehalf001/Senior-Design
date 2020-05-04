from controller import Robot

robot = Robot("Hexapod")

while robot.step(32) != -1:
    print("Hello World!")