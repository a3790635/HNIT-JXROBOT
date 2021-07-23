import time

from hello import RedBallTracking
from naoqi import ALProxy

IP = "192.168.1.107"
PORT = 9559

motionProxy = ALProxy("ALMotion", IP, PORT)
memoryProxy = ALProxy("ALMemory", IP, PORT)
redBallProxy = ALProxy("ALRedBallDetection", IP, PORT)
ballTracking = RedBallTracking(memoryProxy, motionProxy, redBallProxy)
ballTracking.start()
try:
    while True:
        time.sleep(.1)
        angle = ballTracking.getAngles()
        if angle == ():
            print "\rNo Ball",
        else:
            print "\rAngle={},Distance={}".format(angle, ballTracking.getDistance()),
finally:
    ballTracking.stop()
