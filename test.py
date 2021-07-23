# coding=utf-8
import time

from naoqi import ALProxy

IP = "192.168.1.107"
PORT = 9559

redBallProxy = ALProxy("ALRedBallDetection", IP, PORT)
camProxy = ALProxy("ALVideoDevice", IP, PORT)
memoryProxy = ALProxy("ALMemory", IP, PORT)
camProxy.setActiveCamera(1)
period = 100
redBallProxy.subscribe("Test_RedBall", period, 0.0)

memValue = "redBallDetected"
print
try:
    while True:
        time.sleep(period / 1000)
        val = memoryProxy.getData(memValue)
        if val and isinstance(val, list) and len(val) >= 2:
            timeStamp = val[0]
            position = val[3]
            try:
                print"\rX={:8<.6f}\tY={:8<.6f}\tZ={:8<.6f}\tWX={:8<.6f}\tWY={:8<.6f}\tWZ={:8<.6f}".format(
                    position[0], position[1], position[2], position[3], position[4], position[5]),
            except IndexError, e:
                print("RedBall detected, but it seems getData is invalid. ALvalue = ")
                print(val)
                print("Error msg %s" % (str(e)))
        else:
            print("Error with getData. ALValue = %s" % (str(val)))
finally:
    redBallProxy.unsubscribe("Test_RedBall")
