# coding=utf-8
import threading
import time

from naoqi import ALProxy

IP = "192.168.1.106"
PORT = 9559

redBallProxy = ALProxy("ALRedBallDetection", IP, PORT)
camProxy = ALProxy("ALVideoDevice", IP, PORT)
memoryProxy = ALProxy("ALMemory", IP, PORT)
motionProxy = ALProxy("ALMotion", IP, PORT)
camProxy.setActiveCamera(1)
period = 100
redBallProxy.subscribe("Test_RedBall", period, 0.0)

memValue = "redBallDetected"

try:
    while True:
        time.sleep(period / 1000)
        val = memoryProxy.getData(memValue)
        if val and isinstance(val, list) and len(val) >= 2:
            timeStamp = val[0]
            ballInfo = val[1]
            try:
                print("centerX= {} centerY= {}".format(ballInfo[0], ballInfo[1]))
                print("sizeX= {} sizeY= {}".format(ballInfo[2], ballInfo[3]))
                anglesX = ballInfo[0]
                anglesY = ballInfo[1]
                # fractionMaxSpeed = .1
                # motionProxy.setAngles("HeadYaw", anglesX, fractionMaxSpeed)
                # motionProxy.setAngles("HeadPitch", anglesY, fractionMaxSpeed)

                previous_errorX = 0
                integralX = 0
                setPointX = anglesX  # 设定值
                measuredValueX = motionProxy.getAngles("HeadYaw", True)[0]  # 反馈值
                errorX = abs(setPointX - measuredValueX)
                integralX = integralX + errorX
                derivativeX = errorX - previous_errorX
                previous_errorX = errorX
                fractionMaxSpeedX = derivativeX / 2

                previous_errorY = 0
                integralY = 0
                setPointY = anglesY  # 设定值
                measuredValueY = motionProxy.getAngles("HeadPitch", True)[0]  # 反馈值
                errorY = abs(setPointY - measuredValueY)
                integralY = integralY + errorY
                derivativeY = errorY - previous_errorY
                previous_errorY = errorY
                fractionMaxSpeedY = derivativeY / 2

                motionProxy.setAngles("HeadYaw", anglesX, fractionMaxSpeedX)
                motionProxy.setAngles("HeadPitch", anglesY, fractionMaxSpeedY)

            except IndexError, e:
                print("RedBall detected, but it seems getData is invalid. ALvalue = ")
                print(val)
                print("Error msg %s" % (str(e)))
        else:
            print("Error with getData. ALValue = %s" % (str(val)))
finally:
    redBallProxy.unsubscribe("Test_RedBall")
