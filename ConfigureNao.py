import sys
from naoqi import ALProxy
import math


class ConfigureNao(object):
    def __init__(self, IP, PORT=9559):
        self.motionProxy = ALProxy("ALMotion","192.168.1.106",9559)
        self.IP = IP
        self.PORT = PORT
        try:
            self.cameraProxy = ALProxy("ALVideoDevice", self.IP, self.PORT)
            self.postureProxy = ALProxy("ALRobotPosture", self.IP, self.PORT)
            self.tts = ALProxy("ALTextToSpeech", self.IP, self.PORT)
            self.memoryProxy = ALProxy("ALMemory", self.IP, self.PORT)
            # self.sonarProxy = ALProxy("ALSonarProxy", self.IP, self.PORT)
            # self.landMarkProxy = ALProxy("ALLandMarkDetection", self.IP, self.PORT)
            self.ALAutonomousLifeProxy = ALProxy("ALAutonomousLife", self.IP, self.PORT)
        except Exception as e:
            print("Error when configuring the NAO!")
            print(str(e))
            exit(1)


def compensate(origin, now):
    if now * origin < 0:
        if now > 0:
            return math.pi * 2 - (abs(origin) + abs(now))
        else:
            return abs(origin) + abs(now) - 2 * math.pi
    else:
        if now > origin:
            return -abs(origin - now)
        else:
            return abs(now - origin)


def judge(compen):
    if abs(compen) > 0.12:
        return compen / 4
    elif abs(compen) < 0.018:
        return 0
    else:
        return compen
