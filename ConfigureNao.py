import sys
from naoqi import ALProxy


class ConfigureNao(object):
    def __init__(self, IP, PORT=9559):
        self.IP = IP
        self.PORT = PORT
        try:
            self.cameraProxy = ALProxy("ALVideoDevice", self.IP, self.PORT)
            self.motionProxy = ALProxy("ALMotion", self.IP, self.PORT)
            self.postureProxy = ALProxy("ALRobotPosture", self.IP, self.PORT)
            self.tts = ALProxy("ALTextToSpeech", self.IP, self.PORT)
            self.memoryProxy = ALProxy("ALMemory", self.IP, self.PORT)
            self.redBallProxy = ALProxy("ALRedBallDetection", IP, PORT)
            # self.landMarkProxy = ALProxy("ALLandMarkDetection", self.IP, self.PORT)
            self.ALAutonomousLifeProxy = ALProxy("ALAutonomousLife", self.IP, self.PORT)
        except Exception as e:
            print("Error when configuring the NAO!")
            print(str(e))
            exit(1)
