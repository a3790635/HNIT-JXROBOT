# coding=utf-8
from naoqi import ALProxy



IP = "192.168.1.106"  # 机器人的IP地址
PORT = 9559  # 机器人的端口号，默认9559
motion = ALProxy("ALMotion", IP, PORT)
posture = ALProxy("ALRobotPosture", IP, PORT)
motion.wakeUp()


def f(x,y,theta):
    motion.wakeUp()
    posture.goToPosture("StandInit", 1)
    motion.moveTo(x, y, theta,
                       [["MaxStepFrequency", 0.98], ["TorsoWx", 0.00], ["TorsoWy", 0.00], ["StepHeight", 0.025],
                        ["MaxStepTheta", 0.075], ["MaxStepX", 0.013]])


f(0.1,0.1,1)
