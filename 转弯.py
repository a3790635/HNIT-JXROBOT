# coding=utf-8
from naoqi import ALProxy
import Tkinter as tk
import numpy as np
import cv2 as cv
import math

IP = "192.168.1.107"  # 机器人的IP地址
PORT = 9559  # 机器人的端口号，默认9559
motion = ALProxy("ALMotion", IP, PORT)
posture = ALProxy("ALRobotPosture", IP, PORT)
motion.wakeUp()
def f(x,y,theta):
    motion.wakeUp()
    posture.goToPosture("StandInit", 1.0)
    motion.moveTo(x, y, theta)
