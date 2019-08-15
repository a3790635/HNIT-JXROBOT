# _*_coding:utf-8_*_
import argparse
import threading
import cv2
import numpy as np
import numpy as shap
import numpy as ndarray
from numpy import ndarray
import time
import math
import time
import sys
import PIL
from PIL import Image, ImageDraw
from naoqi import ALProxy, ALModule, ALBroker
from multiprocessing import Process
from VisualBasis import *
from ctypes import *
import types
import random
#1.Ready 准备
#2.HoldingPole 握竿
#3.AddressingTheBall 击球准备
#4.Batting 击球
#5.ReceivingPole 收杆
#6.MoveEnd 运动结束

def BalanceLight(img,rows,cols):
    dll = CDLL(r"c++/Dll1.dll")
    dataptr = img.ctypes.data_as(c_char_p)
    dll.BalanceLight(dataptr, rows, cols)

_stepStatue = [["MaxStepX", 0.04], ["MaxStepY", 0.14], ["MaxStepTheta", 0.4], ["MaxStepFrequency", 0.6],
               ["StepHeight", 0.02], ["TorsoWx", 0], ["TorsoWy", 0]]
def run(IP,PORT):
    MemoryProxy = ALProxy("ALMemory", IP, PORT)
    while True:
        flag1 = MemoryProxy.getData("FrontTactilTouched")
        flag2 = MemoryProxy.getData("MiddleTactilTouched")
        flag3 = MemoryProxy.getData("RearTactilTouched")
        dz=MotionClass(IP,PORT)
        if flag1 == 1.0:
            flag1 = 0.0
            time.sleep(0.5)
            dz.HoldingPole()
        if flag2 == 1.0:
            flag2 = 0.0
            time.sleep(0.5)
            while True:
                q=input()
                dz.AddressingTheBall()
                dz.Batting(q)
                dz.ReceivingPole()
        if flag3 == 1.0:
            flag3 = 0.0
            time.sleep(0.5)
            dz.AddressingTheBall2()
            dz.Batting2(0.3)
            dz.ReceivingPole()



class MotionClass(object):
    def __init__(self, IP, PORT):
        self.IP = IP
        self.PORT = PORT

    def TouchFirstHead(self):
        MemoryProxy = ALProxy("ALMemory", self.IP, self.PORT)
        while True:
            flag1 = MemoryProxy.getData("FrontTactilTouched")
            flag2 = MemoryProxy.getData("MiddleTactilTouched")
            flag3 = MemoryProxy.getData("RearTactilTouched")
            if flag1 == 1.0:
                self.HoldingPole()
                break
            if flag2 == 1.0:
                Z=self.WalkToBall(self.IP,self.PORT)
                Z.start()
                self.AddressingTheBall()
                self.Batting(0.3)
                self.ReceivingPole()
                break
            if flag3 == 1.0:

                Move=self.Move(self.IP,self.PORT)
                Move.MoveT(1,-1)
                self.MoveEnd()
                break

    class WalkToBall(object):
        def __init__(self, IP, PORT=9559):
            self.ballDetect = BallDetect(IP, resolution=vd.kVGA)
            self.tts = ALProxy("ALTextToSpeech", IP, PORT)
            self.camera = ALProxy("ALVideoDevice", IP, PORT)
            self.motion = ALProxy("ALMotion", IP, PORT)
            self.posture = ALProxy("ALRobotPosture", IP, PORT)
            self.IP = IP
            print "1"

        def start(self):
            # self.motion.wakeUp()
            self.motion.setStiffnesses("Head", 1.0)
            time.sleep(1.0)
            rnumber = 1
            self.camera.setActiveCamera(1)
            self.motion.setMoveArmsEnabled(False, False)
            # self.tts.say("红球在哪里呢？")
            isenabled = True
            Headnames = "HeadYaw"
            while 1:
                angle = 0
                Headnames = "HeadYaw"
                self.motion.setAngles(Headnames, angle, 0.2)
                time.sleep(1.5)
                ballData = self.ballDetect.updateBallData(colorSpace="HSV", standState="standUp", saveFrameBin=True)
                time.sleep(3)
                if ballData != {}:
                    # self.tts.say("我看到红球了")
                    break
                else:
                    for i in range(1, 3):
                        if i <= 1:
                            angle -= 60 * almath.TO_RAD
                        else:
                            angle = 0
                            angle += 60 * almath.TO_RAD
                            ballData = self.ballDetect.updateBallData()
                            if ballData != {}:
                                break
                        self.motion.setAngles(Headnames, angle, 0.2)
                    self.motion.moveTo(0.5, 0.0, _stepStatue)
            # 第一次转到机器人正对球的方向
            x = 0.0
            y = 0.0
            # self.posture.goToPosture("StandInit", 0.5)
            # time.sleep(1)
            theta = ballData["angle"]
            self.motion.setStiffnesses("Body", 1.0)
            time.sleep(1)
            self.motion.setMoveArmsEnabled(False, False)
            self.motion.moveTo(0, 0, theta, _stepStatue)
            # self.tts.say("已转到面对球的方向")

            # 第二次走到距离球50cm的位置
            _stepStatue[3][1] = 0.2
            time.sleep(0.5)
            ballData = self.ballDetect.updateBallData(colorSpace="HSV", standState="standUp", saveFrameBin=True)
            x = ballData["disX"] - 0.5
            y = 0.0
            theta = 0.0
            # self.tts.say("距离红球还有" + str(x))
            self.motion.setStiffnesses("Body", 1.0)
            time.sleep(1)
            self.motion.setMoveArmsEnabled(False, False)
            self.motion.moveTo(x, y, theta, _stepStatue)
            self.motion.waitUntilMoveIsFinished()

            # 第三次走到距离球20cm的位置
            _stepStatue[3][1] = 0.2
            time.sleep(0.5)
            ballData = self.ballDetect.updateBallData(colorSpace="HSV", standState="standUp", saveFrameBin=True)
            x = ballData["disX"] - 0.2
            y = 0.0
            theta = 0.0
            # self.tts.say("距离红球还有" + str(x))
            self.motion.setStiffnesses("Body", 1.0)
            time.sleep(1)
            self.motion.setMoveArmsEnabled(False, False)
            self.motion.moveTo(x, y, theta, _stepStatue)
            self.motion.waitUntilMoveIsFinished()
            
            # 第四次机器人对准球,进行修正
            _stepStatue[3][1] = 0.6
            # self.posture.goToPosture("StandInit", 0.5)
            # self.motion.waitUntilMoveIsFinshed()
            Headnames = "HeadPitch"
            timelist = 1.5
            angleLists = 30 * almath.TO_RAD
            isenabled = False  # False 表示相对角度
            self.motion.setStiffnesses("Body", 1.0)
            time.sleep(1)
            self.motion.angleInterpolation(Headnames, angleLists, timelist, isenabled)
            ballData = self.ballDetect.updateBallData(colorSpace="HSV", standState="standInit", saveFrameBin=True)
            x = 0.0
            y = 0.0
            theta = ballData["angle"]
            self.motion.setStiffnesses("Body", 1.0)
            time.sleep(1)
            self.motion.setMoveArmsEnabled(False, False)
            self.motion.moveTo(x, y, theta, _stepStatue)
            # self.tts.say("对准球的方向，修正完成")


            # 第五次机器人走到离球10cm的位置
            _stepStatue[3][1] = 0.5
            time.sleep(1.5)
            ballData = self.ballDetect.updateBallData(colorSpace="HSV", standState="standInit", saveFrameBin=True)
            # print (ballData)
            x = ballData["disX"] - 0.1
            y = 0.0
            theta = 0.0
            self.motion.setStiffnesses("Body", 1.0)
            time.sleep(1)
            self.motion.setMoveArmsEnabled(False, False)
            self.motion.moveTo(x, y, theta, _stepStatue)
            # self.tts.say("已走到离球10cm的位置")

            # 第六次机器人对准球进行修正
            _stepStatue[3][1] = 0.6
            ballData = self.ballDetect.updateBallData(colorSpace="HSV", standState="standInit", saveFrameBin=True)
            dx = math.sqrt(ballData["disX"] * ballData["disX"] + ballData["disY"] * ballData["disY"])
            x = 0.0
            y = 0.0
            theta = ballData["angle"]
            self.motion.setStiffnesses("Body", 1.0)
            time.sleep(1)
            self.motion.moveTo(x, y, theta, _stepStatue)
            # self.tts.say("第二次修正完成")

            # 第七次对球进行最终定位
            ballData = self.ballDetect.updateBallData(colorSpace="HSV", standState="standInit", saveFrameBin=True)
            dx = math.sqrt(ballData["disX"] * ballData["disX"] + ballData["disY"] * ballData["disY"])
            x = ballData["disX"]
            y = ballData["disY"]
            theth = ballData["angle"]
            # self.tts.say("完成对球的最终定位")


            Headnames = ["HeadPitch"]
            timelist = [0.5]
            angleLists = [0.0]
            self.motion.angleInterpolation(Headnames, angleLists, timelist, True)  # True 表示绝对角度
            for i in range(0, 20):
                time.sleep(1.5)
                landmark = LandMarkDetect(self.IP)
                time.sleep(1.5)
                landmarkData = landmark.updateLandMarkData()
                if landmarkData != []:
                    alpha = landmarkData[3]
                    d1 = landmarkData[2]
                    self.tts.say("我看到地标了")
                    direction = 0
                    break
                else:
                    Headnames = ["HeadYaw"]
                    timelist = [1.5]
                    angleLists = [45 * almath.TO_RAD]
                    self.motion.angleInterpolation(Headnames, angleLists, timelist, False)  # 相对角度
                    time.sleep(3.5)
                    landmarkData = landmark.updateLandMarkData()
                    if landmarkData != []:
                        alpha = landmarkData[3]
                        d1 = landmarkData[2]
                        self.tts.say("我看到地标了")
                        direction = 1
                        break
                    else:
                        Headnames = ["HeadYaw"]
                        timelist = [1.5]
                        angleLists = [-45 * almath.TO_RAD]
                        self.motion.angleInterpolation(Headnames, angleLists, timelist, True)
                        time.sleep(3.5)
                        landmarkData = landmark.updateLandMarkData()
                        if landmarkData != []:
                            alpha = landmarkData[3]
                            d1 = landmarkData[2]
                            self.tts.say("我看到地标了")
                            direction = -1
                            break
                        else:
                            Headnames = ["HeadYaw"]
                            timelist = [1.5]
                            angleLists = [0.0]
                            self.motion.angleInterpolation(Headnames, angleLists, timelist, True)
                            time.sleep(1.0)
                            self.motion.setStiffnesses("Body", 1.0)
                            time.sleep(1)
                            #self.posture.goToPosture("StandInit", 0.5)
                            time.sleep(1)
                            self.motion.moveTo(0.24, 0.24, -math.pi / 2.0, _stepStatue)
                            time.sleep(3.5)
                            self.motion.waitUntilMoveIsFinished()
                            direction = 0
                            continue
            try:
                self.tts.say("距离地标的位置：" + str(landmarkData[2]))
            except:
                print str(landmarkData[2])
            print str(landmarkData)

            Headnames = ["HeadYaw"]
            timelist = [0.5]
            angleLists = [0.0]
            self.motion.angleInterpolation(Headnames, angleLists, timelist, True)  # True 表示绝对角度
            time.sleep(1.0)
            Headnames = ["HeadPitch"]
            timelist = [0.5]
            angleLists = [30 * almath.TO_RAD]
            self.motion.angleInterpolation(Headnames, angleLists, timelist, True)  # True 表示绝对角度
            time.sleep(1.5)
            ballData = self.ballDetect.updateBallData(colorSpace="HSV", standState="standInit", saveFrameBin=True)
            dx = math.sqrt(ballData["disX"] * ballData["disX"] + ballData["disY"] * ballData["disY"])
            x = ballData["disX"]
            y = ballData["disY"]
            time.sleep(5)
            Headnames = ["HeadPitch"]
            timelist = [0.5]
            angleLists = [0.0]
            self.motion.angleInterpolation(Headnames, angleLists, timelist, True)  # True 表示绝对角度
            theth = ballData["angle"]
            Headnames = ["HeadYaw"]
            timelist = [0.5]
            angleLists = [landmarkData[3]]
            self.motion.angleInterpolation(Headnames, angleLists, timelist, True)  # 将机器人的头对准mark

            time.sleep(1.5)
            # landmarkData = landmark.updateLandMarkData()
            if direction == 1:
                alpha += 45 * almath.TO_RAD
            elif direction == -1:
                alpha -= 45 * almath.TO_RAD
            else:
                alpha = alpha

            theta3 = abs(theth - alpha)  # 球与mark到机器人的夹角
            dball = dx  # 机器人与球之间的距离
            dmark = d1  # 机器人到mark的距离
            print ("dmark:", d1)
            d_ball_mark_square = dball ** 2 + dmark ** 2 - 2 * dball * dmark * math.cos(
                theta3)  # 余弦定理    (正弦定理OK)  求球到mark的距离
            print ("dmarksquare:", d_ball_mark_square)
            d_ball_mark = math.sqrt(d_ball_mark_square)
            theta_robot_ball_mark_data = (dball ** 2 + d_ball_mark_square - dmark ** 2) / (2 * dball * d_ball_mark)
            print ("ball_mark_robot_data", theta_robot_ball_mark_data)
            theta_robot_ball_mark = math.acos(theta_robot_ball_mark_data)  # 球到mark和机器人之间夹角的弧度
            theta_robot_mark_ball = math.pi - theta_robot_ball_mark - theta3  # 机器人到球和mark之间夹角的弧度
            print ("theta_ball_mark_robot", theta_robot_ball_mark)
            print ("theta_mark_ball_robot", theta_robot_mark_ball)
            print ("alpha", alpha)
            """
            if theth - alpha >= 0:
                if theta_ball_mark_robot >= math.pi / 2:
                    theta = theta_ball_mark_robot - math.pi / 2
                    x = dball * math.sin(theta_ball_mark_robot)
                    y = dball * math.cos(theta_ball_mark_robot)
                    y -= 0.10
                elif theta_ball_mark_robot < math.pi:
                    theta = math.pi / 2 - theta_ball_mark_robot
                    x = dball * math.sin(theta_ball_mark_robot)
                    y = dball * math
            """
            if theth - alpha >= 0:  # 红球相对位置在mark右边
                if 0 <= theta_robot_ball_mark < 0.5 * math.pi:
                    theta = 0.5 * math.pi - theta_robot_ball_mark
                    dy = math.sqrt(((dball ** 2) * 2 - 2 * dball * dball * math.cos(theta)))
                    x = -dy * math.cos(theta_robot_mark_ball)
                    y = dy * math.sin(theta_robot_mark_ball)
                    theta = theta
                    print ("锐角theta_robot_ball_mark", theta_robot_ball_mark * almath.TO_DEG)
                    self.tts.say("右边  夹角为锐角")
                else:
                    theta = theta_robot_ball_mark - 0.5 * math.pi
                    dy = (math.sqrt(((dball ** 2) * 2 - 2 * dball * dball * math.cos(theta))))
                    thetatemp = (math.pi - theta) / 2.0
                    x = -dy * math.sin(thetatemp)
                    y = -dy * math.cos(thetatemp)
                    theta = theta + 10 * almath.TO_RAD
                    print ("钝角theta_robot_ba", theta_robot_ball_mark * almath.TO_DEG)
                    self.tts.say("右边 夹角为钝角")
            else:
                if 0 <= theta_robot_ball_mark < 0.5 * math.pi:
                    theta = 0.5 * math.pi - theta_robot_ball_mark
                    dy = math.sqrt(((dball ** 2) * 2 - 2 * dball * dball * math.cos(theta)))
                    x = -dy * math.cos(theta_robot_mark_ball)
                    y = dy * math.sin(theta_robot_mark_ball)
                    theta = -theta
                    self.tts.say("左边  夹角为锐角")
                else:
                    theta = theta_robot_ball_mark - 0.5 * math.pi
                    dy = (math.sqrt(((dball ** 2) * 2 - 2 * dball * dball * math.cos(theta))))
                    thetatemp = (math.pi - theta) / 2.0
                    x = -dy * math.sin(thetatemp)
                    y = dy * math.cos(thetatemp)
                    y = y + 0.17
                    # x = x + 0.03
                    theta = -theta - 10 * almath.TO_RAD
                    self.tts.say("左边 夹角为钝角")

            _stepStatue[4][1] = 0.025
            self.motion.setMoveArmsEnabled(False, False)
            self.motion.moveTo(0.0, 0.0, theta, _stepStatue)
            time.sleep(1)
            self.tts.say("转角完成")
            self.motion.waitUntilMoveIsFinished()
            self.motion.setStiffnesses("Body", 1.0)
            time.sleep(1)
            self.motion.setMoveArmsEnabled(False, False)
            self.motion.moveTo(x, 0.0, 0.0, _stepStatue)
            self.motion.waitUntilMoveIsFinished()
            self.motion.setMoveArmsEnabled(False, False)
            self.motion.moveTo(0.0, y, 0.0, _stepStatue)
            self.motion.waitUntilMoveIsFinished()

            # 再判定一次球的距离
            #self.posture.goToPosture("StandInit", 0.5)
            time.sleep(1)
            Headnames = ["HeadPitch"]
            timelist = [0.5]
            angleLists = 30 * almath.TO_RAD
            self.motion.angleInterpolation(Headnames, angleLists, timelist, True)  # True 表示绝对角度
            ballData = self.ballDetect.updateBallData(colorSpace="HSV", standState="standInit", saveFrameBin=True)
            if ballData != []:
                x = ballData["disX"] - 0.1
                y = ballData["disY"]
                # theta = ballData["angle"]
            self.motion.setStiffnesses("Body", 1.0)
            time.sleep(1)
            # self.motion.setMoveArmsEnabled(False, False)
            # self.motion.moveTo(0.0, 0.0, theta, _stepStatue)
            # self.motion.waitUntilMoveIsFinished()
            self.motion.setMoveArmsEnabled(False, False)
            self.motion.moveTo(x, 0.0, 0.0, _stepStatue)
            self.motion.waitUntilMoveIsFinished()
            self.motion.setMoveArmsEnabled(False, False)
            self.motion.moveTo(0.0, y, 0.0, _stepStatue)
            self.motion.waitUntilMoveIsFinished()

    def Stand(self):
        MotionProxy = ALProxy("ALRobotPosture", self.IP, self.PORT)
        MotionProxy.goToPosture("StandInit", 0.5)

    class Move():
        def __init__(self, robotIP, PORT):
            self.robotIP = robotIP
            self.PORT = PORT


        def MoveT(self, x, y, their=0):
            motionProxy = ALProxy("ALMotion", self.robotIP, self.PORT)
            motionProxy.setStiffnesses("Body", 1.0)
            time.sleep(1.0)
            motionProxy.setMoveArmsEnabled(False, False)
            if(y!=0):
                their=math.atan(x/y)
                print their
            motionProxy.moveTo(0,0, their, [["MaxStepX", 0.04], ["MaxStepY", 0.14],["MaxStepFrequency", 0.4], ["StepHeight", 0.02]])
            motionProxy.moveTo(math.sqrt(x*x+y*y)*0.85,0, 0, [["MaxStepX", 0.04], ["MaxStepY", 0.14],["MaxStepFrequency", 0.4]])

        def HuDu(self, x):
            return -x * math.pi / 180.0

        def MoveToBall(self, x, y):
            z = math.atan(x / y)
            z = self.HuDu(z)
            self.MoveT(0, 0, z)
            x = math.sqrt(x ** 2.0 + y ** 2.0)
            self.MoveT(x, 0, 0)

    def Ready(self):
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([1.36])
        keys.append([[-0.135034, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("HeadYaw")
        times.append([1.36])
        keys.append([[-0.00464392, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LAnklePitch")
        times.append([1.36])
        keys.append([[0.0886833, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LAnkleRoll")
        times.append([1.36])
        keys.append([[-0.108787, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LElbowRoll")
        times.append([1.36])
        keys.append([[-0.80991, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([1.36])
        keys.append([[-1.51563, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([1.36])
        keys.append([[0.8848, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LHipPitch")
        times.append([1.36])
        keys.append([[0.126656, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LHipRoll")
        times.append([1.36])
        keys.append([[0.116344, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LHipYawPitch")
        times.append([1.36])
        keys.append([[-0.17046, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LKneePitch")
        times.append([1.36])
        keys.append([[-0.0923279, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([1.36])
        keys.append([[0.771559, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([1.36])
        keys.append([[0.0797259, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([1.36])
        keys.append([[-0.188724, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RAnklePitch")
        times.append([1.36])
        keys.append([[0.0874194, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RAnkleRoll")
        times.append([1.36])
        keys.append([[0.109102, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([1.36])
        keys.append([[0.621311, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([1.36])
        keys.append([[1.52322, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([1.36])
        keys.append([[0.998363, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RHipPitch")
        times.append([1.36])
        keys.append([[0.126321, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RHipRoll")
        times.append([1.36])
        keys.append([[-0.11736, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RHipYawPitch")
        times.append([1.36])
        keys.append([[0.737377, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RKneePitch")
        times.append([1.36])
        keys.append([[-0.0894577, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([1.36])
        keys.append([[0.65506, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([1.36])
        keys.append([[0.0106959, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([1.36])
        keys.append([[0.00916195, [3, -0.466667, 0], [3, 0, 0]]])


        try:
            motion = ALProxy("ALMotion", self.IP, self.PORT)
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err

    def HoldingPole(self):
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([0.4])
        keys.append([[-0.135034, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("HeadYaw")
        times.append([0.4])
        keys.append([[-0.00464392, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LAnklePitch")
        times.append([0.4])
        keys.append([[0.0886833, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LAnkleRoll")
        times.append([0.4])
        keys.append([[-0.108787, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LElbowRoll")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append([[-0.80991, [3, -0.146667, 0], [3, 0.186667, 0]],
                     [-0.811062, [3, -0.186667, 0.00115195], [3, 0.146667, -0.000905104]],
                     [-0.819703, [3, -0.146667, 0.00252602], [3, 0.2, -0.00344457]],
                     [-0.828973, [3, -0.2, 0], [3, 0.133333, 0]], [-0.819703, [3, -0.133333, 0], [3, 0.186667, 0]],
                     [-1.53923, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append([[-1.51563, [3, -0.146667, 0], [3, 0.186667, 0]],
                     [-1.52345, [3, -0.186667, 0.00331594], [3, 0.146667, -0.00260538]],
                     [-1.5334, [3, -0.146667, 0.00137373], [3, 0.2, -0.00187327]],
                     [-1.53527, [3, -0.2, 0], [3, 0.133333, 0]], [-1.5334, [3, -0.133333, 0], [3, 0.186667, 0]],
                     [-1.73363, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append([[0.8848, [3, -0.146667, 0], [3, 0.186667, 0]],
                     [0.884792, [3, -0.186667, 8.04663e-06], [3, 0.146667, -6.32235e-06]],
                     [0.00413917, [3, -0.146667, 0], [3, 0.2, 0]],
                     [0.00539452, [3, -0.2, -0.000549448], [3, 0.133333, 0.000366299]],
                     [0.00688641, [3, -0.133333, -0.00149189], [3, 0.186667, 0.00208864]],
                     [0.526548, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("LHipPitch")
        times.append([0.4])
        keys.append([[0.126656, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LHipRoll")
        times.append([0.4])
        keys.append([[0.116344, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LHipYawPitch")
        times.append([0.4])
        keys.append([[-0.17046, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LKneePitch")
        times.append([0.4])
        keys.append([[-0.0923279, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append([[0.771559, [3, -0.146667, 0], [3, 0.186667, 0]],
                     [0.774808, [3, -0.186667, -0.00206391], [3, 0.146667, 0.00162164]],
                     [0.782616, [3, -0.146667, -0.00216065], [3, 0.2, 0.00294634]],
                     [0.790128, [3, -0.2, 0], [3, 0.133333, 0]], [0.782616, [3, -0.133333, 0], [3, 0.186667, 0]],
                     [2.01497, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append([[0.0797259, [3, -0.146667, 0], [3, 0.186667, 0]],
                     [0.0788562, [3, -0.186667, 0.000647972], [3, 0.146667, -0.000509121]],
                     [0.0762547, [3, -0.146667, 0.00133235], [3, 0.2, -0.00181685]],
                     [0.0694086, [3, -0.2, 0], [3, 0.133333, 0]], [0.0762547, [3, -0.133333, 0], [3, 0.186667, 0]],
                     [0.0264343, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append([[-0.188724, [3, -0.146667, 0], [3, 0.186667, 0]], [-0.189255, [3, -0.186667, 0], [3, 0.146667, 0]],
                     [-0.0167174, [3, -0.146667, 0], [3, 0.2, 0]], [-1.82049, [3, -0.2, 0], [3, 0.133333, 0]],
                     [-1.54395, [3, -0.133333, -0.0944333], [3, 0.186667, 0.132207]],
                     [-1.14057, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RAnklePitch")
        times.append([0.4])
        keys.append([[0.0874194, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RAnkleRoll")
        times.append([0.4])
        keys.append([[0.109102, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append([[0.621311, [3, -0.146667, 0], [3, 0.186667, 0]],
                     [0.765735, [3, -0.186667, -0.0360534], [3, 0.146667, 0.0283277]],
                     [0.814454, [3, -0.146667, -0.00510808], [3, 0.2, 0.00696557]],
                     [0.82142, [3, -0.2, 0], [3, 0.133333, 0]], [0.812292, [3, -0.133333, 0], [3, 0.186667, 0]],
                     [1.5392, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append([[1.52322, [3, -0.146667, 0], [3, 0.186667, 0]], [1.53252, [3, -0.186667, 0], [3, 0.146667, 0]],
                     [1.53203, [3, -0.146667, 0.000492152], [3, 0.2, -0.000671117]],
                     [1.52334, [3, -0.2, 0], [3, 0.133333, 0]],
                     [1.53203, [3, -0.133333, -0.00582284], [3, 0.186667, 0.00815197]],
                     [1.56527, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append([[0.998363, [3, -0.146667, 0], [3, 0.186667, 0]], [0.998363, [3, -0.186667, 0], [3, 0.146667, 0]],
                     [0.995963, [3, -0.146667, 0.000480046], [3, 0.2, -0.000654608]],
                     [0.994959, [3, -0.2, 0.00100392], [3, 0.133333, -0.000669281]],
                     [0, [3, -0.133333, 0], [3, 0.186667, 0]], [0.0060968, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RHipPitch")
        times.append([0.4])
        keys.append([[0.126321, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RHipRoll")
        times.append([0.4])
        keys.append([[-0.11736, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RHipYawPitch")
        times.append([0.4])
        keys.append([[0.737377, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RKneePitch")
        times.append([0.4])
        keys.append([[-0.0894577, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append([[0.65506, [3, -0.146667, 0], [3, 0.186667, 0]], [0.822434, [3, -0.186667, 0], [3, 0.146667, 0]],
                     [0.77649, [3, -0.146667, 0.00131187], [3, 0.2, -0.00178891]],
                     [0.774701, [3, -0.2, 0], [3, 0.133333, 0]],
                     [0.779286, [3, -0.133333, -0.0045849], [3, 0.186667, 0.00641886]],
                     [1.98158, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append(
            [[0.0106959, [3, -0.146667, 0], [3, 0.186667, 0]], [0.00781977, [3, -0.186667, 0], [3, 0.146667, 0]],
             [0.0682579, [3, -0.146667, 0], [3, 0.2, 0]], [0.0505979, [3, -0.2, 0], [3, 0.133333, 0]],
             [0.0554728, [3, -0.133333, 0], [3, 0.186667, 0]], [-0.0834505, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([0.4, 0.96, 1.4, 2, 2.4, 2.96])
        keys.append([[0.00916195, [3, -0.146667, 0], [3, 0.186667, 0]],
                     [0.00862908, [3, -0.186667, 0.000532867], [3, 0.146667, -0.000418681]],
                     [-1.58205, [3, -0.146667, 0.000378977], [3, 0.2, -0.000516786]],
                     [-1.58257, [3, -0.2, 0], [3, 0.133333, 0]], [-1.58112, [3, -0.133333, 0], [3, 0.186667, 0]],
                     [-1.66805, [3, -0.186667, 0], [3, 0, 0]]])

        try:
            motion = ALProxy("ALMotion", self.IP, self.PORT)
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err

    def AddressingTheBall(self):
        names = list()
        times = list()
        keys = list()

        names.append("LElbowRoll")
        times.append([0.56, 1.08, 1.52])
        keys.append([[-1.54341, [3, -0.2, 0], [3, 0.173333, 0]], [-0.822267, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [-0.83222, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([0.56, 1.08, 1.52])
        keys.append([[-1.71474, [3, -0.2, 0], [3, 0.173333, 0]], [-1.53416, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [-1.54126, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([0.56, 1.08, 1.52])
        keys.append([[0.5268, [3, -0.2, 0], [3, 0.173333, 0]], [0.00439295, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.00459188, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([0.56, 1.08, 1.52])
        keys.append([[2.01823, [3, -0.2, 0], [3, 0.173333, 0]], [0.784998, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.788002, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([0.56, 1.08, 1.52])
        keys.append([[-0.0402248, [3, -0.2, 0], [3, 0.173333, 0]], [0.0738154, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.0683689, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([0.56, 1.08, 1.52])
        keys.append([[-1.13827, [3, -0.2, 0], [3, 0.173333, 0]], [-1.70836, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [-1.70195, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([0.56, 1.08, 1.52])
        keys.append([[1.54339, [3, -0.2, 0], [3, 0.173333, 0]], [0.816889, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.826612, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([0.56, 1.08, 1.52])
        keys.append(
            [[1.54622, [3, -0.2, 0], [3, 0.173333, 0]], [1.53, [3, -0.173333, 0.00434004], [3, 0.146667, -0.00367234]],
             [1.52218, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([0.56, 1.08, 1.52])
        keys.append([[0, [3, -0.2, 0], [3, 0.173333, 0]], [0, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.99973, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([0.56, 1.08, 1.52])
        keys.append([[2.00566, [3, -0.2, 0], [3, 0.173333, 0]], [0.778104, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.780231, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([0.56, 1.08, 1.52])
        keys.append([[-0.025278, [3, -0.2, 0], [3, 0.173333, 0]], [0.0570375, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.0470152, [3, -0.146667, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([0.56, 1.08, 1.52])
        keys.append([[-1.66903, [3, -0.2, 0], [3, 0.173333, 0]], [-1.58255, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [-1.59122, [3, -0.146667, 0], [3, 0, 0]]])

        try:
            motion = ALProxy("ALMotion", self.IP, self.PORT)
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err

    def Batting(self, speed):
        names = list()
        times = list()
        keys = list()

        names.append("LElbowRoll")
        times.append([0, speed])
        keys.append([[-0.822266, [3, -0.0133333, 0], [3, 0.186667, 0]], [-0.819703, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([0, speed])
        keys.append([[-1.53416, [3, -0.0133333, 0], [3, 0.186667, 0]], [-1.5334, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([0, speed])
        keys.append([[0.00439295, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.00413917, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([0, speed])
        keys.append([[0.784999, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.782616, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([0, speed])
        keys.append([[0.0738153, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.0762547, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([0, speed])
        keys.append([[-1.70836, [3, -0.0133333, 0], [3, 0.186667, 0]], [-0.0167174, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([0, speed])
        keys.append([[0.816889, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.814454, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([0, speed])
        keys.append([[1.53, [3, -0.0133333, 0], [3, 0.186667, 0]], [1.53203, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([0, speed])
        keys.append([[0.995607, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.995963, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([0, speed])
        keys.append([[0.778105, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.77649, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([0, speed])
        keys.append([[0.0570375, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.0682579, [3, -0.186667, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([0, speed])
        keys.append([[-1.58255, [3, -0.0133333, 0], [3, 0.186667, 0]], [-1.58205, [3, -0.186667, 0], [3, 0, 0]]])

        try:
            motion = ALProxy("ALMotion", self.IP, self.PORT)
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err

    def ReceivingPole(self):
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([0.44, 1.28])
        keys.append([[-0.13086, [3, -0.16, 0], [3, 0.28, 0]], [-0.13086, [3, -0.28, 0], [3, 0, 0]]])

        names.append("HeadYaw")
        times.append([0.44, 1.28])
        keys.append([[-0.00781646, [3, -0.16, 0], [3, 0.28, 0]], [-0.00781646, [3, -0.28, 0], [3, 0, 0]]])

        names.append("LElbowRoll")
        times.append([0.44, 1.28, 1.96])
        keys.append(
            [[-0.819703, [3, -0.16, 0], [3, 0.28, 0]], [-0.823499, [3, -0.28, 0.00379607], [3, 0.226667, -0.00307301]],
             [-1.53812, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([0.44, 1.28, 1.96])
        keys.append(
            [[-1.5334, [3, -0.16, 0], [3, 0.28, 0]], [-1.53908, [3, -0.28, 0.00568625], [3, 0.226667, -0.00460315]],
             [-1.72773, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([0.44, 1.28, 1.96])
        keys.append([[0.00688641, [3, -0.16, 0], [3, 0.28, 0]],
                     [0.00689399, [3, -0.28, -7.58003e-06], [3, 0.226667, 6.13622e-06]],
                     [0.526673, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([0.44, 1.28, 1.96])
        keys.append(
            [[0.782616, [3, -0.16, 0], [3, 0.28, 0]], [0.790191, [3, -0.28, -0.00757476], [3, 0.226667, 0.00613195]],
             [2.00793, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([0.44, 1.28, 1.96])
        keys.append(
            [[0.0762547, [3, -0.16, 0], [3, 0.28, 0]], [0.0687017, [3, -0.28, 0.00755291], [3, 0.226667, -0.00611426]],
             [0.0290275, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([0.44, 1.28, 1.96])
        keys.append([[-1.7683, [3, -0.16, 0], [3, 0.28, 0]], [-1.78024, [3, -0.28, 0], [3, 0.226667, 0]],
                     [-1.14443, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([0.44, 1.28, 1.96])
        keys.append(
            [[0.812292, [3, -0.16, 0], [3, 0.28, 0]], [0.814847, [3, -0.28, -0.00255517], [3, 0.226667, 0.00206847]],
             [1.53801, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([0.44, 1.28, 1.96])
        keys.append(
            [[1.53203, [3, -0.16, 0], [3, 0.28, 0]], [1.53849, [3, -0.28, -0.0064595], [3, 0.226667, 0.00522912]],
             [1.5705, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([0.44, 1.28, 1.96])
        keys.append([[0.995963, [3, -0.16, 0], [3, 0.28, 0]], [0, [3, -0.28, 0], [3, 0.226667, 0]],
                     [0.000917273, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([0.44, 1.28, 1.96])
        keys.append(
            [[0.779286, [3, -0.16, 0], [3, 0.28, 0]], [0.781223, [3, -0.28, -0.00193732], [3, 0.226667, 0.0015683]],
             [1.9869, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([0.44, 1.28, 1.96])
        keys.append(
            [[0.0554728, [3, -0.16, 0], [3, 0.28, 0]], [0.0478072, [3, -0.28, 0.00766566], [3, 0.226667, -0.00620553]],
             [-0.0770126, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([0.44, 1.28, 1.96])
        keys.append(
            [[-1.58112, [3, -0.16, 0], [3, 0.28, 0]], [-1.59005, [3, -0.28, 0.00892918], [3, 0.226667, -0.00722838]],
             [-1.66302, [3, -0.226667, 0], [3, 0, 0]]])

        try:
            motion = ALProxy("ALMotion", self.IP, self.PORT)
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err

    def AddressingTheBall2(self):
        names = list()
        times = list()
        keys = list()

        names.append("LElbowRoll")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append([[-1.54341, [3, -0.173333, 0], [3, 0.146667, 0]], [-0.822267, [3, -0.146667, 0], [3, 0.16, 0]],
                     [-0.83222, [3, -0.16, 0.00198297], [3, 0.173333, -0.00214822]],
                     [-0.83466, [3, -0.173333, 0], [3, 0.146667, 0]], [-0.83466, [3, -0.146667, 0], [3, 0.173333, 0]],
                     [-0.83466, [3, -0.173333, 0], [3, 0.186667, 0]], [-0.83466, [3, -0.186667, 0], [3, 0.2, 0]],
                     [-0.844776, [3, -0.2, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append([[-1.71474, [3, -0.173333, 0], [3, 0.146667, 0]], [-1.53416, [3, -0.146667, 0], [3, 0.16, 0]],
                     [-1.54126, [3, -0.16, 0], [3, 0.173333, 0]], [-1.53233, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [-1.53233, [3, -0.146667, 0], [3, 0.173333, 0]], [-1.53233, [3, -0.173333, 0], [3, 0.186667, 0]],
                     [-1.53233, [3, -0.186667, 0], [3, 0.2, 0]], [-1.48139, [3, -0.2, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append([[0.5268, [3, -0.173333, 0], [3, 0.146667, 0]], [0.00439295, [3, -0.146667, 0], [3, 0.16, 0]],
                     [0.00459188, [3, -0.16, -4.04784e-05], [3, 0.173333, 4.38516e-05]],
                     [0.00464594, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.00464594, [3, -0.146667, 0], [3, 0.173333, 0]],
                     [0.00464594, [3, -0.173333, 0], [3, 0.186667, 0]], [0.00464594, [3, -0.186667, 0], [3, 0.2, 0]],
                     [0.00581758, [3, -0.2, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append([[2.01823, [3, -0.173333, 0], [3, 0.146667, 0]], [0.784998, [3, -0.146667, 0], [3, 0.16, 0]],
                     [0.788002, [3, -0.16, -0.00144039], [3, 0.173333, 0.00156042]],
                     [0.794001, [3, -0.173333, 0], [3, 0.146667, 0]], [0.794001, [3, -0.146667, 0], [3, 0.173333, 0]],
                     [0.093491, [3, -0.173333, 0], [3, 0.186667, 0]], [0.093491, [3, -0.186667, 0], [3, 0.2, 0]],
                     [0.873171, [3, -0.2, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append([[-0.0402248, [3, -0.173333, 0], [3, 0.146667, 0]], [0.0738154, [3, -0.146667, 0], [3, 0.16, 0]],
                     [0.0683689, [3, -0.16, 0], [3, 0.173333, 0]], [0.0782558, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.0782558, [3, -0.146667, 0], [3, 0.173333, 0]], [0.0782558, [3, -0.173333, 0], [3, 0.186667, 0]],
                     [0.0782558, [3, -0.186667, 0], [3, 0.2, 0]], [-0.0266295, [3, -0.2, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append([[-1.13827, [3, -0.173333, 0], [3, 0.146667, 0]], [-1.70836, [3, -0.146667, 0], [3, 0.16, 0]],
                     [-1.70195, [3, -0.16, -0.00230355], [3, 0.173333, 0.00249551]],
                     [-1.69396, [3, -0.173333, 0], [3, 0.146667, 0]], [-1.69396, [3, -0.146667, 0], [3, 0.173333, 0]],
                     [-1.69396, [3, -0.173333, 0], [3, 0.186667, 0]],
                     [-0.0700772, [3, -0.186667, -0.0441683], [3, 0.2, 0.0473232]],
                     [-0.022754, [3, -0.2, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append([[1.54339, [3, -0.173333, 0], [3, 0.146667, 0]], [0.816889, [3, -0.146667, 0], [3, 0.16, 0]],
                     [0.826612, [3, -0.16, -0.00972327], [3, 0.173333, 0.0105335]],
                     [1.35599, [3, -0.173333, 0], [3, 0.146667, 0]], [1.35599, [3, -0.146667, 0], [3, 0.173333, 0]],
                     [1.35599, [3, -0.173333, 0], [3, 0.186667, 0]], [1.35599, [3, -0.186667, 0], [3, 0.2, 0]],
                     [0.819504, [3, -0.2, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append(
            [[1.54622, [3, -0.173333, 0], [3, 0.146667, 0]], [1.53, [3, -0.146667, 0.00383195], [3, 0.16, -0.00418031]],
             [1.52218, [3, -0.16, 0], [3, 0.173333, 0]], [1.5245, [3, -0.173333, 0], [3, 0.146667, 0]],
             [1.5245, [3, -0.146667, 0], [3, 0.173333, 0]], [1.5245, [3, -0.173333, 0], [3, 0.186667, 0]],
             [1.5245, [3, -0.186667, 0], [3, 0.2, 0]], [1.52307, [3, -0.2, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append([[0, [3, -0.173333, 0], [3, 0.146667, 0]], [0, [3, -0.146667, 0], [3, 0.16, 0]],
                     [0.99973, [3, -0.16, 0], [3, 0.173333, 0]], [0.991605, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.991605, [3, -0.146667, 0], [3, 0.173333, 0]], [0.991605, [3, -0.173333, 0], [3, 0.186667, 0]],
                     [0.991605, [3, -0.186667, 0], [3, 0.2, 0]], [0.992216, [3, -0.2, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append([[2.00566, [3, -0.173333, 0], [3, 0.146667, 0]], [0.778104, [3, -0.146667, 0], [3, 0.16, 0]],
                     [0.780232, [3, -0.16, 0], [3, 0.173333, 0]], [0.153451, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.153451, [3, -0.146667, 0], [3, 0.173333, 0]], [0.153451, [3, -0.173333, 0], [3, 0.186667, 0]],
                     [0.153451, [3, -0.186667, 0], [3, 0.2, 0]], [0.770015, [3, -0.2, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append([[-0.025278, [3, -0.173333, 0], [3, 0.146667, 0]], [0.0570375, [3, -0.146667, 0], [3, 0.16, 0]],
                     [0.0470152, [3, -0.16, 0], [3, 0.173333, 0]], [0.0486284, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [0.0486284, [3, -0.146667, 0], [3, 0.173333, 0]], [0.0486284, [3, -0.173333, 0], [3, 0.186667, 0]],
                     [0.0486284, [3, -0.186667, 0], [3, 0.2, 0]], [0.048196, [3, -0.2, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([0.48, 0.92, 1.4, 1.92, 2.36, 2.88, 3.44, 4.04])
        keys.append([[-1.66903, [3, -0.173333, 0], [3, 0.146667, 0]], [-1.58255, [3, -0.146667, 0], [3, 0.16, 0]],
                     [-1.59122, [3, -0.16, 0], [3, 0.173333, 0]], [-1.58928, [3, -0.173333, 0], [3, 0.146667, 0]],
                     [-1.58928, [3, -0.146667, 0], [3, 0.173333, 0]], [-1.58928, [3, -0.173333, 0], [3, 0.186667, 0]],
                     [-1.58928, [3, -0.186667, 0], [3, 0.2, 0]], [-1.59736, [3, -0.2, 0], [3, 0, 0]]])

        try:
            motion = ALProxy("ALMotion", self.IP, self.PORT)
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print
            err

    def Batting2(self, speed):
        names = list()
        times = list()
        keys = list()

        names.append("LElbowRoll")
        times.append([0, speed])
        keys.append([[-0.844776, [3, -0.0133333, 0], [3, 0.12, 0]], [-0.82291, [3, -0.12, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([0, speed])
        keys.append([[-1.48139, [3, -0.0133333, 0], [3, 0.12, 0]], [-1.53233, [3, -0.12, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([0, speed])
        keys.append([[0.00581758, [3, -0.0133333, 0], [3, 0.12, 0]], [0.00464594, [3, -0.12, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([0, speed])
        keys.append([[0.87317, [3, -0.0133333, 0], [3, 0.12, 0]], [0.776478, [3, -0.12, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([0, speed])
        keys.append([[-0.0266295, [3, -0.0133333, 0], [3, 0.12, 0]], [-0.0331613, [3, -0.12, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([0, speed])
        keys.append([[-0.022754, [3, -0.0133333, 0], [3, 0.12, 0]], [-1.78322, [3, -0.12, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([0, speed])
        keys.append([[0.819503, [3, -0.0133333, 0], [3, 0.12, 0]], [0.818825, [3, -0.12, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([0, speed])
        keys.append([[1.52307, [3, -0.0133333, 0], [3, 0.12, 0]], [1.5245, [3, -0.12, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([0, speed])
        keys.append([[0.992216, [3, -0.0133333, 0], [3, 0.12, 0]], [0.991605, [3, -0.12, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([0, speed])
        keys.append([[0.770016, [3, -0.0133333, 0], [3, 0.12, 0]], [0.766861, [3, -0.12, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([0, speed])
        keys.append([[0.048196, [3, -0.0133333, 0], [3, 0.12, 0]], [0.0597576, [3, -0.12, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([0, speed])
        keys.append([[-1.59736, [3, -0.0133333, 0], [3, 0.12, 0]], [-1.58928, [3, -0.12, 0], [3, 0, 0]]])

        try:
            motion = ALProxy("ALMotion", self.IP, self.PORT)
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print
            err

    def MoveEnd(self):
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([0.88])
        keys.append([[-0.16734, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("HeadYaw")
        times.append([0.88])
        keys.append([[7.10543e-15, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LAnklePitch")
        times.append([0.88])
        keys.append([[0.0874194, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LAnkleRoll")
        times.append([0.88])
        keys.append([[-0.110027, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LElbowRoll")
        times.append([0.88])
        keys.append([[-1.49709, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([0.88])
        keys.append([[-1.57219, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([0.88])
        keys.append([[1.73002e-05, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LHipPitch")
        times.append([0.88])
        keys.append([[0.126915, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LHipRoll")
        times.append([0.88])
        keys.append([[0.118283, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LHipYawPitch")
        times.append([0.88])
        keys.append([[-0.170215, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LKneePitch")
        times.append([0.88])
        keys.append([[-0.0910193, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([0.88])
        keys.append([[1.84076, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([0.88])
        keys.append([[0.0769872, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([0.88])
        keys.append([[-1.55302, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RAnklePitch")
        times.append([0.88])
        keys.append([[0.0874194, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RAnkleRoll")
        times.append([0.88])
        keys.append([[0.11002, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([0.88])
        keys.append([[1.51008, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([0.88])
        keys.append([[1.54362, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([0.88])
        keys.append([[0.245015, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RHipPitch")
        times.append([0.88])
        keys.append([[0.126918, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RHipRoll")
        times.append([0.88])
        keys.append([[-0.118308, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RHipYawPitch")
        times.append([0.88])
        keys.append([[-0.170215, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RKneePitch")
        times.append([0.88])
        keys.append([[-0.0910193, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([0.88])
        keys.append([[1.5798, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([0.88])
        keys.append([[0.0247619, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([0.88])
        keys.append([[-1.78209, [3, -0.306667, 0], [3, 0, 0]]])

        try:
            motion = ALProxy("ALMotion", self.IP, self.PORT)
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err


def main(RobotIP, PORT):
    Motion = MotionClass(RobotIP, PORT)
    Motion.Stand()
    Motion.Ready()
    run(RobotIP,PORT)
    Motion.HoldingPole()
    Move = Motion.Move(RobotIP,PORT)
    Move.MoveT(0.5, 0)
    motionProxy = ALProxy("ALMotion", RobotIP, PORT)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.43.165",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)