# coding=utf-8
from TargetFeature import HogFeature, ColorFeature
from Classifier import KNN
from naoqi import ALProxy
from PIL import Image

import numpy as np
import cv2
import os
import time


class StickDetect(object):
    """调用compute_StickPosition，计算杆的方位角，距离"""

    def __init__(self, robotIp, port=9559):
        self.img = None
        self.robotIp = robotIp
        self.port = port

    # noinspection PyMethodMayBeStatic
    def reshapeStickRect(self, rawRect, numbers):
        newRect = np.zeros((numbers, 4))
        initX, initY, endX, endY = rawRect[0], rawRect[1], rawRect[2], rawRect[3]  # 初始化参数

        # 找出每个小矩阵的顶点坐标
        for i in range(numbers):
            newRect[i][0] = initX
            newRect[i][1] = initY + ((endY - initY) / numbers) * i
            newRect[i][2] = endX
            newRect[i][3] = initY + ((endY - initY) / numbers) * (i + 1)

        return newRect  # 四等分矩阵信息

    # noinspection PyMethodMayBeStatic
    def calColorFeature(self, img, number=16):
        color = ColorFeature(img, number)
        result = color.colorExtract(img)

        return np.round(result, 4)

    # noinspection PyMethodMayBeStatic
    def calHOGFeature(self, img, cellSize):
        rectstickArea = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hog = HogFeature(rectstickArea, cellSize)
        vector, _ = hog.hog_extract()

        return np.round(vector[0], 4)

    # noinspection PyMethodMayBeStatic
    def bilateralFilter(self, img):
        """双边滤波"""
        bilateral = cv2.bilateralFilter(img, 25, 12.5, 50)
        return bilateral

    # noinspection PyMethodMayBeStatic
    def Filter(self, binImg):
        """
        腐蚀，膨胀，高斯滤波
        :param binImg:二值图
        :return:处理结束的图
        """
        kernelErosion = np.ones((5, 5), np.uint8)
        kernelDilation = np.ones((5, 5), np.uint8)
        frameBin = cv2.erode(binImg, kernelErosion, iterations=1)
        frameBin = cv2.dilate(frameBin, kernelDilation, iterations=1)
        frameBin = cv2.GaussianBlur(frameBin, (9, 9), 0)

        return frameBin

    def contoursDetection(self, img):  # 检测图像边界

        minPerimeter = 100
        minArea = 850

        # 预处理
        bilateral = self.bilateralFilter(img)
        HSVImg = cv2.cvtColor(bilateral, cv2.COLOR_BGR2HSV)  # 转到HSV空间
        hmin, hmax, smin, vmin = 27, 45, 55, 115
        # 二值化处理
        minHSV = np.array([hmin, smin, vmin])
        maxHSV = np.array([hmax, 255, 255])
        binImg = cv2.inRange(HSVImg, minHSV, maxHSV)

        # 图像滤波处理（腐蚀，膨胀，高斯）
        frameBin = self.Filter(binImg)

        rects = []
        if cv2.__version__.split(".")[0] == "3":  # for OpenCV >= 3.0.0
            _, contours, _ = cv2.findContours(frameBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        else:
            contours, _ = cv2.findContours(frameBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            return rects
        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            area = cv2.contourArea(contour)
            if perimeter > minPerimeter and area > minArea:
                x, y, w, h = cv2.boundingRect(contour)
                rects.append([x, y, w, h])
        return rects

    # noinspection PyMethodMayBeStatic
    def compute_score(self, rects):
        """计算score"""
        for rect in rects:
            w, h = rect[2], rect[3]
            rect.append(int(10000 * abs(float(w) / h - 0.1)))
        return rects

    def nms(self, rects):
        """非极大值抑制"""
        rects = self.compute_score(rects=rects)
        rects.sort(
            cmp=lambda rects1, rects2: (rects1[4] - rects2[4]))
        return rects[0]

    def result(self):
        img = self.img
        Rects = []
        knn = KNN("data_stick.txt")
        # 返回检测到的矩形矩阵
        # 只需要检测边缘，返回边缘所在的矩阵，返回的轮廓
        rects = self.contoursDetection(self.img)

        if len(rects) == 0:
            print("no rects")
            return []

        for rect in rects:
            rect_ = rect[:4]
            resultTotal = []
            if rect[0] < 0 or rect[1] < 0 or rect[2] + rect[0] > 640 or rect[3] + rect[1] > 480:
                print("out of bound")
                continue
            rect[2] += rect[0]
            rect[3] += rect[1]
            newRects = self.reshapeStickRect(rect, 4)

            for newRect in newRects:
                newInitX, newInitY = int(newRect[0]), int(newRect[1])
                newEndX, newEndY = int(newRect[2]), int(newRect[3])
                rectStickArea = img[newInitY:newEndY, newInitX:newEndX, :]

                resultColor = self.calColorFeature(rectStickArea, 16)
                cellSize = min(newEndX - newInitX, newEndY - newInitY)
                resultHOG = self.calHOGFeature(rectStickArea, cellSize / 2)
                resultTotal.extend(resultColor)
                resultTotal.extend(resultHOG)

            resultTotal = np.array(resultTotal).reshape(1, -1).astype('float64')
            classify = knn.classifyVector(resultTotal)

            if classify == 1:
                Rects.append(rect_)

        stick_rectangle = self.nms(Rects)  # 非极大值抑制

        return stick_rectangle

    def takePhoto(self):
        """
        拍照
        :return:numpy array, toptopCameraX, topCameraY, topCameraHeight
        """
        robotIp = self.robotIp
        port = self.port
        camProxy = ALProxy("ALVideoDevice", robotIp, port)
        motionProxy = ALProxy("ALMotion", robotIp, port)
        camProxy.setActiveCamera(1)
        resolution = 2  # VGA
        colorSpace = 11  # RGB
        topCamera = 0
        subscriberID = "subscriberID"

        cameraPosition = motionProxy.getPosition("CameraTop", 2, True)
        cameraAngles = motionProxy.getAngles("Head", True)
        subscriberID = camProxy.subscribeCamera(subscriberID, topCamera, resolution, colorSpace, 5)
        naoImage = camProxy.getImageRemote(subscriberID)

        camProxy.releaseImage(subscriberID)
        camProxy.unsubscribe(subscriberID)
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]

        img = Image.frombytes("RGB", (imageWidth, imageHeight), array)

        topCameraX = cameraPosition[0]
        topCameraY = cameraPosition[1]
        cameraHeight = cameraPosition[2]

        # PIL image 转cv2 image
        open_cv_image = np.array(img)
        img = cv2.cvtColor(open_cv_image, cv2.COLOR_RGB2BGR)
        return img, topCameraX, topCameraY, cameraHeight, cameraAngles

    # noinspection PyMethodMayBeStatic
    def distance_fixing(self, stickDistance):
        """误差补偿"""
        x = stickDistance
        a = -0.3165899349995387
        b = 1.150241967154712
        fx = (x - a) / b
        return fx

    # noinspection PyMethodMayBeStatic
    def compute_StickPosition(self, stickH=0.47):
        """
        调用此方法，计算杆的方位角，距离
        :param stickH: stick Height
        :return: stickAngleX, stickDistance, img 标注了黄杆的图片
        """
        motion = ALProxy("ALMotion", self.robotIp, self.port)
        motion.setAngles("HeadPitch", 16 * np.pi / 180, 0.3)
        time.sleep(1)
        img, topCameraX, topCameraY, cameraHeight, cameraAngles = self.takePhoto()
        cameraYaw, cameraPitch = cameraAngles
        self.img = img
        imageHeight, imageWidth, _ = img.shape
        stick_rect = self.result()
        if len(stick_rect) is not 0:
            sx = stick_rect[0]
            sy = stick_rect[1]
            sw = stick_rect[2]
            sh = stick_rect[3]
            centerY = sy + sh / 2
            centerX = sx + 0.1 * sh / 2
            # 画出目标
            cv2.rectangle(img, (sx, sy), (sx + sw, sy + sh), (25, 0, 255), 2)
            cameraX = 60.97 * np.pi / 180
            cameraY = 47.64 * np.pi / 180
            stickAngleX = cameraYaw + (imageWidth / 2.0 - centerX) / imageWidth * cameraX
            stickAngleY = cameraPitch + (centerY - imageHeight / 2.0) / imageHeight * cameraY
            print("stickAngleY: {}".format(stickAngleY))
            stickDistance = (cameraHeight - stickH / 2.0) / np.tan(stickAngleY)
            print("stickDistance: {}".format(stickDistance))
            stickDistance = self.distance_fixing(stickDistance)

            return stickAngleX, stickDistance, img
        else:
            print("no stick")
            return []
