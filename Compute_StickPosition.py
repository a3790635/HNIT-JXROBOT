# coding=utf-8
import numpy as np
import cv2
import os
import time
import math
from TargetFeature import HogFeature, ColorFeature
from Classifier import KNN
from naoqi import ALProxy
from PIL import Image
import almath


class RedBallDetection(object):
    """调用compute_ballPosition，计算杆的方位角，距离"""

    def __init__(self, robotIp, port=9559):
        self.img = None
        self.robotIp = robotIp
        self.port = port

    # noinspection PyMethodMayBeStatic
    def __compute_score(self, rects):
        """计算score"""

        minHSV1 = np.array([0, 43, 46])
        maxHSV1 = np.array([10, 255, 255])
        minHSV2 = np.array([156, 43, 46])
        maxHSV2 = np.array([180, 255, 255])

        img = self.img.copy()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        for rect in rects:
            x, y, r = int(rect[0]), int(rect[1]), int(rect[2])
            H = hsv[y - r:y + r, x - r:x + r, 0]
            S = hsv[y - r:y + r, x - r:x + r, 1]
            V = hsv[y - r:y + r, x - r:x + r, 2]
            H = (np.where(H >= minHSV1[0]) and np.where(H <= maxHSV1[0])) or (
                    np.where(H >= minHSV2[0]) and np.where(H <= maxHSV2[0]))
            S = (np.where(S >= minHSV1[1]) and np.where(S <= maxHSV1[1])) or (
                    np.where(S >= minHSV2[2]) and np.where(S <= maxHSV2[2]))
            V = (np.where(V >= minHSV1[2]) and np.where(V <= maxHSV1[2])) or (
                    np.where(V >= minHSV2[2]) and np.where(V <= maxHSV2[2]))

            rect.append(10000 * np.min([len(H), len(S), len(V)]) / float(r ** 2))
        return rects

    def __nms(self, rects):
        """非极大值抑制"""
        rects = self.__compute_score(rects=rects)
        rects.sort(
            cmp=lambda rects1, rects2: (rects2[3] - rects1[3]))
        return rects[0]

    def __HoughDetection(self):
        """霍夫圆检测"""
        img = self.img.copy()
        binImg = self.__preProcess(img)
        circles = cv2.HoughCircles(binImg, cv2.HOUGH_GRADIENT, 1, 100,
                                   param1=150, param2=15, minRadius=8, maxRadius=48)
        if circles is None:
            circles = []
            # print("no circle")
        else:
            circles = circles[0, :]
        return circles

    def __preProcess(self, img):
        """图像处理"""
        HSVImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        minHSV1 = np.array([0, 43, 46])
        maxHSV1 = np.array([10, 255, 255])
        minHSV2 = np.array([156, 43, 46])
        maxHSV2 = np.array([180, 255, 255])

        # 二值化处理
        binImg1 = cv2.inRange(HSVImg, minHSV1, maxHSV1)
        binImg2 = cv2.inRange(HSVImg, minHSV2, maxHSV2)
        binImg = np.maximum(binImg1, binImg2)

        # 图像滤波处理（腐蚀，膨胀，高斯）
        binImg = self.__filter(binImg)
        return binImg

    # noinspection PyMethodMayBeStatic
    def __filter(self, img):
        """滤波"""
        kernelErosion = np.ones((3, 3), np.uint8)
        kernelDilation = np.ones((3, 3), np.uint8)
        resImg = cv2.erode(img, kernelErosion, iterations=2)
        resImg = cv2.dilate(resImg, kernelDilation, iterations=3)
        resImg = cv2.GaussianBlur(resImg, (9, 9), 1.5)

        return resImg

    # noinspection PyMethodMayBeStatic
    def __reshapeBallRect(self, rawRect, numbers):
        newRect = np.zeros((numbers, 4))
        initX, initY, endX, endY = rawRect[0], rawRect[1], rawRect[2], rawRect[3]  # 初始化参数

        newRect[0] = [initX, initY, initX + (endX - initX) / 2, initY + (endY - initY) / 2]
        newRect[1] = [initX + (endX - initX) / 2, initY, endX, initY + (endY - initY) / 2]
        newRect[2] = [initX, initY + (endY - initY) / 2, initX + (endX - initX) / 2, endY]
        newRect[3] = [initX + (endX - initX) / 2, initY + (endY - initY) / 2, endX, endY]

        return newRect

    # noinspection PyMethodMayBeStatic
    def __circle2Rect(self, circle, k=1):
        """转化球在图像中的矩形坐标"""
        centerX, centerY, radius = circle[0], circle[1], circle[2]
        initX, initY = int(centerX - int(k * radius)), int(centerY - int(k * radius))
        endX, endY = int(centerX + int(k * radius)), int(centerY + int(k * radius))

        return [initX, initY, endX, endY]

    # noinspection PyMethodMayBeStatic
    def __calColorFeature(self, img, number=16):
        """提取颜色特征"""
        color = ColorFeature(img, number)
        result = color.colorExtract(img)

        return np.round(result, 4)

    # noinspection PyMethodMayBeStatic
    def __calHOGFeature(self, img, cell_size):
        """提取HOG特征"""
        rectBallArea = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hog = HogFeature(rectBallArea, cell_size)
        vector, img = hog.hog_extract()
        return np.round(vector[0], 4)

    def __result(self):
        """得到最终结果, 返回圆心，半径"""
        Rects = []
        knn = KNN("data_ball1.txt")
        srcImg = self.img.copy()
        rects = self.__HoughDetection()
        if len(rects) is 0:
            # print("no rects")
            return []

        for rect in rects:  # 检测每个轮廓
            resultTotal = []
            x, y, r = rect[:3]

            # cv2.rectangle(srcImg, (x - r, y - r), (x + r, y + r), (0, 0, 255), 2)
            # cv2.imshow("img", srcImg)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            rect_ = [x, y, r]
            rect = self.__circle2Rect(rect)
            if rect[0] < 0 or rect[1] < 0 or rect[2] > 640 or rect[3] > 480:
                print("out of bound")
                continue
            newRects = self.__reshapeBallRect(rect, 4)

            for newRect in newRects:
                newInitX, newInitY = int(newRect[0]), int(newRect[1])
                newEndX, newEndY = int(newRect[2]), int(newRect[3])
                rectBallArea = srcImg[newInitY:newEndY, newInitX:newEndX, :]

                resultColor = self.__calColorFeature(rectBallArea, 16)
                cellSize = min(newEndX - newInitX, newEndY - newInitY)
                resultHOG = self.__calHOGFeature(rectBallArea, cellSize / 2)
                resultTotal.extend(resultColor)
                resultTotal.extend(resultHOG)

            resultTotal = np.array(resultTotal).reshape(1, -1).astype('float64')
            classify = knn.classifyVector(resultTotal)

            if classify == 1:
                Rects.append(rect_)
        if len(Rects) is 0:
            return []
        Rect = self.__nms(Rects)

        return Rect

    def __takePhoto(self):
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
        bottomCamera = 1
        subscriberID = "subscriberID"

        cameraPosition = motionProxy.getPosition("CameraBottom", 2, True)
        cameraAngles = motionProxy.getAngles("Head", True)
        subscriberID = camProxy.subscribeCamera(subscriberID, bottomCamera, resolution, colorSpace, 5)
        naoImage = camProxy.getImageRemote(subscriberID)

        camProxy.releaseImage(subscriberID)
        camProxy.unsubscribe(subscriberID)
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]

        img = Image.frombytes("RGB", (imageWidth, imageHeight), array)

        bottomCameraX = cameraPosition[0]
        bottomCameraY = cameraPosition[1]
        cameraHeight = cameraPosition[2]

        # PIL image 转cv2 image
        open_cv_image = np.array(img)
        img = cv2.cvtColor(open_cv_image, cv2.COLOR_RGB2BGR)
        # cv2.imwrite('ballTest.jpg', img)
        return img, bottomCameraX, bottomCameraY, cameraHeight, cameraAngles

    # noinspection PyMethodMayBeStatic
    def __distance_fixing(self, ballDistance):
        x = ballDistance
        a = -0.3165899349995387
        b = 1.150241967154712
        fx = (x - a) / b
        return fx

    def compute_ballPosition(self, standState="standInit", ballRadius=0.021):
        """
        调用此方法，计算球的在图片中的圆心，半径，实际坐标、方位角、距离
        :param standState:stand State
        :param ballRadius:ball Radius
        :return:[centerX at img, centerY at img, radius at img, ballX, ballY, ballYaw, img]
        """
        motionProxy = ALProxy("ALMotion", self.robotIp, self.port)
        img, cameraX, cameraY, cameraHeight, cameraAngles = self.__takePhoto()
        self.img = img.copy()
        imageHeight, imageWidth, _ = img.shape
        ball_rect = self.__result()

        cameraYawRange = 60.97 * np.pi / 180
        cameraPitchRange = 47.64 * np.pi / 180

        if len(ball_rect) is not 0:
            centerX = ball_rect[0]
            centerY = ball_rect[1]
            radius = ball_rect[2]
            cv2.rectangle(img, (centerX - radius, centerY - radius), (centerX + radius, centerY + radius), (0, 0, 255),
                          2)

            bottomCameraDirection = {"standInit": 49.2, "standUp": 39.7}
            try:
                cameraDirection = bottomCameraDirection[standState]
            except KeyError:
                print("Error! unknown standState, please check the value of stand state!")
            else:
                headPitches = motionProxy.getAngles("HeadPitch", True)
                headPitch = headPitches[0]
                headYaws = motionProxy.getAngles("HeadYaw", True)
                headYaw = headYaws[0]
                ballPitch = (centerY - imageHeight / 2) * cameraPitchRange / 480.0  # y (pitch angle)
                ballYaw = (imageWidth / 2 - centerX) * cameraYawRange / 640.0  # x (yaw angle)
                dPitch = (cameraHeight - ballRadius) / np.tan(cameraDirection / 180 * np.pi + headPitch + ballPitch)
                dYaw = dPitch / np.cos(ballYaw)
                ballX = dYaw * np.cos(ballYaw + headYaw) + cameraX + 0.035
                ballY = dYaw * np.sin(ballYaw + headYaw) + cameraY
                ballYaw = np.arctan2(ballY, ballX)
                if standState == "standInit":
                    ky = 42.513 * ballX ** 4 - 109.66 * ballX ** 3 + 104.2 * ballX ** 2 - 44.218 * ballX + 8.5526
                    # ky = 12.604*ballX**4 - 37.962*ballX**3 + 43.163*ballX**2 - 22.688*ballX + 6.0526
                    ballY = ky * ballY
                    ballYaw = np.arctan2(ballY, ballX)
                return [centerX, centerY, radius, ballX, ballY, ballYaw, img]
        return []


class StickDetection(object):
    """调用compute_stickPosition，计算杆的方位角，距离"""

    def __init__(self, robotIp, port=9559):
        self.img = None
        self.robotIp = robotIp
        self.port = port

    # noinspection PyMethodMayBeStatic
    def __reshapeStickRect(self, rawRect, numbers):
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
    def __calColorFeature(self, img, number=16):
        color = ColorFeature(img, number)
        result = color.colorExtract(img)

        return np.round(result, 4)

    # noinspection PyMethodMayBeStatic
    def __calHOGFeature(self, img, cellSize):
        rectStickArea = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hog = HogFeature(rectStickArea, cellSize)
        vector, _ = hog.hog_extract()

        return np.round(vector[0], 4)

    # noinspection PyMethodMayBeStatic
    def __bilateral__filter(self, img):
        """双边滤波"""
        bilateral = cv2.bilateralFilter(img, 25, 12.5, 50)
        return bilateral

    # noinspection PyMethodMayBeStatic
    def __filter(self, binImg):
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

    def __contoursDetection(self, img):  # 检测图像边界

        minPerimeter = 100
        minArea = 850

        # 预处理
        bilateral = self.__bilateral__filter(img)
        HSVImg = cv2.cvtColor(bilateral, cv2.COLOR_BGR2HSV)  # 转到HSV空间
        hmin, hmax, smin, vmin = 27, 45, 55, 115
        # 二值化处理
        minHSV = np.array([hmin, smin, vmin])
        maxHSV = np.array([hmax, 255, 255])
        binImg = cv2.inRange(HSVImg, minHSV, maxHSV)

        # 图像滤波处理（腐蚀，膨胀，高斯）
        frameBin = self.__filter(binImg)

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
    def __compute_score(self, rects):
        """计算score"""
        for rect in rects:
            w, h = rect[2], rect[3]
            rect.append(int(10000 * abs(float(w) / h - 0.1)))
        return rects

    def __nms(self, rects):
        """非极大值抑制"""
        rects = self.__compute_score(rects=rects)
        rects.sort(
            cmp=lambda rects1, rects2: (rects1[4] - rects2[4]))
        return rects[0]

    def __result(self):
        img = self.img
        Rects = []
        knn = KNN("data_stick.txt")
        # 返回检测到的矩形矩阵
        # 只需要检测边缘，返回边缘所在的矩阵，返回的轮廓
        rects = self.__contoursDetection(self.img)

        if len(rects) == 0:
            # print("no rects")
            return []

        for rect in rects:
            rect_ = rect[:4]
            resultTotal = []
            if rect[0] < 0 or rect[1] < 0 or rect[2] + rect[0] > 640 or rect[3] + rect[1] > 480:
                print("out of bound")
                continue
            rect[2] += rect[0]
            rect[3] += rect[1]
            newRects = self.__reshapeStickRect(rect, 4)

            for newRect in newRects:
                newInitX, newInitY = int(newRect[0]), int(newRect[1])
                newEndX, newEndY = int(newRect[2]), int(newRect[3])
                rectStickArea = img[newInitY:newEndY, newInitX:newEndX, :]

                resultColor = self.__calColorFeature(rectStickArea, 16)
                cellSize = min(newEndX - newInitX, newEndY - newInitY)
                resultHOG = self.__calHOGFeature(rectStickArea, cellSize / 2)
                resultTotal.extend(resultColor)
                resultTotal.extend(resultHOG)

            resultTotal = np.array(resultTotal).reshape(1, -1).astype('float64')
            classify = knn.classifyVector(resultTotal)

            if classify == 1:
                Rects.append(rect_)

        stick_rectangle = self.__nms(Rects)  # 非极大值抑制

        return stick_rectangle

    def __takePhoto(self):
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
    def __distance_fixing(self, stickDistance):
        x = stickDistance
        a = -0.3165899349995387
        b = 1.150241967154712
        fx = (x - a) / b
        return fx

    # noinspection PyMethodMayBeStatic
    def compute_stickPosition(self, stickH=0.47):
        """
        调用此方法，计算杆的方位角，距离
        :param stickH: stick Height
        :return: stickAngleX, stickDistance, img 标注了黄杆的图片
        """
        time.sleep(0.05)
        img, topCameraX, topCameraY, cameraHeight, cameraAngles = self.__takePhoto()
        cameraYaw, cameraPitch = cameraAngles
        self.img = img
        imageHeight, imageWidth, _ = img.shape
        stick_rect = self.__result()
        if len(stick_rect) is not 0:
            sx = stick_rect[0]
            sy = stick_rect[1]
            sw = stick_rect[2]
            sh = stick_rect[3]
            centerY = sy + sh / 2
            centerX = sx + 0.1 * sh / 2
            # 画出目标
            cv2.rectangle(img, (sx, sy), (sx + sw, sy + sh), (0, 0, 255), 2)

            cameraRangeX = 60.97 * np.pi / 180
            cameraRangeY = 47.64 * np.pi / 180

            stickAngleX = cameraYaw + (imageWidth / 2.0 - centerX) / imageWidth * cameraRangeX
            stickAngleY = cameraPitch + (centerY - imageHeight / 2.0) / imageHeight * cameraRangeY

            stickDistance = (cameraHeight - stickH / 2.0) / np.tan(stickAngleY)
            stickDistance = self.__distance_fixing(stickDistance)

            return stickAngleX, stickDistance, img
        else:
            print("no stick")
            return []


class LandMarkDetection(object):
    """调用compute_markPosition,获取LandMark坐标，距离，角度"""

    def __init__(self, robotIp, port=9559):
        self.robotIp = robotIp
        self.port = port

    def compute_markPosition(self, landmarkTheoreticalSize=0.11):
        """
        调用此函数，获取LandMark坐标，距离，角度
        :param landmarkTheoreticalSize:LandMark边长
        :return:landMarkX, landMarkY, distanceFromCameraToLandmark, yawAngle
        """
        currentCamera = "CameraTop"
        memoryProxy = ALProxy("ALMemory", self.robotIp, self.port)
        landmarkProxy = ALProxy("ALLandMarkDetection", self.robotIp, self.port)
        landmarkProxy.subscribe("landmark")
        markData = memoryProxy.getData("LandmarkDetected")
        while markData is None or len(markData) == 0:
            markData = memoryProxy.getData("LandmarkDetected")
            print("No mark")
        wzCamera = markData[1][0][0][1]
        wyCamera = markData[1][0][0][2]
        angularSize = markData[1][0][0][3]
        distanceFromCameraToLandmark = landmarkTheoreticalSize / (2 * math.tan(angularSize / 2))
        motionProxy = ALProxy("ALMotion", IP, 9559)
        transform = motionProxy.getTransform(currentCamera, 2, True)
        transformList = almath.vectorFloat(transform)
        robotToCamera = almath.Transform(transformList)
        cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)
        cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)
        robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform
        landMarkX = robotToLandmark.r1_c4
        landMarkY = robotToLandmark.r2_c4
        landmarkProxy.unsubscribe("landmark")
        yawAngle = math.atan2(landMarkY, landMarkX)
        return [landMarkX, landMarkY, distanceFromCameraToLandmark, yawAngle]


if __name__ == '__main__':
    for ii in range(100):
        s_time = time.time()
        YellowStick = StickDetection("192.168.43.165")
        stickInfo = YellowStick.compute_stickPosition1()
        if len(stickInfo) is not 0:
            stickImg = stickInfo[2]
            cv2.imshow("YellowStick", stickImg)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            print(stickInfo[0:2])
        else:
            print("no stick")
        print(time.time() - s_time)
