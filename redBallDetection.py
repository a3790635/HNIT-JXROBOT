# coding=utf-8
from TargetFeature import HogFeature, ColorFeature
from Classifier import KNN

import numpy as np
import cv2
import os

class RedBallDetection(object):
    def __init__(self, img):
        self.img = img

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

    def HoughDetection(self, isShow=False):
        binImg = self.preProcess(self.img)
        circles = cv2.HoughCircles(binImg, cv2.HOUGH_GRADIENT, 1, 100,
                                   param1=150, param2=15, minRadius=2, maxRadius=60)
        if circles is None:
            circles = []
            print("no circle")
        else:
            circles = circles[0,]
            if isShow is True:
                self.showHoughResult(self.img, circles)
        return circles

    def showHoughResult(self, img, circles):  # 显示检测出来的圆的轮廓，默认不显示
        for circle in circles:
            rect = self.circle2Rect(circle)
            initX, initY = rect[0], rect[1]
            endX, endY = rect[2], rect[3]
            cv2.rectangle(img, (initX, initY), (endX, endY), (0, 0, 255), 2)  # 画矩形

            x, y, r = int(circle[0]), int(circle[1]), int(circle[2])
            cv2.circle(img, (x, y), r, (0, 0, 255), 2)  # 画圆

        cv2.imshow("Hough Result", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def preProcess(self, img):
        HSVImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        smin1, vmin1, hmax1, hmin2 = 9, 21, 39, 153

        minHSV1 = np.array([0, smin1, vmin1])
        maxHSV1 = np.array([hmax1, 255, 255])
        minHSV2 = np.array([hmin2, smin1, vmin1])
        maxHSV2 = np.array([180, 255, 255])

        # 二值化处理
        binImg1 = cv2.inRange(HSVImg, minHSV1, maxHSV1)
        binImg2 = cv2.inRange(HSVImg, minHSV2, maxHSV2)
        binImg = np.maximum(binImg1, binImg2)

        # 图像滤波处理（腐蚀，膨胀，高斯）
        binImg = self.filter(binImg)
        return binImg

    def filter(self, img):
        kernelErosion = np.ones((3, 3), np.uint8)
        kernelDilation = np.ones((3, 3), np.uint8)
        resImg = cv2.erode(img, kernelErosion, iterations=2)
        resImg = cv2.dilate(resImg, kernelDilation, iterations=3)
        resImg = cv2.GaussianBlur(resImg, (9, 9), 1.5)

        return resImg

    def reshapeBallRect(self, rawRect, numbers):
        newRect = np.zeros((numbers, 4))
        initX, initY, endX, endY = rawRect[0], rawRect[1], rawRect[2], rawRect[3]  # 初始化参数

        newRect[0] = [initX, initY, initX + (endX - initX) / 2, initY + (endY - initY) / 2]
        newRect[1] = [initX + (endX - initX) / 2, initY, endX, initY + (endY - initY) / 2]
        newRect[2] = [initX, initY + (endY - initY) / 2, initX + (endX - initX) / 2, endY]
        newRect[3] = [initX + (endX - initX) / 2, initY + (endY - initY) / 2, endX, endY]

        return newRect

    def circle2Rect(self, circle, k=1):
        centerX, centerY, radius = circle[0], circle[1], circle[2]
        initX, initY = int(centerX - int(k * radius)), int(centerY - int(k * radius))
        endX, endY = int(centerX + int(k * radius)), int(centerY + int(k * radius))

        return [initX, initY, endX, endY]

    def calColorFeature(self, img, number=16):
        color = ColorFeature(img, number)
        result = color.colorExtract(img)

        return np.round(result, 4)

    def calHOGFeature(self, img, cell_size):
        rectBallArea = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hog = HogFeature(rectBallArea, cell_size)
        vector, image = hog.hog_extract()
        return np.round(vector[0], 4)

    def result(self):
        Rects = []
        knn = KNN("data_ball.txt")
        srcImg = self.img
        rects = self.HoughDetection()
        if rects == []:
            print("no rects")
            return Rects

        for rect in rects:  # 检测每个轮廓
            resultTotal = []
            x , y, r = rect[:3]
            rect_ = [x, y, r]
            rect = self.circle2Rect(rect)
            if rect[0] < 0 or rect[1] < 0 or rect[2] > 640 or rect[3] > 480:
                print("out of bound")
                continue
            newRects = self.reshapeBallRect(rect, 4)

            for newRect in newRects:
                newInitX, newInitY = int(newRect[0]), int(newRect[1])
                newEndX, newEndY = int(newRect[2]), int(newRect[3])
                rectBallArea = srcImg[newInitY:newEndY, newInitX:newEndX, :]

                resultColor = self.calColorFeature(rectBallArea, 16)
                cellSize = min(newEndX - newInitX, newEndY - newInitY)
                resultHOG = self.calHOGFeature(rectBallArea, cellSize / 2)
                resultTotal.extend(resultColor)
                resultTotal.extend(resultHOG)

            resultTotal = np.array(resultTotal).reshape(1, -1).astype('float64')
            classify = knn.classifyVector(resultTotal)

            if classify == 1:
                classifyResult = "yes"
                Rects.append(rect_)
                cv2.rectangle(srcImg, (rect[0], rect[1]), (rect[2], rect[3]), (0, 0, 255), 2)

            else:
                classifyResult = "no"
                cv2.rectangle(srcImg, (rect[0], rect[1]), (rect[2], rect[3]), (0, 255, 0), 2)

            print('classify', classifyResult)
        cv2.imshow("test " + str(i), srcImg)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return Rects

if __name__ == '__main__':
    imgPath = "./img_test/"
    numbers = len(os.listdir(imgPath))
    cnt = 0
    for i in xrange(0, numbers):
        image = cv2.imread("img_test/{}.jpg".format(i))
        redball = RedBallDetection(image)
        print(redball.result())