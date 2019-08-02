# coding=utf-8
import cv2
import numpy as np
import time


class StickDetection(object):
    def __init__(self, img):
        """
        :param img: img path
        """
        self.img = img
        self.low_range = np.array([27, 55, 115])
        self.high_range = np.array([40, 255, 255])

    def bilateralFilter(self, img):
        bilateral = cv2.bilateralFilter(img, 25, 12.5, 50)
        # cv2.imshow("src", img)
        # cv2.imshow("bilateral", bilateral)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return bilateral

    def __brightEnhance(self, img, alpha=1.5, beta=1.0):
        """
        亮度增强
        :param img: img array
        :param alpha: multi
        :param beta: Bias
        :return:
        """
        img1 = img.copy()
        # img1 = img1 * alpha + beta
        img1 = np.where(img1 <= 50, img1, img1 * alpha + beta)
        img1 = np.where(img1 <= 255, img1, 255 * 2 - img1)
        img1 = np.array(img1, dtype=np.uint8)
        return img1

    def __compute_score(self, hsv, x, y, w, h):
        """
        :param x: box minx
        :param y: box miny
        :param hsv: hsv array
        :param w: box width
        :param h: box height
        :return: score
        """
        score = 0
        for j in range(x, x + w):
            for i in range(y, y + h):
                if i < len(hsv[0]) and j < len(hsv):
                    if self.low_range[0] < hsv[i][j][0] < self.high_range[0] and self.low_range[1] < hsv[i][j][1] < \
                            self.high_range[1] and self.low_range[2] < hsv[i][j][2] < self.high_range[2]:
                        score += 1
        return h
        # int(100 * (float(score) / (w * h)))
        # score
        # w * h
        # h

    def nms(self, score_list):
        """
        nms非极大值抑制
        :param score_list: score list
        :return: x, y, w, h
        """
        score_list.sort(
            cmp=lambda score1, score2: score2[4] - score1[4])
        # score2[4] - score1[4]
        # int(100 * (abs(score1[2] / score1[3] - 0.1) - abs(score2[2] / score2[3] - 0.1)))
        return score_list[0][0], score_list[0][1], score_list[0][2], score_list[0][3]

    def stick_detection(self):
        """
        stick detection
        :return: stick center
        """
        self.img = cv2.imread(self.img, 1)
        if len(self.img) != 640:
            self.img = cv2.resize(self.img, (640, 480))
        self.img = self.bilateralFilter(self.img)
        # self.img = self.__brightEnhance(self.img)
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        kernelDilation = np.ones((5, 5), dtype=np.uint8)

        th = cv2.inRange(hsv, self.low_range, self.high_range)

        # dilated = cv2.dilate(th, kernelDilation, iterations=1)
        # gaus = cv2.GaussianBlur(th, (9, 9), 0)

        _, contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # draw_img = cv2.drawContours(hsv.copy(), contours, 2, (0, 255, 0), 1)
        num = len(contours)
        # print("num: ", num)
        # nms非极大值抑制，score为hsv黄色像素
        score_list = list()
        for i in range(num):
            x, y, w, h = cv2.boundingRect(contours[i])
            score = self.__compute_score(hsv, x, y, w, h)
            score_list.append([x, y, w, h, score])
            # cv2.circle(self.img, (x + w / 2, y + h / 2), 4, (255, 0, 0), -1)
            # cv2.rectangle(self.img, (x, y), (x + w, y + h), (0, 0, 255), 2)

        if len(score_list) > 1:
            x, y, w, h = self.nms(score_list)
            center = (x + w / 2, y + h / 2)
            cv2.circle(self.img, center, 4, (255, 0, 0), -1)
            cv2.rectangle(self.img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # cv2.namedWindow("result", False)
            # cv2.imshow("result", self.img)
            # cv2.moveWindow("result", 0, 0)
            # cv2.waitKey()
            # cv2.imwrite("img/stick1_ed.jpg", self.img)
            # cv2.destroyAllWindows()
            return center
        else:
            return 0, 0

    def save_img(self, path):
        cv2.imwrite(path, self.img)


if __name__ == '__main__':
    stime = time.time()
    for z in range(1, 341):
        imgPath = "stickImg/stick_{}.jpg".format(z)
        stick = StickDetection(imgPath)

        stick_center = stick.stick_detection()
        stick.save_img("detectedImg/{}".format(imgPath.split('/')[-1]))

        if stick_center != (0, 0):
            print("times: {}  center: {}".format(z, stick_center))
        else:
            print "Not Found stick"
    etime = time.time()
    print("Time spent: {}s".format(etime - stime))
