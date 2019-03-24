#!/usr/bin/env python


import cv2
import numpy as np
import os
import random
# import rospy
# from std_msgs.msg import Float32

searchR = 50
stepC = 100
stepSize = np.pi / stepC * 2

isD = False

mode = 3

path = 'pics/'

if __name__ == "__main__":
    pics = os.listdir(path)
    # random.shuffle(pics)

    if isD:
        pics = ['275.png', '277.png', '278.png']

    for pic in pics:
        imgori = cv2.imread(path + pic)
        sizex = imgori.shape[1]
        sizey = imgori.shape[0]

        lowbd = np.array([110, 175, 110])
        highbd = np.array([255, 255, 255])
        img2v = cv2.inRange(imgori, lowbd, highbd)

        if isD:
            cv2.imshow('im', imgori)
            cv2.waitKey()
            cv2.imshow('im', img2v)
            cv2.waitKey()
            cv2.imshow('im', imgori)
            cv2.waitKey()

        ke = np.ones((2, 2))
        img2v = cv2.erode(img2v, ke, iterations=3)
        kd = np.ones((10, 10))
        img2v = cv2.dilate(img2v, kd)

        if isD:
            cv2.imshow('im', img2v)
            cv2.waitKey()
        else:
            cv2.imwrite('after/' + pic, img2v)

        imgLine = cv2.Canny(img2v, 5, 200, 3)

        lines = cv2.HoughLines(imgLine, 1, np.pi / 180, 50)

        if lines is not None:
            lineInfos = []
            lines1 = lines[:, 0, :]
            for pair in lines1[:]:
                rho = pair[0]
                theta = pair[1]

                blHasRecorded = False
                for pairRecorded in lineInfos:

                    if abs(abs(pairRecorded[1] - pair[1]) - np.pi) < 0.3:
                        blHasRecorded = True
                        break

                    if abs(pairRecorded[1] - pair[1]) < 0.3:
                        blHasRecorded = True
                        break

                if blHasRecorded:
                    continue

                a = np.cos(theta)
                b = np.sin(theta)

                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * a)
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * a)
                cv2.line(imgori, (x1, y1), (x2, y2), (255, 0, 0), 2)

                lineInfos.append([rho, theta])

            if isD:
                cv2.imshow('im', img2v)
                cv2.waitKey()

                cv2.imshow('im', imgori)
                cv2.waitKey()

            if not isD:

                if len(lineInfos) == 1:

                    r1, a1 = lineInfos[0]
                    # jie ju
                    if a1 == 0:
                        a1 = 0.00001
                    x1 = r1 / np.cos(a1)
                    y1 = r1 / np.sin(a1)
                    x2 = x1 - sizey * np.tan(a1)
                    y2 = y1 - sizex / np.tan(a1)

                    edgePoints = []

                    if 0 < x1 <= sizex:
                        edgePoints.append([int(x1), 0])

                    if 0 < x2 <= sizex:
                        edgePoints.append([int(x2), sizey])

                    if 0 <= y1 < sizey:
                        edgePoints.append([0, int(y1)])

                    if 0 < y2 <= sizey:
                        edgePoints.append([sizex, int(y2)])

                    midx = (edgePoints[0][0] + edgePoints[1][0]) / 2
                    midy = (edgePoints[0][1] + edgePoints[1][1]) / 2

                    cv2.circle(imgori, (midx, midy), 10, (0, 0, 255), 5)

                    cv2.imwrite('1/' + pic, imgori)

                elif len(lineInfos) == 2:
                    r1, a1 = lineInfos[0]
                    r2, a2 = lineInfos[1]

                    y1 = np.sin(a1) / r1
                    x1 = np.cos(a1) / r1
                    y2 = np.sin(a2) / r2
                    x2 = np.cos(a2) / r2

                    ans = np.matrix([[x1, y1], [x2, y2]]).I
                    # x0 y0 is the cross point of two lines
                    x0 = ans[0, 0] + ans[0, 1]
                    y0 = ans[1, 0] + ans[1, 1]

                    if abs(x0 - sizex / 2) <= sizex / 4 and abs(y0 - sizey / 2) <= sizey / 4:

                        thBegin = 0
                        for i in range(0, stepC):
                            th = i * stepSize
                            rx = int(searchR * np.cos(th) + x0)
                            ry = int(searchR * np.sin(th) + y0)
                            if img2v[ry][rx] == 0:
                                thBegin = th
                                break

                        ringPairs = []
                        isInRing = False
                        for i in range(1, stepC + 1):
                            th = thBegin + i * stepSize
                            rx = int(searchR * np.cos(th) + x0)
                            ry = int(searchR * np.sin(th) + y0)
                            if isInRing:
                                if img2v[ry][rx] == 0:
                                    ringPairs[-1].append(th)
                                    isInRing = False
                            else:
                                if img2v[ry][rx] == 255:
                                    ringPairs.append([th])
                                    isInRing = True

                        if len(ringPairs[-1]) == 1:
                            ringPairs[-1].append(thBegin + 2 * np.pi)

                        if ringPairs[-1][1] < ringPairs[0][1]:
                            ringPairs[-1][1] += 2 * np.pi

                        ringDir = []
                        for ringPair in ringPairs:
                            th = (ringPair[0] + ringPair[1]) / 2
                            rx = int(searchR * np.cos(th) + x0)
                            ry = int(searchR * np.sin(th) + y0)
                            cv2.line(imgori, (x0, y0), (rx, ry), (255, 0, 0), 3)
                            ringDir.append(th)

                        cv2.imwrite('2/' + pic, imgori)

                    else:
                        cv2.circle(imgori, (x0, y0), 10, (0, 0, 255), 5)
                        cv2.imwrite('2/' + pic, imgori)


                else:
                    print(lineInfos)
                    cv2.imshow('im', img2v)
                    cv2.waitKey()
                    cv2.imshow('im', imgori)
                    cv2.waitKey()



        # cv2.imwrite('after/' + pic, imgcut)

        # cv2.imshow('im', imgcut)
        # cv2.waitKey()
        #
        # cv2.imshow('im', imgori)
        # cv2.waitKey()




