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

lineDevideC = 50

isD = False

# 1: moving on the edge
# 2: moving to node
# 3: aimming at node
# 4: leaving from node
mode = 4
outDir = [1, 0]
aimCount = 0

path = 'pics/'


def getiDevided(i, twoP):
    x = twoP[0][0] * i + twoP[1][0] * (1 - i)
    y = twoP[0][1] * i + twoP[1][1] * (1 - i)
    return [int(x), int(y)]


def exractTopoStructor(img2v, x0, y0):
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

    return ringDir


if __name__ == "__main__":
    pics = os.listdir(path)
    # random.shuffle(pics)

    if isD:
        pics = ['270.png']

    for pic in pics:

        # image processing
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
        kd = np.ones((5, 5))
        img2v = cv2.dilate(img2v, kd)

        if isD:
            cv2.imshow('im', img2v)
            cv2.waitKey()
        else:
            cv2.imwrite('after/' + pic, img2v)

        imgLine = cv2.Canny(img2v, 5, 200, 3)
        if isD:
            cv2.imshow('im', imgLine)
            cv2.waitKey()
        else:
            cv2.imwrite('line/' + pic, imgLine)

        lines = cv2.HoughLines(imgLine, 1, np.pi / 180, 50)

        if lines is not None:
            lineInfos = {}
            lines1 = lines[:, 0, :]
            for pair in lines1[:]:

                rho = pair[0]
                theta = pair[1]

                # a = np.cos(theta)
                # b = np.sin(theta)
                #
                # x0 = a * rho
                # y0 = b * rho
                # x1 = int(x0 + 1000 * (-b))
                # y1 = int(y0 + 1000 * a)
                # x2 = int(x0 - 1000 * (-b))
                # y2 = int(y0 - 1000 * a)
                # cv2.line(imgori, (x1, y1), (x2, y2), (0, 255, 0), 2)

                dirHasRecorded = False
                for angle in lineInfos.keys():

                    if abs(abs(angle - theta) - np.pi) < 0.3:
                        theta = angle
                        rho = -rho

                    if abs(angle - theta) < 0.3:
                        lineInfos[angle][0].append(theta)
                        lineInfos[angle][1].append(rho)
                        dirHasRecorded = True
                        break

                if dirHasRecorded:
                    continue

                lineInfos[theta] = [[theta], [rho]]

            linesExracted = []
            for lineSet in lineInfos.values():
                th = np.average(lineSet[0])
                rh = np.average(lineSet[1])
                linesExracted.append([th, rh])

            if isD:
                cv2.imshow('im', img2v)
                cv2.waitKey()

                cv2.imshow('im', imgori)
                cv2.waitKey()

            # one line situation
            if len(linesExracted) == 1:

                a1, r1 = linesExracted[0]
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

                xb, yb = getiDevided(0.25, edgePoints)
                xe, ye = getiDevided(0.75, edgePoints)
                bIsLine = img2v[yb][xb] == 255
                eIsLine = img2v[ye][xe] == 255

                if bIsLine and eIsLine:
                    midx = (edgePoints[0][0] + edgePoints[1][0]) / 2
                    midy = (edgePoints[0][1] + edgePoints[1][1]) / 2

                    cv2.circle(imgori, (midx, midy), 10, (0, 0, 255), 5)

                    cv2.imwrite('1/' + pic, imgori)

                else:
                    if bIsLine:
                        beginType = 255
                    else:
                        beginType = 0
                    for i in range(lineDevideC / 4, lineDevideC * 3 / 4):
                        x, y = getiDevided(float(i) / lineDevideC, edgePoints)
                        if img2v[y][x] != beginType:
                            break
                    cv2.circle(imgori, (x, y), 10, (0, 0, 255), 5)

                    if bIsLine:
                        cv2.line(imgori, (x, y), (xb, yb), (255, 0, 0), 5)
                    else:
                        cv2.line(imgori, (x, y), (xe, ye), (255, 0, 0), 5)

                    cv2.imwrite('1/' + pic, imgori)

            # two lines situation
            elif len(linesExracted) == 2:
                a1, r1 = linesExracted[0]
                a2, r2 = linesExracted[1]

                ans = np.matrix([[np.cos(a1), np.sin(a1)], [np.cos(a2), np.sin(a2)]]).I
                # x0 y0 is the cross point of two lines
                x0 = int(ans[0, 0] * r1 + ans[0, 1] * r2)
                y0 = int(ans[1, 0] * r1 + ans[1, 1] * r2)

                if abs(x0 - sizex / 2) <= sizex / 4 and abs(y0 - sizey / 2) <= sizey / 4:
                    rings = exractTopoStructor(img2v, x0, y0)
                    cv2.imwrite('2/' + pic, imgori)

                else:
                    cv2.circle(imgori, (x0, y0), 10, (0, 0, 255), 5)
                    cv2.imwrite('2/' + pic, imgori)

            else:
                print(linesExracted)
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
