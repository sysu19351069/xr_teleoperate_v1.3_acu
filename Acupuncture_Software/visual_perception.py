# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2

# 识别出红色标记穴位并标记中心点
def locate_acupoint(img, hsv_lower, hsv_upper, iou=None, min_contour_area=50, max_contour_area=1000):
    src = img.copy()
    if iou is not None:
        u_min = iou[0]
        u_max = iou[1]
        v_min = iou[2]
        v_max = iou[3]
        points_iou = np.array([[u_min, v_min], [u_max, v_min], [u_max, v_max], [u_min, v_max]])
        contours_iou = np.array([points_iou])
        mask_iou = np.zeros(src.shape[:2], np.uint8)
        cv2.polylines(mask_iou, contours_iou, 1, 255) #*绘制多边形
        cv2.fillPoly(mask_iou, contours_iou, 255) #*填充颜色
        src = cv2.bitwise_and(src, src, mask=mask_iou)

    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV) #*将RGB转为HSV
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper) #*为二值图
    cv2.imshow('mask', mask)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    contours_ellipse = []
    for i in range(len(contours)):
        print(cv2.contourArea(contours[i]))
        if hierarchy[0][i][3] == -1 and min_contour_area < cv2.contourArea(contours[i]) < max_contour_area:
            contours_ellipse.append(contours[i])
        # if hierarchy[0][i][3] != -1 and min_contour_area < cv2.contourArea(contours[i]) < max_contour_area:
        #     contours_ellipse.append(contours[i])
    
    centers = []
    for contour in contours_ellipse:
        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        centers.append([cx, cy])

    return contours_ellipse, centers


if __name__ == '__main__':
    # imgf_he = 'handeye.jpg'
    # img_src_he = cv2.imread(imgf_he)
    # hsv_lower = np.array([15, 80, 80])
    # hsv_upper = np.array([34, 255, 255])
    # contours_ellipse_he = locate_acupoint(img_src_he, hsv_lower, hsv_upper, None, 50, 2000)
    # img_ellipse_he = img_src_he.copy()
    # cv2.drawContours(img_ellipse_he, contours_ellipse_he, -1, (0, 0, 255), 3)
    # cv2.imshow("ellipse_handeye", img_ellipse_he)

    # imgf_gl = 'global.jpg'
    imgf_gl = '/home/ros/Acupuncture_Robot_Software/2.jpg'
    # imgf_gl = '/home/ros/Acupuncture_Robot_Software/test_img.jpg'
    img_src_gl = cv2.imread(imgf_gl)
    img_src_gl = cv2.resize(img_src_gl,dsize=(1280,720))
    # iou_acupoint = [550, 600, 550, 600]
    # hsv_lower = np.array([0, 80, 80])
    # hsv_upper = np.array([5, 255, 255])

    # hsv_lower = np.array([0, 0, 120])
    # hsv_upper = np.array([180, 255, 255])
    # iou_acupoint = [40, 731, 315, 490]

    hsv_lower = np.array([100, 43, 46])
    hsv_upper = np.array([124, 255, 255])
    iou_acupoint = [520, 840, 490, 630]
    min_contour_area = 1
    max_contour_area = 100

    # min_contour_area = 60
    # max_contour_area = 250
    contours_ellipse_gl, centers = locate_acupoint(
        img_src_gl, hsv_lower, hsv_upper, iou_acupoint, min_contour_area, max_contour_area)
    img_ellipse_gl = img_src_gl.copy()
    print('centers',centers)

    for center in centers:
        img_ellipse_gl = cv2.circle(img_ellipse_gl, center, 1, (0,255,0), 1)
    cv2.drawContours(img_ellipse_gl, contours_ellipse_gl, -1, (0, 0, 255), 3)
    cv2.imshow("ellipse_global", img_ellipse_gl)
    # cv2.imwrite("/home/ros/Acupuncture_Robot_Software/save1.png",img_ellipse_gl)
    cv2.waitKey(0)
