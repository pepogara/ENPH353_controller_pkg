#! /usr/bin/env python3

import numpy as np
import cv2 as cv
import time

def HSV(img):


    img = cv.medianBlur(img,5)

    # Convert BGR to HSV
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    uh = 130
    us = 255
    uv = 103 # originally 255
    lh = 118 # originally 110
    ls = 50 # originally 50 then 195
    lv = 75 # originally 50
    lower_hsv = np.array([lh,ls,lv])
    upper_hsv = np.array([uh,us,uv])

    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_hsv, upper_hsv)
    return mask

    """Code for calibrating HSV values"""
    # window_name = "HSV Calibrator"
    # cv.namedWindow(window_name)

    # def nothing(x):
    #     print("Trackbar value: " + str(x))
    #     pass

    # # create trackbars for Upper HSV
    # cv.createTrackbar('UpperH',window_name,0,255,nothing)
    # cv.setTrackbarPos('UpperH',window_name, uh)

    # cv.createTrackbar('UpperS',window_name,0,255,nothing)
    # cv.setTrackbarPos('UpperS',window_name, us)

    # cv.createTrackbar('UpperV',window_name,0,255,nothing)
    # cv.setTrackbarPos('UpperV',window_name, uv)

    # # create trackbars for Lower HSV
    # cv.createTrackbar('LowerH',window_name,0,255,nothing)
    # cv.setTrackbarPos('LowerH',window_name, lh)

    # cv.createTrackbar('LowerS',window_name,0,255,nothing)
    # cv.setTrackbarPos('LowerS',window_name, ls)

    # cv.createTrackbar('LowerV',window_name,0,255,nothing)
    # cv.setTrackbarPos('LowerV',window_name, lv)

    # font = cv.FONT_HERSHEY_SIMPLEX

    # print("Loaded images")

    # while(1):
    #     # Threshold the HSV image to get only blue colors
    #     mask = cv.inRange(hsv, lower_hsv, upper_hsv)
    #     cv.putText(mask,'Lower HSV: [' + str(lh) +',' + str(ls) + ',' + str(lv) + ']', (10,30), font, 0.5, (200,255,155), 1, cv.LINE_AA)
    #     cv.putText(mask,'Upper HSV: [' + str(uh) +',' + str(us) + ',' + str(uv) + ']', (10,60), font, 0.5, (200,255,155), 1, cv.LINE_AA)
    #     cv.imshow(window_name,mask)

    #     k = cv.waitKey(1) & 0xFF
    #     if k == 27:
    #         pass
    #         break
    #     # get current positions of Upper HSV trackbars
    #     uh = cv.getTrackbarPos('UpperH',window_name)
    #     us = cv.getTrackbarPos('UpperS',window_name)
    #     uv = cv.getTrackbarPos('UpperV',window_name)
    #     upper_blue = np.array([uh,us,uv])
    #     # get current positions of Lower HSCV trackbars
    #     lh = cv.getTrackbarPos('LowerH',window_name)
    #     ls = cv.getTrackbarPos('LowerS',window_name)
    #     lv = cv.getTrackbarPos('LowerV',window_name)
    #     upper_hsv = np.array([uh,us,uv])
    #     lower_hsv = np.array([lh,ls,lv])

    #     time.sleep(.1)

    # cv.destroyAllWindows()
    
def homography(hsv, img):

    # Find contours in the HSV image
    # RETR_CCOMP retrieves all contours and organizes them into a two level heirarchy
    # CHAIN_APPROX_SIMPLE stores only the corner points of the contour
    contours, heirarchy = cv.findContours(hsv, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)

    # Finds all inner contours (holes)
    inner_contours = [contours[i] for i in range(len(contours)) if heirarchy[0][i][3] >= 0]
    
    try:
        # Find the largest inner contour (the white rectangular frame, others are noise) if it exists
        largest_contour = max(inner_contours, key=cv.contourArea)
    except:
        return None
    
    # ignores contours that are too small
    if cv.contourArea(largest_contour) < 2000:
        return None

    # Approximate the largest contour with a polygon
    epsilon = 0.1 * cv.arcLength(largest_contour, True)
    approx_polygon = cv.approxPolyDP(largest_contour, epsilon, True)

    # Extract the corner points from the approximated polygon
    corner_points = [point[0] for point in approx_polygon]

    """Untested code for if corner_points is not 4"""
    if len(corner_points) != 4:
        epsilon = (0.1 + .02*(len(corner_points)- 4)) * cv.arcLength(largest_contour, True)
        approx_polygon = cv.approxPolyDP(largest_contour, epsilon, True)

        # Extract the corner points from the approximated polygon
        corner_points = [point[0] for point in approx_polygon]
    

    # Sort the corner points in the order top left, bottom left, bottom right, top right (sorted sorts ascending)
    sorted_corner_points = sorted(corner_points, key=lambda point: point[0])
    left = sorted(sorted_corner_points[:2], key=lambda point: point[1])
    right = sorted(sorted_corner_points[2:], key=lambda point: point[1], reverse=True)
    sorted_corner_points = left + right
    # print(sorted_corner_points)

    # Create the source and destination points for perspective transform
    src_pts = np.float32([[point[0], point[1]] for point in sorted_corner_points])
    dst_pts = np.float32([[0, 0], [0, hsv.shape[0]], [hsv.shape[1], hsv.shape[0]], [hsv.shape[1], 0]])
    # Compute the perspective transform matrix
    matrix = cv.getPerspectiveTransform(src_pts, dst_pts)


    # Apply the same perspective transform to the img
    transformed_img = cv.warpPerspective(img, matrix, (img.shape[1], img.shape[0]))

    return transformed_img
    # return cv.drawContours(img, [largest_contour], -1, (0, 255, 0), 1)
    # for point in corner_points:
    #     cv.circle(img, point, 5, (0, 0, 255), -1)
    # return img
