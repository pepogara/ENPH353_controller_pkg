#! /usr/bin/env python3

import numpy as np
import cv2 as cv
import rospy


def HSV(img):
    """!
    @brief      Function to apply mask to image to isolate hints

    @param      img - image to be masked

    @return     mask - masked image
    """

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
    """!
    @brief      Function to apply perspective transform to image to isolate hint

    @param      hsv - masked image used to get the transform 
    @param      img - original image that transform will be applied to

    @return     transformed_img - full color image with perspective transform applied
                None - if no hint is found, or if the hint is too small
    """

    # Find contours in the HSV image
    # RETR_CCOMP retrieves all contours and organizes them into a two level heirarchy
    # CHAIN_APPROX_SIMPLE stores only the corner points of the contour
    contours, hierarchy = cv.findContours(hsv, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)

    # Finds all inner contours (holes)
    inner_contours = [contours[i] for i in range(len(contours)) if hierarchy[0][i][3] >= 0]
    
    try:
        # Find the largest inner contour (the white rectangular frame, others are noise) if it exists
        largest_contour = max(inner_contours, key=cv.contourArea)
    except:
        return None, 0
    
    # ignores contours that are too small
    contourArea = cv.contourArea(largest_contour)
    if contourArea < 11000:
        return None, 0

    # Approximate the largest contour with a polygon
    epsilon = 0.1 * cv.arcLength(largest_contour, True)
    approx_polygon = cv.approxPolyDP(largest_contour, epsilon, True)

    # Extract the corner points from the approximated polygon
    corner_points = [point[0] for point in approx_polygon]

    """Untested code for if corner_points is not 4"""
    startTime = rospy.get_time()
    while len(corner_points) != 4:
        if rospy.get_time() - startTime > 0.1:
            return None, 0
        epsilon = (0.1 + .01*(len(corner_points)- 4)) * cv.arcLength(largest_contour, True)
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

    return brighten(transformed_img), contourArea
    # return cv.drawContours(img, [largest_contour], -1, (0, 255, 0), 1)
    # for point in corner_points:
    #     cv.circle(img, point, 5, (0, 0, 255), -1)
    # return img

def brighten(img, value=255):
    """!
    @brief      Function to brighten image
    
    Code from: https://stackoverflow.com/questions/32609098/how-to-fast-change-image-brightness-with-python-opencv

    @param      img - image to be brightened
    @param      value - value to brighten image by, default is 255 (max brightness)

    @return     img - brightened image
    """

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv.merge((h, s, v))
    img = cv.cvtColor(final_hsv, cv.COLOR_HSV2BGR)
    return img 

def character_split(img, clue=True):
    """!
    @brief      Function to split image into characters

    @param      img - image to be split
    @param      clue - boolean to indicate if image is a clue or not
                True for clues, False for type (top of image)
    
    @return     characters - list of images with each character
    """

    img = cv.resize(img, (400, 600))

    img = np.expand_dims(img, axis=0)  # Expand dimension to match neural network input shape

    y_range = (250, 340) if clue else (30, 120)
    start_x = 30 if clue else 250
    end_x = 75 if clue else 295
    increment = 45

    num_chars = 11 if clue else 5

    characters = []

    for i in range(num_chars):
        char = img[:, y_range[0]:y_range[1], start_x:end_x]  # Adjust slicing to match expanded dimension
        characters.append(char)
        start_x += increment
        end_x += increment


    return characters

def onehotToStr(one_hot_vector):
    """!
    @brief Convert a one-hot encoded vector to a string.

    This function takes a one-hot encoded vector and converts it into a string
    representation based on the characters defined in the "characters" string.

    @param one_hot_vector: One-hot encoded vector to convert to a string.

    @return: A string representation of the one-hot encoded vector.
    """
    characters = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ "
    decoded_strings = []

    index = np.argmax(one_hot_vector)
    character = characters[index]
    decoded_strings.append(character)

    # Join the characters to form the final string
    return ''.join(decoded_strings)

def onehotToIndex(one_hot_vector):
    """!
    @brief Convert a one-hot encoded vector to its index.

    This function takes a one-hot encoded vector and returns the index of
    the maximum value in the vector, indicating which character is encoded.

    @param one_hot_vector: One-hot encoded vector to convert to an index.

    @return: The index of the maximum value in the one-hot vector.
    """
    return np.argmax(one_hot_vector)

def predict_word(nn, img, clue=True):
    """!
    @brief      Function to predict the word from an image

    @param      nn - neural network to use for prediction
    @param      img - image to be predicted

    @return     word - predicted word
    """
    
    characters = character_split(img, clue)
    decoded_chars = []

    for char in characters:
        prediction = nn.predict(char)
        single_dig = onehotToStr(prediction)
        decoded_chars.append(single_dig)

    return ''.join(decoded_chars)