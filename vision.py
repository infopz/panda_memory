import random
import cv2
import numpy as np
import rospy
import message_filters
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import vgg

image = None


def start_camera_node():

    print("Camera Started")
    subscriber = message_filters.Subscriber("/camera/image_raw", Image)
    syncro = message_filters.TimeSynchronizer([subscriber], 1)
    syncro.registerCallback(save_image)

    #image = rospy.wait_for_message("/camera/image_raw", Image)

    rospy.spin()


def save_image(camera_image):
    global image

    cv2_image = CvBridge().imgmsg_to_cv2(camera_image, "bgr8")
    image = cv2_image


def sift_descriptor(img):
    sift_des = cv2.convertScaleAbs(img, 0.5, 2.5)
    sift_des = cv2.rotate(sift_des, cv2.ROTATE_90_CLOCKWISE)
    sift_des = cv2.cvtColor(sift_des, cv2.COLOR_BGR2GRAY)
    sift_des = sift_des[105:230, 65:190]
    sift = cv2.xfeatures2d.SIFT_create()
    sift_key, sift_des = sift.detectAndCompute(sift_des, None)

    return sift_des


def sift_compare(des1, des2):

    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)
    threshold = 0.75
    good_matches = []
    for m, n in matches:
        if m.distance < threshold * n.distance:
            good_matches.append(m)

    return len(good_matches) > 25


def tm_descriptor(img):
    # The descriptor is the entire image
    tm_des = cv2.convertScaleAbs(img, 0.5, 2.5)
    tm_des = cv2.rotate(tm_des, cv2.ROTATE_90_CLOCKWISE)
    tm_des = cv2.cvtColor(tm_des, cv2.COLOR_BGR2GRAY)

    return tm_des


def tm_compare(des1, des2):
    template = des2[105:230, 65:190]
    result = cv2.matchTemplate(des1, template, cv2.TM_CCOEFF_NORMED)
    threshold = 0.9
    loc = np.where(result >= threshold)
    return len(loc[0]) > 0


def get_descriptor(method):
    global image

    time.sleep(0.5)

    if method == "SIFT":
        desc = sift_descriptor(image)
    elif method == "TM":
        desc = tm_descriptor(image)
    elif method == "NN":
        desc = vgg.vgg_descriptor(image)

    # TODO: da rimuovere
    cv2.imwrite("imgs/img"+str(random.randint(0,100000))+".jpg", image)

    return desc


def get_compare_func(method):
    if method == "SIFT":
        return sift_compare
    elif method == "TM":
        return tm_compare
    elif method == "NN":
        return vgg.vgg_compare
