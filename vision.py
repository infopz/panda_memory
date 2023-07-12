import random

import cv2
import numpy as np
#import torch
import sys
import rospy
import message_filters
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

image = None

def start_camera_node():

    print("Starting Node Vision")
    print("Subscribing to camera images")
    subscriber = message_filters.Subscriber("/camera/image_raw", Image)
    syncro = message_filters.TimeSynchronizer([subscriber], 1)
    syncro.registerCallback(save_image)

    #image = rospy.wait_for_message("/camera/image_raw", Image)
    #cv2_image = CvBridge().imgmsg_to_cv2(image, "bgr8")
    #image = cv2_image
    #cv2.imwrite("img"+str(random.randint(0,100000))+".jpg", image)
    #return image
    rospy.spin()


def save_image(camera_image):
    global image

    cv2_image = CvBridge().imgmsg_to_cv2(camera_image, "bgr8")
    image = cv2_image
    #cv2.imwrite("img.jpg", image)


def extract_sift(image):
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    sift = cv2.xfeatures2d.SIFT_create()
    sift_keypoints, sift_descriptor = sift.detectAndCompute(image_gray, None)

    return sift_descriptor


def get_descriptor():
    global image

    desc = extract_sift(image)
    cv2.imwrite("imgs/img"+str(random.randint(0,100000))+".jpg", image)

    return desc


def sift_compare(des1, des2):

    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)
    threshold = 0.75
    good_matches = []
    for m, n in matches:
        if m.distance < threshold * n.distance:
            good_matches.append(m)

    return len(good_matches)>100
