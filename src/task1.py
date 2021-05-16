#!/usr/bin/env python
import rospy
from thymio import ThymioController, PID
from sensor_msgs.msg import Range, Image
import pdb

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError # this is necessary to work with images in ROS

class Task1(ThymioController):

    def __init__(self):
        super(Task1, self).__init__()

        self.bridge_object = CvBridge()

        # rosmsg show sensor_msgs/Image
        # rostopic echo -n1 /thymio10/camera/image_raw/height
        # rostopic echo -n1 /thymio10/camera/image_raw/width
        # rostopic echo -n1 /thymio10/camera/image_raw/encoding
        # rostopic echo -n1 /thymio10/camera/image_raw/data

        self.image_sub = rospy.Subscriber("/thymio10/camera/image_raw", Image, self.camera_callback)

    def camera_callback(self, data):
        try:
            # bgr8 cause its the default OpenCV encoding
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            # crop the image to make process much faster
            height, width, channels = cv_image.shape
            crop_img = cv_image[height-200::,:,:]

            # convert from RGB to HSV
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            # get blue lines
            

            pdb.set_trace()
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("Image window", crop_img)
        cv2.waitKey(0) # you must put in pause gazebo

    def run(self):
        pass


if __name__ == '__main__':
    try:
        controller = Task1()
        controller.run()
    except rospy.ROSInterruptException as e:
        pass

    cv2.destroyAllWindows()
