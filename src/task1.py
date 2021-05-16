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
            crop_img = cv_image[height-200::,:]

            pdb.set_trace()

            # convert RGB blue to BGR blue
            RGB_blue = [58,99,173]
            BGR_blue = np.uint8([[[RGB_blue[2], RGB_blue[1], RGB_blue[0]]]])

            # get HSV blue and a range 
            HSV_blue = cv2.cvtColor(BGR_blue, cv2.COLOR_BGR2HSV)[0][0]
            lower_blue = np.array([HSV_blue[0]-15, HSV_blue[1]-15, HSV_blue[2]-15])
            upper_blue = np.array([HSV_blue[0]+15, HSV_blue[1]+15, HSV_blue[2]+15])

            # convert from RGB to HSV
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            # apply a mask
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

            # calculate centroid
            m = cv2.moments(mask, False)
            try:
                cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
            except ZeroDivisionError: #if we don't detect anything
                cx, cy = (100, width/2) #center of the image

            # draw the centroid
            cv2.circle(res, (int(cx), int(cy)), 10, (0,0,255), -1)


        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("Mask", mask)
        cv2.imshow("Res", res)
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
