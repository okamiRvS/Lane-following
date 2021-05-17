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

        # pitch camera: 0.076417
        # rosmsg show sensor_msgs/Image
        # rostopic echo -n1 /thymio10/camera/image_raw/height
        # rostopic echo -n1 /thymio10/camera/image_raw/width
        # rostopic echo -n1 /thymio10/camera/image_raw/encoding
        # rostopic echo -n1 /thymio10/camera/image_raw/data

        self.image_sub = rospy.Subscriber("/thymio10/camera/image_raw", Image, self.camera_callback)
        self.cv_image = None

    def camera_callback(self, data):
        try:
            # bgr8 cause its the default OpenCV encoding
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        except CvBridgeError as e:
            print(e)
        

    def run(self):
        while not rospy.is_shutdown():
            self.sleep()
            if self.cv_image is not None:
                break

        while not rospy.is_shutdown():           
            try:
                # crop the image to make process much faster
                height, width, channels = self.cv_image.shape
                cropxSize = 100
                crop_img = self.cv_image[height-cropxSize::,:]

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

                kernel = np.ones((5, 5), np.uint8)
                blur = cv2.blur(mask, (5,5))
                erosion = cv2.erode(blur, kernel, iterations=1)
                #dilation = cv2.dilate(erosion, kernel, iterations=1)

                res = cv2.bitwise_and(crop_img, crop_img, mask=erosion)

                # find contours
                contours, hierarchy = cv2.findContours(erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if len(contours) == 0:
                    print("I can't see anything, help me") #do the same action 
                elif len(contours) == 1:
                    # draw contours
                    cv2.drawContours(res, contours[0], -1, (0, 255, 0), 3)

                    points = np.zeros((len(contours[0]), 2), dtype=np.uint16)
                    for idx, point in enumerate(contours[0]):
                        points[idx] = point[0]

                    # point with higher y coordinate
                    orientationPoint = points[np.argsort(points[:,1])[0]]

                    #draw the orientationPoint
                    cv2.circle(res, (orientationPoint[0], orientationPoint[1]), 5, (0,200,255), -1)

                    # calculate centroid
                    m = cv2.moments(contours[0], False)
                    try:
                        cx1, cy1 = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
                    except ZeroDivisionError: #if we don't detect anything
                        cx1, cy1 = (int(cropxSize/2), int(width/2)) #center of the image

                    # draw the centroid of the line
                    cv2.circle(res, (cx1, cy1), 5, (0,0,255), -1)

                    # if we are at the end of the curve boost a little bit
                    # let's see the amount of pixel of the curve where they are
                    boost = 1
                    if len(points) == len(points[points[:,0] < int(width/2)]):
                        boost = 2

                    if cx1 - orientationPoint[0] < 0:
                        print("turn right")
                        self.vel_msg.linear.x = .12
                        self.vel_msg.angular.z = -.2 * boost
                    elif cx1 - orientationPoint[0] >= 0:
                        print("turn left")
                        self.vel_msg.linear.x = .12
                        self.vel_msg.angular.z = .2 * boost

                elif len(contours) > 1:

                    # draw contours left
                    cv2.drawContours(res, contours[0], -1, (0, 255, 0), 3)

                    # calculate centroid from left
                    m = cv2.moments(contours[0], False)
                    try:
                        cx1, cy1 = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
                    except ZeroDivisionError: #if we don't detect anything
                        cx1, cy1 = (int(cropxSize/2), int(width/2)) #center of the image

                    # draw the centroid of the left line
                    cv2.circle(res, (cx1, cy1), 5, (0,0,255), -1)

                    # draw contours right
                    cv2.drawContours(res, contours[1], -1, (0, 255, 0), 3)

                    # calculate centroid from right
                    m = cv2.moments(contours[1], False)
                    try:
                        cx2, cy2 = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
                    except ZeroDivisionError: #if we don't detect anything
                        cx2, cy2 = (int(cropxSize/2), int(width/2)) #center of the image

                    # draw the centroid of the right line
                    cv2.circle(res, (cx2, cy2), 5, (0,0,255), -1)

                    # draw the center of the lines
                    centerx, centery = int((cx1+cx2)/2), int((cy1+cy2)/2)
                    cv2.circle(res, (centerx, centery), 10, (0,0,255), -1)


                    if int(width/2) - centerx > 10:
                        print("turn left")
                        self.vel_msg.linear.x = .15
                        self.vel_msg.angular.z = .1
                    elif int(width/2) - centerx < -10:
                        print("turn right")
                        self.vel_msg.linear.x = .15
                        self.vel_msg.angular.z = -.1
                    else:
                        print("I'm in the center")
                        self.vel_msg.linear.x = .2
                        self.vel_msg.angular.z = 0

                self.velocity_publisher.publish(self.vel_msg)

            except CvBridgeError as e:
                print(e)

            '''
            cv2.imshow("Blur", blur)
            cv2.imshow("Erosion", erosion)
            #cv2.imshow("Dilation", dilation)
            cv2.imshow("Original", self.cv_image)
            cv2.imshow("HSV", hsv)
            cv2.imshow("Mask", mask)
            '''
            cv2.imshow("Res", res)
            cv2.waitKey(1) # you must put in pause gazebo
        
            self.sleep()

        self.stop()


if __name__ == '__main__':
    try:
        controller = Task1()
        controller.run()
    except rospy.ROSInterruptException as e:
        pass

    cv2.destroyAllWindows()
