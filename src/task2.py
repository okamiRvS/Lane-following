#!/usr/bin/env python
import rospy
from thymio import ThymioController, PID
from sensor_msgs.msg import Range, Image

import pdb
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose, Twist, Vector3

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError # this is necessary to work with images in ROS

class Task2(ThymioController):

    def __init__(self):
        super(Task2, self).__init__()
        self.bridge_object = CvBridge()

        # pitch camera: 0.076417
        # rosmsg show sensor_msgs/Image
        # rostopic echo -n1 /thymio10/camera/image_raw/height
        # rostopic echo -n1 /thymio10/camera/image_raw/width
        # rostopic echo -n1 /thymio10/camera/image_raw/encoding
        # rostopic echo -n1 /thymio10/camera/image_raw/data
        print(self.name)
        self.image_sub = rospy.Subscriber(self.name +"/camera/image_raw", Image, self.camera_callback)
        self.frontsensor_sub = rospy.Subscriber(self.name + '/proximity/center', Range, self.set_proximity)
        self.cv_image = None
        self.speed = 1
        self.prox = 0.12
        self.onmarking = False
        self.last_state = None

    def camera_callback(self, data):
        try:
            # bgr8 cause its the default OpenCV encoding
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        except CvBridgeError as e:
            print(e)
        
    def set_proximity(self, data):
        self.prox = data.range

    def run(self):
        while not rospy.is_shutdown():
            self.sleep()
            if self.cv_image is not None:
                break
        self.lastspeedupdate = rospy.Time.now().to_nsec() - 10**9
        while not rospy.is_shutdown():           
            try:
                # crop the image to make process much faster
                height, width, channels = self.cv_image.shape
                cropxSize = 100
                crop_img = self.cv_image[height-cropxSize-25:height-25:,:]

                #detect lines
                RGB_green = [0,176,80]
                BGR_green = np.uint8([[[RGB_green[2], RGB_green[1], RGB_green[0]]]])
                HSV_green = cv2.cvtColor(BGR_green, cv2.COLOR_BGR2HSV)[0][0]
                lower_green = np.array([HSV_green[0]-15, HSV_green[1]-15, HSV_green[2]-15])
                upper_green = np.array([HSV_green[0]+15, HSV_green[1]+15, HSV_green[2]+15])
                RGB_red = [255,0,0]
                BGR_red = np.uint8([[[RGB_red[2], RGB_red[1], RGB_red[0]]]])
                HSV_red = cv2.cvtColor(BGR_red, cv2.COLOR_BGR2HSV)[0][0]
                lower_red = np.array([HSV_red[0]-15, HSV_red[1]-15, HSV_red[2]-15])
                upper_red = np.array([HSV_red[0]+15, HSV_red[1]+15, HSV_red[2]+15])
                crop_img_low = self.cv_image[height-10::,int(width/2)-50:int(width/2)+50]
                hsv_low = cv2.cvtColor(crop_img_low, cv2.COLOR_BGR2HSV)
                mask_green = cv2.inRange(hsv_low, lower_green, upper_green)
                mask_red = cv2.inRange(hsv_low, lower_red, upper_red)
                threshold = 100
                timenow = rospy.Time.now().to_nsec()
                if (timenow - self.lastspeedupdate > 10**9) and np.sum(mask_green) > threshold and not self.onmarking:
                    self.speed += 0.5
                    self.onmarking = True
                    self.lastspeedupdate = timenow
                    print("speed up")
                elif (timenow - self.lastspeedupdate > 10**9) and np.sum(mask_red) > threshold and not self.onmarking:
                    self.speed -= 0.5
                    self.onmarking = True
                    self.lastspeedupdate = timenow
                    print("speed down")
                if self.onmarking and np.sum(mask_red) ==0 and np.sum(mask_green) ==0:
                    self.onmarking = False

                ###################

                # convert RGB black to BGR black
                RGB_black = [12,12,12]
                BGR_black = np.uint8([[[RGB_black[2], RGB_black[1], RGB_black[0]]]])

                # get HSV black and a range 
                HSV_black = cv2.cvtColor(BGR_black, cv2.COLOR_BGR2HSV)[0][0]
                lower_black = np.array([HSV_black[0]-12, HSV_black[1]-12, HSV_black[2]-12])
                upper_black = np.array([HSV_black[0]+12, HSV_black[1]+12, HSV_black[2]+12])

                # convert from RGB to HSV
                hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

                # apply a mask
                mask = cv2.inRange(hsv, lower_black, upper_black)

                kernel = np.ones((5, 5), np.uint8)
                blur = cv2.blur(mask, (5,5))
                erosion_black = cv2.erode(blur, kernel, iterations=1)

                # find contours
                black_contours, hierarchy = cv2.findContours(erosion_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                ####################

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

                res = cv2.bitwise_and(crop_img, crop_img, mask=erosion)

                # find contours
                contours, hierarchy = cv2.findContours(erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if len(contours) == 0 and len(black_contours) == 0:
                    current_state = "I can't see anything, help me"
                    if self.last_state != current_state:
                        self.last_state = current_state
                        print(current_state) #do the same action done before
                    
                elif len(contours) == 0 and len(black_contours) == 1:
                    # draw contours
                    cv2.drawContours(res, black_contours[0], -1, (0, 255, 0), 3)

                    points = np.zeros((len(black_contours[0]), 2), dtype=np.uint16)
                    for idx, point in enumerate(black_contours[0]):
                        points[idx] = point[0]

                    # point with higher y coordinate
                    orientationPoint = points[np.argsort(points[:,1])[0]]

                    #draw the orientationPoint
                    cv2.circle(res, (orientationPoint[0], orientationPoint[1]), 5, (0,200,255), -1)

                    # calculate centroid
                    m = cv2.moments(black_contours[0], False)
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
                        current_state = "Turn right"
                        if self.last_state != current_state:
                            self.last_state = current_state
                            print(current_state)
                        self.vel_msg.linear.x = .12 * self.speed
                        self.vel_msg.angular.z = -.2 * boost
                    elif cx1 - orientationPoint[0] >= 0:
                        current_state = "Turn left"
                        if self.last_state != current_state:
                            self.last_state = current_state
                            print(current_state)
                        self.vel_msg.linear.x = .12 * self.speed
                        self.vel_msg.angular.z = .2 * boost

                elif len(contours) == 1 and len(black_contours) == 0:
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
                        current_state = "Turn right"
                        if self.last_state != current_state:
                            self.last_state = current_state
                            print(current_state)
                        self.vel_msg.linear.x = .12 * self.speed
                        self.vel_msg.angular.z = -.2 * boost
                    elif cx1 - orientationPoint[0] >= 0:
                        current_state = "Turn left"
                        if self.last_state != current_state:
                            self.last_state = current_state
                            print(current_state)
                        self.vel_msg.linear.x = .12 * self.speed
                        self.vel_msg.angular.z = .2 * boost

                elif len(contours) > 0 and len(black_contours) > 0:

                    # draw contours left
                    cv2.drawContours(res, black_contours[0], -1, (0, 255, 0), 3)

                    # calculate centroid from left
                    m = cv2.moments(black_contours[0], False)
                    try:
                        cx1, cy1 = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
                    except ZeroDivisionError: #if we don't detect anything
                        cx1, cy1 = (int(cropxSize/2), int(width/2)) #center of the image

                    # draw the centroid of the left line
                    cv2.circle(res, (cx1, cy1), 5, (0,0,255), -1)

                    # draw contours right
                    cv2.drawContours(res, contours[0], -1, (0, 255, 0), 3)

                    # calculate centroid from right
                    m = cv2.moments(contours[0], False)
                    try:
                        cx2, cy2 = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
                    except ZeroDivisionError: #if we don't detect anything
                        cx2, cy2 = (int(cropxSize/2), int(width/2)) #center of the image

                    # draw the centroid of the right line
                    cv2.circle(res, (cx2, cy2), 5, (0,0,255), -1)

                    # draw the center of the lines
                    centerx, centery = int((cx1+cx2)/2), int((cy1+cy2)/2)
                    cv2.circle(res, (centerx, centery), 10, (0,0,255), -1)

                    if centery < 25:
                        current_state = "Crossroads"
                        if self.last_state != current_state:
                            self.last_state = current_state
                            print(current_state)
                        self.vel_msg.linear.x = .1 * self.speed
                        self.vel_msg.angular.z = 0
                    elif int(width/2) - centerx > 10:
                        current_state = "Turn left"
                        if self.last_state != current_state:
                            self.last_state = current_state
                            print(current_state)
                        self.vel_msg.linear.x = .15 * self.speed
                        self.vel_msg.angular.z = .1
                    elif int(width/2) - centerx < -10:
                        current_state = "Turn right"
                        if self.last_state != current_state:
                            self.last_state = current_state
                            print(current_state)
                        self.vel_msg.linear.x = .15 * self.speed
                        self.vel_msg.angular.z = -.1
                    else:
                        current_state = "I'm in the center"
                        if self.last_state != current_state:
                            self.last_state = current_state
                            print(current_state)
                        self.vel_msg.linear.x = .2 * self.speed
                        self.vel_msg.angular.z = 0

                #stop if object in front
                if self.prox < 0.11:
                    self.vel_msg.linear.x = 0
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
            cv2.imshow("Res", res)
            cv2.waitKey(1) # you must put in pause gazebo
            '''
        
            self.sleep()

        self.stop()


if __name__ == '__main__':
    try:
        controller = Task2()
        controller.run()
    except rospy.ROSInterruptException as e:
        pass

    cv2.destroyAllWindows()
