#!/usr/bin/env python
import rospy
from thymio import ThymioController, PID
from sensor_msgs.msg import Range

class Task2(ThymioController):

    def __init__(self):
        super(Task2, self).__init__()

        self.sensors = ["left",
                        "center_left",
                        "center",
                        "center_right",
                        "right"]

        self.prox_sub = [
            rospy.Subscriber(f"/{self.name}/proximity/{sensor}", 
                            Range, 
                            self.prox_update, 
                            sensor)
            for sensor in self.sensors
        ]
        
        self.prox_distances = dict()

        #PID controller to control the angular velocity
        self.rotation_controller = PID(5, 0, 1)

    def run(self, distance_tolerance=.11, sensor_tolerance=.001):
        while not rospy.is_shutdown():
            self.sleep()
            if len(self.prox_distances) == len(self.sensors):
                break

        while not rospy.is_shutdown():
            distances = self.prox_distances.values()

            if sum(distance < distance_tolerance for distance in distances) >= 2:
                break

            self.vel_msg.linear.x = .3
            self.vel_msg.angular.z = 0

            self.velocity_publisher.publish(self.vel_msg)
            self.sleep()

        self.stop()

        count = 0

        while not rospy.is_shutdown():
            #Detect if robot is facing the wall
            target_diff = 0
            diff = self.prox_distances["center_left"] - self.prox_distances["center_right"]

            error = target_diff - diff

            #Ensure that the error stays below target for a few cycles to smooth out the noise a bit
            if abs(error) <= sensor_tolerance:
                count += 1
            else:
                count = 0

            if count == 3:
                break

            #Use the PID controller to minimize the distance difference
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = self.rotation_controller.step(error, self.step.to_sec())

            self.velocity_publisher.publish(self.vel_msg)
            self.sleep()

        self.stop()

    def prox_update(self, data, sensor):
        self.prox_distances[sensor] = data.range


if __name__ == '__main__':
    try:
        controller = Task2()
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
