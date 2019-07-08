#!/usr/bin/env python
import sys
import os
import rospy
from std_msgs.msg import Int64, Float64
import time
import numpy as np
import serial
import math
from subprocess import call, Popen
from time import sleep
import rospkg

# from pir_control.msg import Motor
from pir_control.srv import *


class TextfileController(object):
    def __init__(self):
        self.rate = rospy.Rate(100) # 100hz
        self.rate_time = 0.01
        self.scan_flag = 0
        self.motor = rospy.ServiceProxy('/pir_control/motor', AddMotor)


    def executive_file(self, file):
        ### start main controller ###
        instance_name = TextfileController()
        instance_name.textfile_controller()

    def file_open(self, file_name):
        if file_name is not None:
            path = os.path.join(rospkg.RosPack().get_path('pir_control')+ '/motion', file_name)
            filepath = os.path.splitext(path)[0] + '.mts'
            file_content = open(filepath)
            print("\n")
            print("*** "+ file_name + ".mts ***")
            return file_content
        else:
            print("Done")
            self.shutdown()
            rospy.signal_shutdown("Done")

    def extract_commands(self, text):
        text = text.replace('\n'," ")
        text = text.split(" ")
        return text

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.vel_msg.publish(Twist())
        rospy.sleep(1)


    def textfile_controller(self):

        file_name = rospy.get_param("/textfile_controller/file_name")
        file_content = self.file_open(file_name)

        for texts in file_content:

            command_param_set = self.extract_commands(texts)
            command = command_param_set[0]

            req = AddMotorRequest()


            if command == "acceleration":
                acc = float(command_param_set[1])
                # acc = acc / 1000

                target_speed = float(command_param_set[2])
                # target_speed = target_speed / 1000

                req.order.data = "a"
                req.acceleration.data = acc
                req.target_speed.data = target_speed

                result = self.motor(req)

            #forward
            elif command == "forward":

                speed = float(command_param_set[1])
                # speed = speed / 1000  # convert from mm to m

                distance = float(command_param_set[2])
                # distance = distance / 1000  # convert from mm to m

                req.order.data = "f"
                req.speed.data = speed
                req.distance.data = distance

                result = self.motor(req)


            elif command == "rotation":

                omega = float(command_param_set[1])
                angle = float(command_param_set[2])

                relative_angle = angle * 2 * math.pi / 360  # convert from degree to radian

                req.order.data = "r"
                req.omega.data = omega
                req.angle.data = angle

                result = self.motor(req)


            elif command == "turning":

                speed = float(command_param_set[1])
                # speed = speed / 1000  #convert from mm/s to m/s

                radius = float(command_param_set[2])
                # radius = radius / 1000   #convert from mm to m

                distance = float(command_param_set[3])
                # distance = distance / 1000  #convert from mm to m

                direction = command_param_set[4]  #'left' is left, 'right' is right

                req.order.data = "t"
                req.speed.data = speed
                req.radius.data = radius
                req.distance.data = distance
                req.direction.data = direction

                result = self.motor(req)


            elif command == "sleep":
                time = float(command_param_set[1])
                time = time / 1000  #convert from s to ms
                rospy.sleep(time)
                print("----- sleep -----")

            elif command == "pause":
                pause_time = float(command_param_set[1])
                # pause_time = pause_time / 1000  #convert ms to s

                req.order.data = "p"
                req.time.data = pause_time

                result = self.motor(req)
                print("_____ pause ______")


            elif command == "stop":

                req.order.data = "s"

                result = self.motor(req)
                print("===== stop ======")

            elif command == 'slowstop':

                down_acc = float(command_param_set[1])
                # down_acc = down_acc / 1000
                req.order.data = "ss"
                req.acceleration.data = down_acc

                result = self.motor(req)
                print("===== stop =====")



            elif command == 'detect':
                #  distance [m] is from robot to obstacle.
                distance = float(command_param_set[1])
                rospy.set_param("/textfile_controller/distance", distance)
                next_file = command_param_set[2]
                rospy.set_param("/textfile_controller/next_file", next_file)

                print ("searching someone within {:.3f}(m)".format(distance))

            elif command == "e":
                file = command_param_set[1]
                self.executive_file(file)

            elif command == "end":
                self.file_open(None)

            elif command == "" or command == "#":
                pass

            else:
                print "" + command + " : type error"


class Move(TextfileController):
    def __init__(self):
        super(Move, self).__init__()
        self.result_obs_sub = rospy.Subscriber("/search/result", Int64, self.search_callback)
        move2 = TextfileController()
        move2.textfile_controller()

    def search_callback(self, scan_result):
        if scan_result.data == 1:
            self.scan_flag = 1
        else:
            self.scan_flag = 0



def main():
    rospy.init_node('textfile_controller2', anonymous=True)
    rospy.set_param("/textfile_controller/distance", 0)
    move = Move()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")

if __name__ == '__main__':
    main()
