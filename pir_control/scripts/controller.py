#!/usr/bin/env python
import sys
import os
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int64, Float64
import time
import numpy as np
import serial
import PyKDL
from math import radians, copysign, sqrt, pow, pi
import random
import tf
from subprocess import call, Popen
from time import sleep

import rospkg

def file_open(file_name):
    if file_name is not None:
        path = os.path.join(rospkg.RosPack().get_path('pir_control')+ '/motion', file_name)
        filepath = os.path.splitext(path)[0] + '.mts'
        file_content = open(filepath)
        print("\n")
        print("*** "+ file_name + ".mts ***")
        return file_content
    else:
        print("Done")
        rospy.signal_shutdown("Done")

def extract_commands(text):
    text = text.replace('\n'," ")
    text = text.split(" ")
    return text

flag = 0
file_name = rospy.get_param("/textfile_controller/init_file_name") #rospy.get_param("/textfile_controller/init_file_name")
file_content = file_open(file_name)
end_flag = 0
class TextfileController(object):
    def __init__(self):
        self.PI = 3.1415926535897
        self.MAX_WHEEL_VELOCITY = 0.48
        self.MIN_WHEEL_VELOCITY = -(0.48)
        self.WHEEL_SEPARATION = 0.287
        self.HALF_WHEEL_SEPARATION = 0.1435

        self.pub = rospy.Publisher('/spt_vel', Twist, queue_size=10)
        self.led_pub = rospy.Publisher('/led_array', Int16MultiArray, queue_size=100)
        self.vel_msg = Twist()
        self.led_msg = Int16MultiArray()
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.current_linear = 0.0
        self.rate = rospy.Rate(100) # 100hz
        self.rate_time = 0.01
        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/odom'
        self.led = []
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except(tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")
        self.position = Point()
        self.result_list = []
        self.repeat = 5

    def constrain(self, left_velocity, right_velocity, MIN_VELOCITY, MAX_VELOCITY):
        if left_velocity <= MIN_VELOCITY:
            left_velocity = MIN_VELOCITY
        if left_velocity >= MAX_VELOCITY:
            left_velocity = MAX_VELOCITY
        if right_velocity <= MIN_VELOCITY:
            right_velocity = MIN_VELOCITY
        if right_velocity >= MAX_VELOCITY:
            right_velocity = MAX_VELOCITY

        return left_velocity, right_velocity

    def set_speed(self, left_velocity, right_velocity, target_left_velocity, target_right_velocity, acceleration):
        acc = acceleration * self.rate_time
        if left_velocity < target_left_velocity and right_velocity < target_right_velocity:
            while left_velocity < target_left_velocity or right_velocity < target_right_velocity:
                if left_velocity < target_left_velocity:
                    left_velocity += acc
                if right_velocity < target_right_velocity:
                    right_velocity += acc
                self.pub_speed(left_velocity, right_velocity)
                self.rate.sleep()

        elif left_velocity < target_left_velocity and right_velocity > target_right_velocity:
            while left_velocity < target_left_velocity or right_velocity > target_right_velocity:
                if left_velocity < target_left_velocity:
                    left_velocity += acc
                if right_velocity > target_right_velocity:
                    right_velocity -= acc
                self.pub_speed(left_velocity, right_velocity)
                self.rate.sleep()


        elif left_velocity > target_left_velocity and right_velocity > target_right_velocity:
            while left_velocity > target_left_velocity or right_velocity > target_right_velocity:
                if left_velocity > target_left_velocity:
                    left_velocity -= acc
                if right_velocity > target_right_velocity:
                    right_velocity -= acc
                self.pub_speed(left_velocity, right_velocity)
                self.rate.sleep()

        else:  #left > target_left and right < target_right
            while left_velocity > target_left_velocity or right_velocity < target_right_velocity:
                if left_velocity > target_left_velocity:
                    left_velocity -= acc
                if right_velocity < target_right_velocity:
                    right_velocity += acc
                self.pub_speed(left_velocity, right_velocity)
                self.rate.sleep()

        return left_velocity, right_velocity

    def change_file(self, file):
        global flag
        global file_name
        global file_content
        global end_flag

        if flag == 1:
            file_name = file
            file_content = file_open(file_name)
            flag = 0
            rospy.set_param("/textfile_controller/exit_flag", 1)
            while end_flag == 0:
                pass
            rospy.set_param("/textfile_controller/exit_flag", 0)
            instance_name = random.random()
            instance_name = TextfileController()
            instance_name.textfile_controller()

    def executive_file(self, file):
        global flag
        global file_name
        global file_content
        global end_flag

        if flag == 1:
            file_name = file
            file_content = file_open(file_name)
            flag = 0
            rospy.set_param("/textfile_controller/exit_flag", 1)
            rospy.set_param("/textfile_controller/exit_flag", 0)
            instance_name = random.random()
            instance_name = TextfileController()
            instance_name.textfile_controller()




    def pub_speed(self, left_velocity, right_velocity):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = left_velocity
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = right_velocity
        self.vel_msg.angular.z = 0
        self.pub.publish(self.vel_msg)

    def led_def(self, interval_time, led_num_1, led_num_2):
        if led_num_2 != 999:
            self.led = [interval_time, led_num_1, led_num_2]
        else:
            self.led = [interval_time, led_num_1]
        self.led_msg.data = self.led
        self.led_pub.publish(self.led_msg)

    def get_distance(self, distance_x, distance_y):
        distance = sqrt(distance_x**2 + distance_y**2)
        return distance

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.vel_msg.publish(Twist())
        rospy.sleep(1)

    def acceleration(self, target_speed, distance, x_start, y_start):
        self.current_velocity = (self.left_velocity + self.right_velocity) / 2  #Decide initial velocity

        new_acc = (target_speed * target_speed) - (self.current_velocity * self.current_velocity) / 2 / distance

        if target_speed < 0:
            new_acc = -new_acc

        time = 0.0
        current_distance = 0.0
        while ( current_distance < distance):

            exit_flag = rospy.get_param("/textfile_controller/exit_flag")
            if exit_flag == 1:
                rospy.set_param("/textfile_controller/exit_flag", 0)
                end_flag = 1
                return

            self.left_velocity += new_acc * self.rate_time
            self.right_velocity += new_acc * self.rate_time
            self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
            self.pub_speed(self.left_velocity, self.right_velocity)
            time += 0.01
            self.rate.sleep()
            (position, rotation) = self.get_odom()
            current_distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
        print ("finish: acceleration {0:4.0f}(mm/s) {1:4.0f}(mm) : acceleration value is {2:4.0f} (mm\s^2)".format(target_speed*1000,distance*1000,new_acc*1000))

    def acc(self, acc, target_speed):
        self.current_velocity = (self.left_velocity + self.right_velocity) / 2
        while (self.current_velocity < target_speed):
            self.left_velocity += acc * self.rate_time
            self.right_velocity += acc * self.rate_time
            self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
            self.pub_speed(self.left_velocity, self.right_velocity)
            self.rate.sleep()
            self.current_velocity = (self.left_velocity + self.right_velocity) / 2
        print "finish: acc"

    def forward(self, velocity, distance, x_start, y_start):
        time = 0.0
        current_distance = 0.0
        self.set_speed(self.left_velocity, self.right_velocity, velocity, velocity, 0.4)
        while( current_distance < distance):

            exit_flag = rospy.get_param("/textfile_controller/exit_flag")
            if exit_flag == 1:
                rospy.set_param("/textfile_controller/exit_flag", 0)
                end_flag = 1
                return

            self.left_velocity = velocity
            self.right_velocity = velocity
            self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
            self.pub_speed(self.left_velocity,self.right_velocity)
            time += 0.01
            self.rate.sleep()
            (position, rotation) = self.get_odom()
            current_distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))

        # current_linear = vel_msg.linear.x
        print ("finish: forward {0:4.0f}(mm/s) {1:4.0f}(mm)".format(velocity*1000,distance*1000))

    def rotation(self, omega, relative_angle):
        angular_tolerance = radians(2.5)

        (prev_position, prev_rotation) = self.get_odom()
        last_angle = prev_rotation
        turn_angle = 0.0

        while( abs(turn_angle + angular_tolerance) < abs(relative_angle)):

            exit_flag = rospy.get_param("/textfile_controller/exit_flag")
            if exit_flag == 1:
                rospy.set_param("/textfile_controller/exit_flag", 0)
                end_flag = 1
                return

            if omega > 0:

                self.left_velocity = - (omega * self.HALF_WHEEL_SEPARATION)
                self.right_velocity = omega * self.HALF_WHEEL_SEPARATION

            else:

                self.left_velocity = - (omega * self.HALF_WHEEL_SEPARATION)
                self.right_velocity = omega * self.HALF_WHEEL_SEPARATION

            self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
            self.pub_speed(self.left_velocity, self.right_velocity)
            self.rate.sleep()
            (position, rotation) = self.get_odom()
            delta_angle = normalize_angle(rotation - last_angle)
            turn_angle += delta_angle
            last_angle = rotation
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.pub_speed(self.left_velocity, self.right_velocity)

        print ("finish: rotation {0}(rad/s) {1}(radian)".format(omega,relative_angle))

    def turning(self, speed, radius, distance, direction):
        time = 0.0
        current_distance = 0.0

        if direction == 'left' or direction == 'right':
            if direction == 'left':
                if (self.left_velocity == 0 and self.right_velocity == 0):
                    target_left_velocity = speed * (radius - self.HALF_WHEEL_SEPARATION) / 2 / radius * 2
                    target_right_velocity = speed * (radius + self.HALF_WHEEL_SEPARATION) / 2 / radius * 2
                    self.set_speed(self.left_velocity, self.right_velocity, target_left_velocity,target_right_velocity, 0.2)
                else:
                    target_left_velocity = speed * (radius - self.HALF_WHEEL_SEPARATION) / 2 / radius * 2
                    target_right_velocity = speed * (radius + self.HALF_WHEEL_SEPARATION) / 2 / radius * 2
                    self.set_speed(self.left_velocity, self.right_velocity, target_left_velocity,target_right_velocity, 0.4)
            else:
                if (self.left_velocity == 0 and self.right_velocity == 0):
                    target_left_velocity = speed * (radius + self.HALF_WHEEL_SEPARATION) / radius
                    target_right_velocity = speed * (radius - self.HALF_WHEEL_SEPARATION) /radius
                    self.set_speed(self.left_velocity, self.right_velocity, target_left_velocity, target_right_velocity, 0.2)
                else:
                    target_left_velocity = speed * (radius + self.HALF_WHEEL_SEPARATION) / radius
                    target_right_velocity = speed * (radius - self.HALF_WHEEL_SEPARATION) / radius
                    self.set_speed(self.left_velocity, self.right_velocity, target_left_velocity, target_right_velocity, 0.4)

            while( current_distance < distance):

                (prev_position, prev_rotation) = self.get_odom()
                exit_flag = rospy.get_param("/textfile_controller/exit_flag")
                if exit_flag == 1:
                    rospy.set_param("/textfile_controller/exit_flag", 0)
                    end_flag = 1

                if direction == 'left':
                    self.left_velocity = target_left_velocity
                    self.right_velocity = target_right_velocity
                elif direction == 'right':
                    self.left_velocity = target_left_velocity
                    self.right_velocity = target_right_velocity

                self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
                self.pub_speed(self.left_velocity, self.right_velocity)
                self.rate.sleep()
                (position, rotation) = self.get_odom()
                current_distance += sqrt(pow((position.x - prev_position.x), 2) + pow((position.y - prev_position.y), 2))


            print ("finish: turning {0:4.0f}(mm/s) {1:4.0f}(mm) {2:4.0f}(mm) {3}".format(speed*1000,radius*1000,distance*1000,direction))

        elif direction == "":
            print "Please write direction : 'left' or 'right' "

        else:
            print "" + direction + " : type error"

    def slowstop(self, down_acc):

        self.current_velocity = (self.left_velocity + self.right_velocity) / 2  #Decide initial velocity

        if self.current_velocity > 0:
            while ( self.left_velocity >= 0 and self.right_velocity >= 0 ):

                exit_flag = rospy.get_param("/textfile_controller/exit_flag")
                if exit_flag == 1:
                    rospy.set_param("/textfile_controller/exit_flag", 0)
                    end_flag = 1

                self.left_velocity -= down_acc * self.rate_time
                self.right_velocity -= down_acc * self.rate_time

                self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
                self.pub_speed(self.left_velocity, self.right_velocity)
                self.rate.sleep()
        else:
            while (self.left_velocity <= 0 and self.right_velocity <= 0):

                exit_flag = rospy.get_param("/textfile_controller/exit_flag")
                if exit_flag == 1:
                    rospy.set_param("/textfile_controller/exit_flag", 0)
                    end_flag = 1

                self.left_velocity += down_acc * self.rate_time
                self.right_velocity += down_acc * self.rate_time


                self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
                self.pub_speed(self.left_velocity, self.right_velocity)
                self.rate.sleep()
        self.pub_speed(0,0)

    def textfile_controller(self):
        global flag
        global file_content
        global end_flag
        for texts in file_content:
            command_param_set = extract_commands(texts)
            command = command_param_set[0]
            print(command_param_set[-1])

            if command == "acceleration":


                target_speed = float(command_param_set[1])
                target_speed = target_speed / 1000  #convert from mm/s to m/s

                distance = float(command_param_set[2])
                distance = distance / 1000  #convert from mm/s^2 to m/s^2

                (position, rotation) = self.get_odom()
                x_start = position.x
                y_start = position.y

                self.acceleration(target_speed, distance, x_start, y_start)

            elif command == "acc":
                acc = float(command_param_set[1])
                acc = acc / 1000
                
                target_speed = float(command_param_set[2])
                target_speed = target_speed / 1000

                self.acc(acc, target_speed)

            #forward
            elif command == "forward":

                speed = float(command_param_set[1])
                speed = speed / 1000  # convert from mm to m

                distance = float(command_param_set[2])
                distance = distance / 1000  # convert from mm to m

                (position, rotation) = self.get_odom()
                x_start = position.x
                y_start = position.y

                self.forward(speed, distance, x_start, y_start)

            elif command == "rotation":

                omega = float(command_param_set[1])
                angle = float(command_param_set[2])

                relative_angle = angle * 2 * self.PI / 360  # convert from degree to radian

                self.rotation(omega, relative_angle)

            elif command == "turning":

                speed = float(command_param_set[1])
                speed = speed / 1000  #convert from mm/s to m/s

                radius = float(command_param_set[2])
                radius = radius / 1000   #convert from mm to m

                distance = float(command_param_set[3])
                distance = distance / 1000  #convert from mm to m

                direction = command_param_set[4]  #'left' is left, 'right' is right

                self.turning(speed, radius, distance, direction)

            elif command == "sleep":
                time = float(command_param_set[1])
                time = time / 1000  #convert from s to ms
                rospy.sleep(time)
                print("----- sleep -----")

            elif command == "pause":
                pause_time = float(command_param_set[1])
                pause_time = pause_time / 1000  #convert ms to s

                time = 0.0
                while time < pause_time:
                    exit_flag = rospy.get_param("/textfile_controller/exit_flag")
                    if exit_flag == 1:
                        rospy.set_param("/textfile_controller/exit_flag", 0)
                        end_flag = 1
                    time += 0.01
                    self.rate.sleep()

                print("_____ pause ______")


            elif command == "stop":

                self.pub_speed(0, 0)  #velocity is stop

                print("===== stop ======")

            elif command == 'slowstop':

                down_acc = float(command_param_set[1])
                down_acc = down_acc / 1000

                self.slowstop(down_acc)

                print("===== stop =====")



            elif command == 'detect':
                exit_flag = rospy.get_param("/textfile_controller/exit_flag")
                if exit_flag == 1:
                    rospy.set_param("/textfile_controller/exit_flag", 0)
                    end_flag = 1
                    return
                flag = 1
                #  distance [m] is from robot to obstacle.
                distance = float(command_param_set[1])
                rospy.set_param("/textfile_controller/distance", distance)
                next_file = command_param_set[2]
                rospy.set_param("/textfile_controller/next_file", next_file)
                print ("searching someone within {:.3f}(m)".format(distance))

            elif command == "e":
                exit_flag = rospy.get_param("/textfile_controller/exit_flag")
                if exit_flag == 1:
                    rospy.set_param("/textfile_controller/exit_flag", 0)
                    end_flag = 1
                    return
                flag = 1
                file = command_param_set[1]
                self.executive_file(file)
            elif command == "end":
                file_open(None)
            elif command == "":
                pass

            else:

                print "" + command + " : type error"

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]

def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

class Move(TextfileController):
    def __init__(self):
        super(Move, self).__init__()
        self.result_obs_sub = rospy.Subscriber("/result", Int64, self.callback)
        move2 = TextfileController()
        move2.textfile_controller()

    def callback(self, result_obs):
        if result_obs.data == 1:
            next_file = rospy.get_param("/textfile_controller/next_file")
            self.change_file(next_file)



def main():
    rospy.init_node('textfile_controller', anonymous=True)
    rospy.set_param("/textfile_controller/distance", 0)
    rospy.set_param("/textfile_controller/exit_flag", 0)
    move = Move()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")

if __name__ == '__main__':
    main()
