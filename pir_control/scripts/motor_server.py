#!/usr/bin/env python

import rospy
import numpy as np
from math import radians, copysign, sqrt, pow, pi
import time
import PyKDL

import tf

from std_msgs.msg import String, Int64, Float64, Bool
from geometry_msgs.msg import Twist, Point, Quaternion
from pir_control.srv import AddMotor
from pir_control.srv import AddMotorResponse

class Publishsers():

    def spt_pub(self, left, right):
        spt_msg = Twist()

        spt_msg.linear.y = left
        spt_msg.angular.y = right

        self.spt_publisher.publish(spt_msg)


class Server(Publishsers):
    def __init__(self):
        self.PI = 3.1415926535897
        self.MAX_WHEEL_VELOCITY = 0.48
        self.MIN_WHEEL_VELOCITY = -(0.48)
        self.WHEEL_SEPARATION = 0.287
        self.HALF_WHEEL_SEPARATION = 0.1435
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.current_linear = 0.0
        self.rate = rospy.Rate(100) # 100hz
        self.rate_time = 0.01
        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/odom'

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

        self.spt_publisher = rospy.Publisher('/spt_vel', Twist, queue_size=10)

        # message for result topic
        self.result = String()

        # Declaration Subscriber
        self.search_sub = rospy.Subscriber('/search/result', Int64 , self.search_callback)

        # Declaration Service Server
        self.server = rospy.Service("/prediction/target_human", AddMotor, self.service_callback)


    def search_callback(self, msg):
        pass

    def service_callback(self, req):
        result = Bool()

        order = req.order.data

        if order == "a":
            acc = req.acceleration.data
            acc = acc / 1000.0

            target_speed = req.target_speed.data
            target_speed = target_speed / 1000.0

            self.acc(acc, target_speed)

        elif order == "f":
            speed = req.speed.data
            speed = speed / 1000.0  # convert from mm to m

            distance = req.distance.data
            distance = distance / 1000.0  # convert from mm to m

            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y

            self.forward(speed, distance, x_start, y_start)

        elif order == "r":
            omega = req.omega.data
            angle = req.angle.data

            relative_angle = angle * 2 * self.PI / 360  # convert from degree to radian

            self.rotation(omega, relative_angle)

        elif order == "t":
            speed = req.speed.data
            speed = speed / 1000.0  #convert from mm/s to m/s

            radius = req.radius.data
            radius = radius / 1000.0   #convert from mm to m

            distance = req.distance.data
            distance = distance / 1000.0  #convert from mm to m

            direction = req.direction.data  #'left' is left, 'right' is right

            self.turning(speed, radius, distance, direction)

        elif order == "sstop":

            down_acc = abs(req.acc.data)
            down_acc = down_acc / 1000.0

            self.slowstop(down_acc)

            print("===== stop =====")

        elif order == "stop":
            self.spt_pub(0, 0)  #velocity is stop

            print("===== stop ======")
        else:
            print "else"


        result.data = True
        return AddMotorResponse(result)


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
                self.spt_pub(left_velocity, right_velocity)
                self.rate.sleep()

        elif left_velocity < target_left_velocity and right_velocity > target_right_velocity:
            while left_velocity < target_left_velocity or right_velocity > target_right_velocity:
                if left_velocity < target_left_velocity:
                    left_velocity += acc
                if right_velocity > target_right_velocity:
                    right_velocity -= acc
                self.spt_pub(left_velocity, right_velocity)
                self.rate.sleep()


        elif left_velocity > target_left_velocity and right_velocity > target_right_velocity:
            while left_velocity > target_left_velocity or right_velocity > target_right_velocity:
                if left_velocity > target_left_velocity:
                    left_velocity -= acc
                if right_velocity > target_right_velocity:
                    right_velocity -= acc
                self.spt_pub(left_velocity, right_velocity)
                self.rate.sleep()

        else:  #left > target_left and right < target_right
            while left_velocity > target_left_velocity or right_velocity < target_right_velocity:
                if left_velocity > target_left_velocity:
                    left_velocity -= acc
                if right_velocity < target_right_velocity:
                    right_velocity += acc
                self.spt_pub(left_velocity, right_velocity)
                self.rate.sleep()

        return left_velocity, right_velocity

    def get_distance(self, distance_x, distance_y):
        distance = sqrt(distance_x**2 + distance_y**2)
        return distance

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

    def acc(self, acc, target_speed):
        self.current_velocity = (self.left_velocity + self.right_velocity) / 2
        while (self.current_velocity < target_speed):
            self.left_velocity += acc * self.rate_time
            self.right_velocity += acc * self.rate_time
            self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
            self.spt_pub(self.left_velocity, self.right_velocity)
            self.rate.sleep()
            self.current_velocity = (self.left_velocity + self.right_velocity) / 2
        print "finish: acc"

    def forward(self, velocity, distance, x_start, y_start):
        time = 0.0
        current_distance = 0.0
        self.set_speed(self.left_velocity, self.right_velocity, velocity, velocity, 0.4)
        while( current_distance < distance):

            self.left_velocity = velocity
            self.right_velocity = velocity
            self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
            self.spt_pub(self.left_velocity,self.right_velocity)
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

            if omega > 0:

                self.left_velocity = - (omega * self.HALF_WHEEL_SEPARATION)
                self.right_velocity = omega * self.HALF_WHEEL_SEPARATION

            else:

                self.left_velocity = - (omega * self.HALF_WHEEL_SEPARATION)
                self.right_velocity = omega * self.HALF_WHEEL_SEPARATION

            self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
            self.spt_pub(self.left_velocity, self.right_velocity)
            self.rate.sleep()
            (position, rotation) = self.get_odom()
            delta_angle = self.normalize_angle(rotation - last_angle)
            turn_angle += delta_angle
            last_angle = rotation
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.spt_pub(self.left_velocity, self.right_velocity)

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

                if direction == 'left':
                    self.left_velocity = target_left_velocity
                    self.right_velocity = target_right_velocity
                elif direction == 'right':
                    self.left_velocity = target_left_velocity
                    self.right_velocity = target_right_velocity

                self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
                self.spt_pub(self.left_velocity, self.right_velocity)
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

                self.left_velocity -= down_acc * self.rate_time
                self.right_velocity -= down_acc * self.rate_time

                self.left_velocity,self.right_velocity = self.constrain(self.left_velocity, self.right_velocity, self.MIN_WHEEL_VELOCITY, self.MAX_WHEEL_VELOCITY)
                self.spt_pub(self.left_velocity, self.right_velocity)
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
                self.spt_pub(self.left_velocity, self.right_velocity)
                self.rate.sleep()
        self.spt_pub(0,0)


if __name__ == '__main__':
    rospy.init_node('prediction_human')

    server = Server()

    # rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , server.ptm_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, server.pose_callback)

    rospy.spin()
