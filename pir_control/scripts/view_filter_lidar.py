#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan


class Publishsers():
    def make_msg(self, msg):

       current_time = rospy.Time.now()
       self.scan.header.stamp = current_time
       self.scan.header.frame_id = 'base_scan'
       self.scan.angle_min = msg.angle_min 
       self.scan.angle_max = msg.angle_max
       self.scan.angle_increment = msg.angle_increment
       self.scan.time_increment = msg.time_increment
       self.scan.range_min = msg.range_min
       self.scan.range_max = msg.range_max

       self.scan.ranges = []
       self.scan.intensities = msg.intensities 
       for i in range(len(msg.ranges)):
	  if i < self.ang_min or i > 360 - self.ang_max:
              if str(msg.ranges[i]) == "inf":
                  self.scan.ranges.append(0.0)
              else:
                  self.scan.ranges.append(msg.ranges[i])
          else:
	      self.scan.ranges.append(0.0)

    def send_msg(self):
        self.publisher.publish(self.scan)


class Subscribe_publishers(Publishsers):
    def __init__(self):

        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.publisher = rospy.Publisher('/filtered/scan', LaserScan, queue_size=10)
	self.scan = LaserScan()

        self.ang_min = 24
        self.ang_max = 26


    def callback(self, msg):

        self.make_msg(msg)
        self.send_msg()

def main():

    rospy.init_node('view_filter_lidar')

    sub = Subscribe_publishers()

    rospy.spin()

if __name__ == '__main__':
    main()
