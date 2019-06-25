#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64
from std_msgs.msg import Float64


class Lidar(object):
    def __init__(self):
        self.scan_data_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.detect_result_pub = rospy.Publisher("/result", Int64, queue_size=1)

    def callback(self, scan_data):
        result = Int64()
        distance = rospy.get_param("/textfile_controller/distance")
        result.data = self.detect_obs(scan_data.ranges, distance)
        self.detect_result_pub.publish(result)

    def judge_pub(self, result):
        if result.data == 1:
            self.count += 1
        else:
            self.count = 0

        print self.count 
        if self.count == 5:
            self.detect_result_pub.publish(1)
            self.count = 0

    def detect_obs(self, scan_data, distance):
        #  scan data [m]
        data = np.array(scan_data)
        #  size of scan data = 360
        data_num = len(data)
        #  range of object for detector [m]
        obs_range = 0.15
        #  threshold
        margin = 17
        dif = distance - data
        #  if obstacles is detected, retrun 1.
        obs_flag = 0
        #  if dif exceed threshold, count.
        num = 0

        #searching area is from 0 to 23 and from 334 to 359
        for i in range(24):
            if dif[i] > 0 and dif[i] < distance:
                num += 1

        for i in range(data_num - 26, data_num):
            if dif[i] > 0 and dif[i] < distance:
                num += 1

        if num > margin:
            obs_flag = 1


        # print("scan line size{:04x}".format(data_num))
        return obs_flag


def main():
    lidar = Lidar()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")


if __name__ == '__main__':
    rospy.init_node("lidar", anonymous=True)
    main()
