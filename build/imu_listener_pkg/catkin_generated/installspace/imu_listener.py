#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def callback(data):
    rospy.loginfo(f"Accel: ({data.linear_acceleration.x:.2f}, {data.linear_acceleration.y:.2f}, {data.linear_acceleration.z:.2f})")

def listener():
    rospy.init_node('imu_listener_node', anonymous=True)
    rospy.Subscriber("/imu0", Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
