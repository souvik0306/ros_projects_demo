#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

def callback(data):
    msg_time = data.header.stamp.secs + data.header.stamp.nsecs * 1e-9

    rospy.loginfo(f"Timestamp: {msg_time} seconds")

    rospy.loginfo(f"Linear Accel: ({data.linear_acceleration.x:.2f}, {data.linear_acceleration.y:.2f}, {data.linear_acceleration.z:.2f})")
    
    rospy.loginfo(f"Gyro: ({data.angular_velocity.x:.2f}, {data.angular_velocity.y:.2f}, {data.angular_velocity.z:.2f})")
    
    rospy.loginfo("-----------------------")

def listener():
    rospy.init_node('imu_listener_node', anonymous=True)
    rospy.Subscriber("/imu0", Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
