#!/usr/bin/env python3

import rospy
import inspect
from sensor_msgs.msg import Imu, Temperature
from im1r_ros_driver.msg import IM1R_EXTRA

def callback(data):
    attributes = inspect.getmembers(data, lambda a: not (inspect.isroutine(a)))
    rospy.loginfo(data._type)
    for a in attributes:
        if not (a[0].startswith('_') or a[0].endswith('_')):
            rospy.loginfo("%s: %s" % (a[0], a[1]))

def main():
    rospy.init_node('im1r_data_subscriber', anonymous=True)

    rospy.Subscriber('imu/data', Imu, callback)
    rospy.Subscriber('temperature', Temperature, callback)
    rospy.Subscriber('im1r/extra', IM1R_EXTRA, callback)

    rospy.loginfo("IM1R Data Subscriber Node Started. Waiting for data...")
    rospy.spin()

if __name__ == "__main__":
    main()
