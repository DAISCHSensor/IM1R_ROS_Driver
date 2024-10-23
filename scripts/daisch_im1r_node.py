#!/usr/bin/env python3

import rospy
import sys
import serial
from parser import parse_frame
from im1r_ros_driver.msg import IM1R_EXTRA
from sensor_msgs.msg import Imu, Temperature
import math

# Constants
TEMP_DBL = -1.0
USED_FRAME_LEN = 62
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 115200
FRAME_ID = "IM1R"

class RealTimeCOM:
    def __init__(self, port, rate=115200, timeout=2):
        self.port = port
        self.rate = rate
        self.timeout = timeout
        self.com = None

    def open(self):
        self.com = serial.Serial(self.port, self.rate, timeout=self.timeout)

    def close(self):
        if self.com and self.com.is_open:
            self.com.close()

    def get_data(self):
        return self.com.readline().strip() if self.com else ""

def initialize_serial_port():
    try:
        port = sys.argv[1]
    except IndexError:
        port = DEFAULT_PORT
        rospy.loginfo(f'Default port used: {DEFAULT_PORT}')
    return port

def initialize_serial_baudrate():
    try:
        baudrate = sys.argv[2]
    except IndexError:
        baudrate = DEFAULT_BAUDRATE
        rospy.loginfo(f'Default baudrate used: {DEFAULT_BAUDRATE}')
    return baudrate

def initialize_publishers():
    pub_imu_data = rospy.Publisher('imu/data', Imu, queue_size=10)
    pub_temperature = rospy.Publisher('temperature', Temperature, queue_size=10)
    pub_im1r_extra = rospy.Publisher('im1r/extra', IM1R_EXTRA, queue_size=10)
    return pub_imu_data, pub_temperature, pub_im1r_extra

def publish_imu_data(pub, stamp, data):
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = FRAME_ID
    msg.linear_acceleration.x = data['AccX']
    msg.linear_acceleration.y = data['AccY']
    msg.linear_acceleration.z = data['AccZ']
    msg.angular_velocity.x = data['GyroX'] * (math.pi / 180)
    msg.angular_velocity.y = data['GyroY'] * (math.pi / 180)
    msg.angular_velocity.z = data['GyroZ'] * (math.pi / 180)
    msg.orientation.w = msg.orientation.x = msg.orientation.y = msg.orientation.z = TEMP_DBL
    msg.orientation_covariance[0] = msg.orientation_covariance[4] = msg.orientation_covariance[8] = TEMP_DBL
    pub.publish(msg)
    # rospy.loginfo(msg._type)

def publish_temperature(pub, stamp, data):
    msg = Temperature()
    msg.header.stamp = stamp
    msg.header.frame_id = FRAME_ID
    msg.temperature = data['Temperature']
    msg.variance = TEMP_DBL
    pub.publish(msg)
    # rospy.loginfo(msg._type)

def publish_extra_data(pub, data):
    msg = IM1R_EXTRA()
    msg.count = data['Count']
    msg.timestamp = data['Timestamp']
    msg.pitch = data['Pitch']
    msg.roll = data['Roll']
    msg.imu_status = data['IMUStatus']
    msg.gyro_bias_x = data['GyroBiasX']
    msg.gyro_bias_y = data['GyroBiasY']
    msg.gyro_bias_z = data['GyroBiasZ']
    msg.gyro_static_bias_x = data['GyroStaticBiasX']
    msg.gyro_static_bias_y = data['GyroStaticBiasY']
    msg.gyro_static_bias_z = data['GyroStaticBiasZ']
    pub.publish(msg)
    # rospy.loginfo(msg._type)

def main():
    serial_port = initialize_serial_port()
    serial_baudrate = initialize_serial_baudrate()

    rospy.init_node('daisch_im1r_node')
    pub_imu_data, pub_temperature, pub_im1r_extra = initialize_publishers()

    try:
        serial_com = RealTimeCOM(serial_port, serial_baudrate, timeout=1)
        serial_com.open()
        rospy.loginfo("SerialPort Open")

        while not rospy.is_shutdown():
            data = serial_com.get_data()
            stamp = rospy.Time.now()
            
            if len(data) != USED_FRAME_LEN:
                continue

            try:
                parsed_data = parse_frame(data)
                if parsed_data is None:
                    rospy.logwarn(f"Parse frame error! Data was: {data}")
                else:
                    publish_imu_data(pub_imu_data, stamp, parsed_data)
                    publish_temperature(pub_temperature, stamp, parsed_data)
                    publish_extra_data(pub_im1r_extra, parsed_data)

            except ValueError as e:
                rospy.logwarn(f"Value error, likely due to missing fields in the messages. Error was: {e}")

    except rospy.ROSInterruptException:
        serial_com.close()  # Close serial port

if __name__ == "__main__":
    main()
