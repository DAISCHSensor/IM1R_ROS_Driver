#!/usr/bin/env python3

import rospy
import sys
import serial
from parser import parse_frame, euler_to_quaternion
from im1r_ros_driver.msg import IM1R_EXTRA
from sensor_msgs.msg import Imu, Temperature
import math

# Constants
TEMP_DBL = -1.0
LEN_A = 68
LEN_B = 96
MIN_FRAME_LEN = min(LEN_A, LEN_B)
FRAME_HEAD = b'\xA5\x5A'
FRAME_ID = "imu_link"
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 115200

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
        return self.com.readline() if self.com else b''

    def clear_buff(self):
        self.com.reset_input_buffer()

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
    pub_rawimu_data = rospy.Publisher('rawimu/data', Imu, queue_size=10)
    pub_temperature = rospy.Publisher('temperature', Temperature, queue_size=10)
    pub_im1r_extra = rospy.Publisher('im1r/extra', IM1R_EXTRA, queue_size=10)
    return pub_imu_data, pub_rawimu_data, pub_temperature, pub_im1r_extra

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

    if all(k in data and data[k] is not None for k in ['Quat0', 'Quat1', 'Quat2', 'Quat3']):
            msg.orientation.w = data['Quat0']
            msg.orientation.x = data['Quat1']
            msg.orientation.y = data['Quat2']
            msg.orientation.z = data['Quat3']
    else:
        quaternion = euler_to_quaternion(data['Roll'], data['Pitch'], data['Yaw'])
        msg.orientation.w = quaternion[0]
        msg.orientation.x = quaternion[1]
        msg.orientation.y = quaternion[2]
        msg.orientation.z = quaternion[3]

    # msg.orientation_covariance[0] = msg.orientation_covariance[4] = msg.orientation_covariance[8] = TEMP_DBL
    pub.publish(msg)

def publish_temperature(pub, stamp, data):
    msg = Temperature()
    msg.header.stamp = stamp
    msg.header.frame_id = FRAME_ID
    msg.temperature = data['Temperature']
    # msg.variance = TEMP_DBL
    pub.publish(msg)
    # rospy.loginfo(msg._type)

def publish_extra_data(pub, data):
    msg = IM1R_EXTRA()
    msg.count = data.get('Count', 0)
    msg.timestamp = data.get('Timestamp', 0.0)
    msg.pitch = data.get('Pitch', 0.0)
    msg.roll = data.get('Roll', 0.0)
    msg.yaw = data.get('Yaw', 0.0)
    msg.imu_status = data.get('IMUStatus', 0)

    def deg_to_rad_safe(val):
        return (val or 0.0) * (math.pi / 180)

    msg.gyro_bias_x = deg_to_rad_safe(data.get('GyroBiasX'))
    msg.gyro_bias_y = deg_to_rad_safe(data.get('GyroBiasY'))
    msg.gyro_bias_z = deg_to_rad_safe(data.get('GyroBiasZ'))
    msg.gyro_static_bias_x = deg_to_rad_safe(data.get('GyroStaticBiasX'))
    msg.gyro_static_bias_y = deg_to_rad_safe(data.get('GyroStaticBiasY'))
    msg.gyro_static_bias_z = deg_to_rad_safe(data.get('GyroStaticBiasZ'))

    pub.publish(msg)


def read_frame(serial_com):
    data = serial_com.get_data()
    if not data:
        return None
    head_pos = data.find(FRAME_HEAD)
    if head_pos < 0:
        return None
    if head_pos > 0:
        data = data[head_pos:]
    while len(data) < MIN_FRAME_LEN:
        data += serial_com.get_data()
    try:
        payload_len = data[4]
    except IndexError:
        return None
    expect_len = LEN_A if payload_len == 60 else LEN_B
    while len(data) < expect_len:
        data += serial_com.get_data()
    frame = data[:expect_len]
    if not frame.endswith(b'\x0D\x0A'):
        return None
    return frame


def main():
    serial_port = initialize_serial_port()
    serial_baudrate = initialize_serial_baudrate()

    rospy.init_node('daisch_im1r_node')
    pub_imu_data, pub_rawimu_data, pub_temperature, pub_im1r_extra = initialize_publishers()

    try:
        serial_com = RealTimeCOM(serial_port, serial_baudrate, timeout=1)
        serial_com.open()
        serial_com.clear_buff()
        rospy.loginfo("SerialPort Open")
        while not rospy.is_shutdown():
            frame = read_frame(serial_com)
            if not frame:
                continue
            try:
                data, data_raw = parse_frame(frame)
                if data is not None and data_raw is not None:
                    stamp = rospy.Time.now()
                    publish_imu_data(pub_imu_data, stamp, data)
                    publish_imu_data(pub_rawimu_data, stamp, data_raw)
                    publish_temperature(pub_temperature, stamp, data)
                    publish_extra_data(pub_im1r_extra, data)

            except ValueError as e:
                rospy.logwarn(f"Value error, likely due to missing fields in the messages. Error was: {e}")     

    except rospy.ROSInterruptException:
        serial_com.close()  # Close serial port

if __name__ == "__main__":
    main()
