#!/usr/bin/env python3

import rospy
import sys
import serial
from im1r_ros_driver.msg import IM1R_EXTRA
from sensor_msgs.msg import Imu, Temperature
import math

if sys.version_info[0] < 3:
    print("Running with Python 2")
    from parser_python2 import parse_frame, euler_to_quaternion
else:
    print("Running with Python 3")
    from parser_python3 import parse_frame, convert_quaternion, convert_vector


# Constants
TEMP_DBL = -1.0
USED_FRAME_LEN = 84
FRAME_ID = "IM1R"
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
        rospy.loginfo('Default port used: {DEFAULT_PORT}')
    return port

def initialize_serial_baudrate():
    try:
        baudrate = sys.argv[2]
    except IndexError:
        baudrate = DEFAULT_BAUDRATE
        rospy.loginfo('Default baudrate used: {DEFAULT_BAUDRATE}')
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

    ax, ay, az = convert_vector(data['AccX'], data['AccY'], data['AccZ'])
    msg.linear_acceleration.x = ax
    msg.linear_acceleration.y = ay
    msg.linear_acceleration.z = az

    gx, gy, gz = convert_vector(data['GyroX'], data['GyroY'], data['GyroZ'])
    msg.angular_velocity.x = gx * (math.pi / 180)
    msg.angular_velocity.y = gy * (math.pi / 180)
    msg.angular_velocity.z = gz * (math.pi / 180)

    # qx0 = data['Quat1']
    # qy0 = data['Quat2']
    # qz0 = data['Quat3']
    # qw0 = data['Quat0']

    # rospy.loginfo("[origin] x: %.4f, y: %.4f, z: %.4f, w: %.4f" % (qx0, qy0, qz0, qw0))

    qx, qy, qz, qw = convert_quaternion(data['Quat1'], data['Quat2'], data['Quat3'], data['Quat0'])
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw

    # rospy.loginfo("[convert] x: %.4f, y: %.4f, z: %.4f, w: %.4f" % (qx, qy, qz, qw))
    
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
    msg.count = data['Count']
    msg.timestamp = data['Timestamp']
    msg.pitch = data['Pitch']
    msg.roll = data['Roll']
    msg.yaw = data['Yaw']
    msg.imu_status = data['IMUStatus']
    msg.gyro_bias_x = data['GyroBiasX'] * (math.pi / 180)
    msg.gyro_bias_y = data['GyroBiasY'] * (math.pi / 180)
    msg.gyro_bias_z = data['GyroBiasZ'] * (math.pi / 180)
    msg.gyro_static_bias_x = data['GyroStaticBiasX'] * (math.pi / 180)
    msg.gyro_static_bias_y = data['GyroStaticBiasY'] * (math.pi / 180)
    msg.gyro_static_bias_z = data['GyroStaticBiasZ'] * (math.pi / 180)
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
        serial_com.clear_buff()
        rospy.loginfo("SerialPort Open")
        while not rospy.is_shutdown():
            data = serial_com.get_data()
            while len(data) < USED_FRAME_LEN:
                data += serial_com.get_data()
            try:
                parsed_data = parse_frame(data)
                if parsed_data:
                    stamp = rospy.Time.now()
                    publish_imu_data(pub_imu_data, stamp, parsed_data)
                    publish_temperature(pub_temperature, stamp, parsed_data)
                    publish_extra_data(pub_im1r_extra, parsed_data)

            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the messages. Error was: {e}")     

    except rospy.ROSInterruptException:
        serial_com.close()  # Close serial port

if __name__ == "__main__":
    main()