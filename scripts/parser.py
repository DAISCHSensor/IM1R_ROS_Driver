import struct
import math


# Define constants
HEAD = b'\xA5\x5A'
TAIL = b'\x0D\x0A'
HEAD_LEN = 2
DOM_LEN = 1
CMD_LEN = 1
LEN_LEN = 1
CRC_LEN = 1
TAIL_LEN = 2
FRAME_LEN_OLD = 68
FRAME_LEN_BINARY = 72
FRAME_LEN_SIMPLE = 47


def checksum_crc(data, crc_ref):
    """
    :param data: The data to be checked (should be bytes).
    :param crc_ref: The reference CRC value (should be bytes).
    :return: True if the computed CRC matches the reference CRC, False otherwise.
    """
    if len(crc_ref) == CRC_LEN: 
        crc = 0x00  # Initial value
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if (crc & 0x80) != 0:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
        # Compare the calculated CRC with the reference CRC
        return crc == crc_ref[0]
    else:
        return False


# Parse frame data
def parse_frame(frame):

    if len(frame) == FRAME_LEN_SIMPLE:
        head = frame[0:HEAD_LEN]
        length = frame[HEAD_LEN : HEAD_LEN + LEN_LEN]
        Data_Len = struct.unpack('B', length)[0]

        data = frame[HEAD_LEN + LEN_LEN:HEAD_LEN + LEN_LEN + Data_Len]
        crc = frame[HEAD_LEN + LEN_LEN + Data_Len:
                    HEAD_LEN + LEN_LEN + Data_Len + CRC_LEN]

        if len(data) == Data_Len and checksum_crc(head + length + data, crc):
            parsed_data = {
                'Count': struct.unpack('B', data[0:1])[0],
                'AccX': struct.unpack('<f', data[1:5])[0],
                'AccY': struct.unpack('<f', data[5:9])[0],
                'AccZ': struct.unpack('<f', data[9:13])[0],
                'GyroX': struct.unpack('<f', data[13:17])[0],
                'GyroY': struct.unpack('<f', data[17:21])[0],
                'GyroZ': struct.unpack('<f', data[21:25])[0],
                'Pitch': struct.unpack('<h', data[25:27])[0] * 0.0153,
                'Roll': struct.unpack('<h', data[27:29])[0] * 0.0153,
                'Yaw': struct.unpack('<h', data[29:31])[0] * 0.0153,
                'Quat0': struct.unpack('<h', data[31:33])[0] * 0.0003,
                'Quat1': struct.unpack('<h', data[33:35])[0] * 0.0003,
                'Quat2': struct.unpack('<h', data[35:37])[0] * 0.0003,
                'Quat3': struct.unpack('<h', data[37:39])[0] * 0.0003,
                'Temperature': struct.unpack('B', data[39:40])[0] - 50,
                'IMUStatus': struct.unpack('B', data[40:41])[0]
            }
            return parsed_data
        else:
            return None
    else:
        return None


def euler_to_quaternion(roll, pitch, yaw):
    # Convert degrees to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    
    # Compute the quaternion components
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    
    return (qw, qx, qy, qz)
