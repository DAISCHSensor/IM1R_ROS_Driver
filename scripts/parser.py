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
FRAME_LEN_BINARY = 96


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
    
    if len(frame) == FRAME_LEN_BINARY:
        head = frame[0:HEAD_LEN]
        dom = frame[HEAD_LEN:HEAD_LEN + DOM_LEN]
        cmd = frame[HEAD_LEN + DOM_LEN:HEAD_LEN + DOM_LEN + CMD_LEN]
        length = frame[HEAD_LEN + DOM_LEN + CMD_LEN:HEAD_LEN + DOM_LEN + CMD_LEN + LEN_LEN]

        # cmd_value = struct.unpack('B', cmd)[0]
        # Data_Len = struct.unpack('B', length)[0]
        Data_Len = 88

        data = frame[HEAD_LEN + DOM_LEN + CMD_LEN + LEN_LEN:HEAD_LEN + DOM_LEN + CMD_LEN + LEN_LEN + Data_Len]
        crc = frame[HEAD_LEN + DOM_LEN + CMD_LEN + LEN_LEN + Data_Len:
                    HEAD_LEN + DOM_LEN + CMD_LEN + LEN_LEN + Data_Len + CRC_LEN]

        if len(data) == Data_Len and checksum_crc(head + dom + cmd + length + data, crc):
            parsed_data = {
                'Count': struct.unpack('B', data[0:1])[0],
                'Timestamp': struct.unpack('<Q', data[1:9])[0],
                'AccX': struct.unpack('<f', data[9:13])[0],
                'AccY': struct.unpack('<f', data[13:17])[0],
                'AccZ': struct.unpack('<f', data[17:21])[0],
                'GyroX': struct.unpack('<f', data[21:25])[0],
                'GyroY': struct.unpack('<f', data[25:29])[0],
                'GyroZ': struct.unpack('<f', data[29:33])[0],
                'Pitch': struct.unpack('<f', data[33:37])[0],
                'Roll': struct.unpack('<f', data[37:41])[0],
                'Yaw': struct.unpack('<f', data[41:45])[0],
                'Quat0': struct.unpack('<f', data[45:49])[0],
                'Quat1': struct.unpack('<f', data[49:53])[0],
                'Quat2': struct.unpack('<f', data[53:57])[0],
                'Quat3': struct.unpack('<f', data[57:61])[0],
                'Temperature': struct.unpack('<h', data[61:63])[0] * 0.1,
                'IMUStatus': struct.unpack('B', data[63:64])[0],
                'GyroBiasX': None,
                'GyroBiasY': None,
                'GyroBiasZ': None,
                'GyroStaticBiasX': None,
                'GyroStaticBiasY': None,
                'GyroStaticBiasZ': None
            }
            parsed_data_raw = {
                'Count': struct.unpack('B', data[0:1])[0],
                'Timestamp': struct.unpack('<Q', data[1:9])[0],
                'AccX': struct.unpack('<f', data[64:68])[0],
                'AccY': struct.unpack('<f', data[68:72])[0],
                'AccZ': struct.unpack('<f', data[72:76])[0],
                'GyroX': struct.unpack('<f', data[76:80])[0],
                'GyroY': struct.unpack('<f', data[80:84])[0],
                'GyroZ': struct.unpack('<f', data[84:88])[0],
                'Pitch': struct.unpack('<f', data[33:37])[0],
                'Roll': struct.unpack('<f', data[37:41])[0],
                'Yaw': struct.unpack('<f', data[41:45])[0],
                'Quat0': struct.unpack('<f', data[45:49])[0],
                'Quat1': struct.unpack('<f', data[49:53])[0],
                'Quat2': struct.unpack('<f', data[53:57])[0],
                'Quat3': struct.unpack('<f', data[57:61])[0],
                'Temperature': struct.unpack('<h', data[61:63])[0] * 0.1,
                'IMUStatus': struct.unpack('B', data[63:64])[0],
                'GyroBiasX': None,
                'GyroBiasY': None,
                'GyroBiasZ': None,
                'GyroStaticBiasX': None,
                'GyroStaticBiasY': None,
                'GyroStaticBiasZ': None
            }
            return parsed_data, parsed_data_raw
        else:
            return None, None
        
    else:
        return None, None


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
