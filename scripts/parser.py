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


def checksum_crc(data, crc_ref):
    """
    :param data:
    :param crc_ref:
    :return: True/False
    """
    if len(crc_ref) == CRC_LEN: 
        crc = 0x00  # Initial value
        for byte in data:
            crc ^= struct.unpack('B', byte)[0]
            for _ in range(8):
                if (crc & 0x80) != 0:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc == struct.unpack('B', crc_ref[0])[0]
    else:
        return False


# Parse frame data
def parse_frame(frame):

    if len(frame) == FRAME_LEN_OLD:
        head = frame[0:HEAD_LEN]
        dom = frame[HEAD_LEN:HEAD_LEN + DOM_LEN]
        cmd = frame[HEAD_LEN + DOM_LEN:HEAD_LEN + DOM_LEN + CMD_LEN]
        length = frame[HEAD_LEN + DOM_LEN + CMD_LEN:HEAD_LEN + DOM_LEN + CMD_LEN + LEN_LEN]

        # cmd_value = struct.unpack('B', cmd)[0]
        Data_Len = struct.unpack('B', length)[0]

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
                'Quat0': None,
                'Quat1': None,
                'Quat2': None,
                'Quat3': None,
                'Temperature': struct.unpack('<h', data[45:47])[0] * 0.1,
                'IMUStatus': struct.unpack('B', data[47:48])[0],
                'GyroBiasX': struct.unpack('<h', data[48:50])[0] * 0.0001,
                'GyroBiasY': struct.unpack('<h', data[50:52])[0] * 0.0001,
                'GyroBiasZ': struct.unpack('<h', data[52:54])[0] * 0.0001,
                'GyroStaticBiasX': struct.unpack('<h', data[54:56])[0] * 0.0001,
                'GyroStaticBiasY': struct.unpack('<h', data[56:58])[0] * 0.0001,
                'GyroStaticBiasZ': struct.unpack('<h', data[58:60])[0] * 0.0001
            }
            return parsed_data
        else:
            return None
        
    elif len(frame) == FRAME_LEN_BINARY:
        head = frame[0:HEAD_LEN]
        dom = frame[HEAD_LEN:HEAD_LEN + DOM_LEN]
        cmd = frame[HEAD_LEN + DOM_LEN:HEAD_LEN + DOM_LEN + CMD_LEN]
        length = frame[HEAD_LEN + DOM_LEN + CMD_LEN:HEAD_LEN + DOM_LEN + CMD_LEN + LEN_LEN]

        # cmd_value = struct.unpack('B', cmd)[0]
        Data_Len = struct.unpack('B', length)[0]

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