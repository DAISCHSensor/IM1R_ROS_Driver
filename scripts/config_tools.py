import sys
import serial
import time

# Default configuration constants
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 115200
BAUD_FREQ_CONFIG = {
    '09': (115200, 10), '0A': (230400, 10), '0B': (460800, 10), '0C': (921600, 10),
    '11': (115200, 20), '12': (230400, 20), '13': (460800, 20), '14': (921600, 20),
    '19': (115200, 50), '1A': (230400, 50), '1B': (460800, 50), '1C': (921600, 50),
    '21': (115200, 100), '22': (230400, 100), '23': (460800, 100), '24': (921600, 100),
    '29': (115200, 250), '2A': (230400, 250), '2B': (460800, 250), '2C': (921600, 250),
    '31': (115200, 500), '32': (230400, 500), '33': (460800, 500), '34': (921600, 500), 
    '39': (115200, 1000), '3A': (230400, 1000), '3B': (460800, 1000), '3C': (921600, 1000)
}
GYRO_CONFIG = {
    '00': (274, 125), '01': (274, 250), '02': (274, 500), '03': (274, 1000),
    '10': (212, 125), '11': (212, 250), '12': (212, 500), '13': (212, 1000),
    '20': (150, 125), '21': (150, 250), '22': (150, 500), '23': (150, 1000),
    '30': (390, 125), '31': (390, 250), '32': (390, 500), '33': (390, 1000),
    '40': (99, 125), '41': (99, 250), '42': (99, 500), '43': (99, 1000),
    '50': (50, 125), '51': (50, 250), '52': (50, 500), '53': (50, 1000),
    '60': (25, 125), '61': (25, 250), '62': (25, 500), '63': (25, 1000),
    '70': (12.6, 125), '71': (12.6, 250), '72': (12.6, 500), '73': (12.6, 1000)
}
ACC_CONFIG = {
    '00': (417, 2), '01': (417, 4), '02': (417, 8), '03': (417, 16),
    '10': (167, 2), '11': (167, 4), '12': (167, 8), '13': (167, 16),
    '20': (83.4, 2), '21': (83.4, 4), '22': (83.4, 8), '23': (83.4, 16),
    '30': (37, 2), '31': (37, 4), '32': (37, 8), '33': (37, 16),
    '40': (16.7, 2), '41': (16.7, 4), '42': (16.7, 8), '43': (16.7, 16),
    '50': (8.3, 2), '51': (8.3, 4), '52': (8.3, 8), '53': (8.3, 16),
    '60': (4.2, 2), '61': (4.2, 4), '62': (4.2, 8), '63': (4.2, 16),
    '70': (2.1, 2), '71': (2.1, 4), '72': (2.1, 8), '73': (2.1, 16)
}

# A class to handle real-time communication via a serial port
class RealTimeCOM:
    def __init__(self, port, rate=DEFAULT_BAUDRATE, timeout=1):
        self.port = port
        self.rate = rate
        self.timeout = timeout
        self.com = None

    def open(self, stdscr):
        try:
            self.com = serial.Serial(self.port, self.rate, timeout=self.timeout)
            stdscr.addstr("Successfully opened the SerialPort: %s\n" % self.port)
            stdscr.refresh()
            return True
        except serial.SerialException as e:
            stdscr.addstr('%s\n' % e)
            stdscr.refresh()
            return False

    def close(self, stdscr):
        try:
            if self.com and self.com.is_open:
                self.com.close()
                stdscr.addstr('SerialPort closed.\n')
                stdscr.refresh()
            return True
        except Exception as e:
            stdscr.addstr('%s\n' % e)
            stdscr.refresh()
            return False

    def get_data(self):
        return self.com.read_all().decode().strip() if self.com else ""

    def send_data(self, message):
        self.com.write(message.encode())

# Initialize the serial port from command-line arguments or use the default port
def initialize_serial_port():
    try:
        port = sys.argv[1]
    except IndexError:
        port = DEFAULT_PORT
        print('Default port used: %s' % DEFAULT_PORT)
    return port

def send_and_receive(serial_instance, message, expected_response, response_dic, timeout=0.5, max_retries=3):
    """
    Send a message and wait for an expected response with retries.

    :param serial_instance: Instance of RealTimeSerial
    :param message: Message to send
    :param expected_response: Expected response header
    :param response_dict: Dictionary of expected response values
    :param timeout: Wait time between retries
    :param max_retries: Maximum number of retries
    :return: Response value if successful, None otherwise
    """
    retries = 0
    while retries < max_retries:
        serial_instance.send_data(message)
        print("Sended: %s" % message)
        time.sleep(timeout)
        response = serial_instance.get_data()
        print("Received: %s" % response)
        if expected_response == "$DSIMC,ACK*50":
            return response.startswith(expected_response)
        else:
            if response.startswith(expected_response):
                parts = response.split(',')
                if len(parts) >= 4:
                    response_rel = parts[3].split('*')[0]
                    if response_rel in response_dic.keys():
                        return response_dic[response_rel]
        retries += 1
        print("Retrying... (%s/%s)" % (retries, max_retries))
    print("Error: Failed to receive the expected response.")
    return None

def get_baudrate_and_frequency(serial_instance):
    """
    Get the current baud rate and frequency from the device.

    :param serial_instance: Instance of RealTimeSerial
    :return: Tuple of baud rate and frequency
    """
    message = "$DSIMC,GET,00\r\n"
    expected_response = "$DSIMC,ACK,00"
    ack_dic = BAUD_FREQ_CONFIG
    return send_and_receive(serial_instance, message, expected_response, ack_dic)
    
def get_gyro_status(serial_instance):
    """
    Get the current gyroscope configuration.

    :param serial_instance: Instance of RealTimeSerial
    :return: Tuple of gyroscope frequency and range
    """
    message = "$DSIMC,GET,01\r\n"
    expected_response = "$DSIMC,ACK,01"
    ack_dic = GYRO_CONFIG
    return send_and_receive(serial_instance, message, expected_response, ack_dic)

def get_acc_status(serial_instance):
    """
    Get the current accelerometer configuration.

    :param serial_instance: Instance of RealTimeSerial
    :return: Tuple of accelerometer frequency and range
    """
    message = "$DSIMC,GET,02\r\n"
    expected_response = "$DSIMC,ACK,02"
    ack_dic = ACC_CONFIG
    return send_and_receive(serial_instance, message, expected_response, ack_dic)

def set_baudrate_and_frequency(serial_instance, baud_freq_setting):
    """
    Set the baud rate and frequency on the device.

    :param serial_instance: Instance of RealTimeSerial
    :param baud_freq_setting: Tuple of desired baud rate and frequency
    :return: True if successful, False otherwise
    """
    set_key = find_key_by_value(BAUD_FREQ_CONFIG, baud_freq_setting)
    if set_key is not None:
        message = "$DSIMC,SET,00,%s\r\n" % set_key
        expected_response = "$DSIMC,ACK,00"
        ack_dic = BAUD_FREQ_CONFIG
        return baud_freq_setting == send_and_receive(serial_instance, message, expected_response, ack_dic)
    else:
        return False

def set_gyro_status(serial_instance, gyro_setting):
    """
    Set the gyroscope configuration on the device.

    :param serial_instance: Instance of RealTimeSerial
    :param gyro_setting: Tuple of desired gyroscope frequency and range
    :return: True if successful, False otherwise
    """
    set_key = find_key_by_value(GYRO_CONFIG, gyro_setting)
    if set_key is not None:
        message = "$DSIMC,SET,01,%s\r\n" % set_key
        expected_response = "$DSIMC,ACK,01"
        ack_dic = GYRO_CONFIG
        return gyro_setting == send_and_receive(serial_instance, message, expected_response, ack_dic)
    else:
        return False
    
def set_acc_status(serial_instance, acc_setting):
    """
    Set the accelerometer configuration on the device.

    :param serial_instance: Instance of RealTimeSerial
    :param acc_setting: Tuple of desired accelerometer frequency and range
    :return: True if successful, False otherwise
    """
    set_key = find_key_by_value(ACC_CONFIG, acc_setting)
    if set_key is not None:
        message = "$DSIMC,SET,02,%s\r\n" % set_key
        expected_response = "$DSIMC,ACK,02"
        ack_dic = ACC_CONFIG
        return acc_setting == send_and_receive(serial_instance, message, expected_response, ack_dic)
    else:
        return False
    
def restore_factory_settings(serial_instance):
    """
    Restore the device to its factory settings.

    :param serial_instance: Instance of RealTimeSerial
    """
    message = "$DSIMC,RST\r\n"
    expected_response = "$DSIMC,ACK*50"
    ack_dic = {}
    if send_and_receive(serial_instance, message, expected_response, ack_dic):
        print('Restore factory settings successfully ')
    else:
        print('Failed to restore factory settings')

def get_submenu_list(key):
    """
    Generate a list of submenu options based on the configuration key.

    :param key: Configuration key ('BAUDRATE', 'FREQUENCY', 'GYRO_FREQ', 'GYRO_RANGE', 'ACC_FREQ', 'ACC_RANGE')
    :return: Tuple of submenu list and corresponding configuration values
    """
    submenu_list = []
    config_value_list = []
    items_set = set()

    if key == 'BAUDRATE':
        for config in BAUD_FREQ_CONFIG.values():
            baudrate, _ = config
            items_set.add(baudrate)

    elif key == 'FREQUENCY':
        for config in BAUD_FREQ_CONFIG.values():
            _, frequency = config
            items_set.add(frequency)

    elif key == 'GYRO_FREQ':
        for config in GYRO_CONFIG.values():
            frequency, _ = config
            items_set.add(frequency)

    elif key == 'GYRO_RANGE':
        for config in GYRO_CONFIG.values():
            _, range = config
            items_set.add(range)

    elif key == 'ACC_FREQ':
        for config in ACC_CONFIG.values():
            frequency, _ = config
            items_set.add(frequency)

    elif key == 'ACC_RANGE':
        for config in ACC_CONFIG.values():
            _, range = config
            items_set.add(range)
    else:
        pass

    items_list = sorted(list(items_set))
    for i, item in enumerate(items_list):
        submenu_list.append('%s. %s' % (i+1, item))
        config_value_list.append(item)
    return submenu_list, config_value_list
    
def find_key_by_value(config_dict, value):
    """
    Find the key corresponding to a given value in a dictionary.

    :param config_dict: Dictionary to search
    :param value: Value to find
    :return: Key if found, None otherwise
    """
    for key, val in config_dict.items():
        if val == value:
            return key
    return None
