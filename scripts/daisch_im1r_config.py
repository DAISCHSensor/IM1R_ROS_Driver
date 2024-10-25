#!/usr/bin/env python3

import rospy
import curses
import sys
import serial
from datetime import datetime
from config_tools import *

# Class to handle printing messages to the curses window with optional timestamps
class CursesPrinter:
    def __init__(self, stdscr, start_line=0, add_timestamp=True):
        self.stdscr = stdscr
        self.start_line = start_line
        self.log_buffer = ['', '', '', '', '']  # Buffer to hold the last 5 lines of log
        self.add_timestamp = add_timestamp

    # Write a message to the curses window
    def write(self, message):
        if message.endswith('\n'):
            message = message[:-1]
        if len(message) == 0:
            return
        self.log_buffer = self.log_buffer[1:]  # Shift the log lines buffer up
        if self.add_timestamp:
            current_time = datetime.now().strftime('%H:%M:%S')
            self.log_buffer.append("[%s] %s" % (current_time, message))
        else:
            self.log_buffer.append(message)
        self.print_lines()  # Display the updated log lines

    def flush(self):
        pass

    def print_lines(self):
        height, width = self.stdscr.getmaxyx()
        for i, line in enumerate(self.log_buffer):
            print_line(self.stdscr, self.start_line + i, line[:width-1])

# Class to store the IMU status
class IM1RStatus:
    def __init__(self):
        self.baudrate = None
        self.frequency = None
        self.GyroRange = None
        self.GyroFilterFreq = None
        self.AcceRange = None
        self.AcceFilterFreq = None

# Class to manage the main menu interface and interaction with the IMU
class MainMenu:
    def __init__(self, stdscr, serial_com, start_line=0, end_line=12):
        self.stdscr = stdscr
        self.serial_com = serial_com
        self.start_line = start_line
        self.end_line = end_line
        self.im1r_status = IM1RStatus()

    # Clear the menu area on the screen
    def clear_menu(self):
        for idx in range(self.start_line, self.end_line):
            self.stdscr.move(idx, 0)
            self.stdscr.clrtoeol()
        self.stdscr.refresh()

    # Display the main menu
    def display_main_menu(self):
        self.clear_menu()
        self.stdscr.addstr(self.start_line, 0, "****************************************")
        print_line(self.stdscr, self.start_line+1, 'IM1R STATUS:')
        print_line(self.stdscr, self.start_line+2, '1. Baudrate: %s' % self.im1r_status.baudrate)
        print_line(self.stdscr, self.start_line+3, '2. Frequency: %s (Hz)' % self.im1r_status.frequency)
        print_line(self.stdscr, self.start_line+4, '3. Gyro Range: %s (deg/s)' % self.im1r_status.GyroRange)
        print_line(self.stdscr, self.start_line+5, '4. Gyro Filter Frequency: %s (Hz)' % self.im1r_status.GyroFilterFreq)
        print_line(self.stdscr, self.start_line+6, '5. Acc Range: %s (g)' % self.im1r_status.AcceRange)
        print_line(self.stdscr, self.start_line+7, '6. Acc Filter Frequency: %s (Hz)' % self.im1r_status.AcceFilterFreq)
        print_line(self.stdscr, self.start_line+8, '7. Restore factory settings')
        print_line(self.stdscr, self.end_line-2, '0. Exit program')
        self.stdscr.addstr(self.end_line-1, 0, "****************************************")
        self.stdscr.refresh()
        self.get_mainmenu_choice()

    # Display a submenu for configuring a specific parameter
    def display_sub_menu(self, submenu_key):
        units = {'BAUDRATE': '', 'FREQUENCY': 'Hz', 'GYRO_RANGE': 'deg/s', 'GYRO_FREQ': 'Hz', 
             'ACC_RANGE': 'g', 'ACC_FREQ': 'Hz', }
        options, values = get_submenu_list(submenu_key)
        self.clear_menu()
        self.stdscr.addstr(self.start_line, 0, "****************************************")
        print_line(self.stdscr, self.start_line+1, '%s [%s]' % (submenu_key, units[submenu_key]))
        for i, option in enumerate(options):
            print_line(self.stdscr, self.start_line+i+2, option)
        print_line(self.stdscr, self.end_line-2, '0. Return to Main Menu')
        self.stdscr.addstr(self.end_line-1, 0, "****************************************")
        self.stdscr.refresh()
        self.get_submenu_choice(submenu_key, values)

    # Handle the user's choice from the main menu
    def get_mainmenu_choice(self):
        while not rospy.is_shutdown():
            print_line(self.stdscr, self.end_line, "Enter option to configure: ")
            choice = self.stdscr.getstr().decode('utf-8')
            menu_map = {
                '1': 'BAUDRATE',
                '2': 'FREQUENCY',
                '3': 'GYRO_RANGE',
                '4': 'GYRO_FREQ',
                '5': 'ACC_RANGE',
                '6': 'ACC_FREQ',
                '7': 'RESTORE'
            }
            if choice in menu_map:
                if choice == '7':
                    restore_factory_settings(self.serial_com)
                    self.update_im1r_status()
                    self.display_main_menu()
                else:
                    self.display_sub_menu(menu_map[choice])
                break
            elif choice == '0':
                print("Exit program... Please waite")
                break
            else:
                print("Invalid choice. Please try again")
    
    # Handle the user's choice from a submenu
    def get_submenu_choice(self, submenu_key, value_list):
        while not rospy.is_shutdown():
            print_line(self.stdscr, self.end_line, "Select configuration option: ")
            choice = self.stdscr.getstr().decode('utf-8')
            if choice.isdigit() and 1 <= int(choice) <= len(value_list):
                self.config_im1r(submenu_key, value_list[int(choice) - 1])
                self.update_im1r_status()
                self.display_main_menu()
                break
            elif choice == '0':
                self.display_main_menu()
                break
            else:
                print("Invalid choice. Please try again")

    # Update the IMU status from the device
    def update_im1r_status(self):
        try:
            baudrate, frequency = get_baudrate_and_frequency(self.serial_com)
            GyroFilterFreq, GyroRange = get_gyro_status(self.serial_com)
            AcceFilterFreq, AcceRange = get_acc_status(self.serial_com)
            self.im1r_status.baudrate = baudrate
            self.im1r_status.frequency = frequency
            self.im1r_status.GyroFilterFreq = GyroFilterFreq
            self.im1r_status.GyroRange = GyroRange
            self.im1r_status.AcceFilterFreq = AcceFilterFreq
            self.im1r_status.AcceRange = AcceRange
        except serial.SerialException as e:
            print(e)

    # Configure a specific parameter on the IMU
    def config_im1r(self, config_key, choice):
        try:
            if config_key == 'BAUDRATE':
                if self.im1r_status.frequency is not None:
                    baud_freq_setting = (choice, self.im1r_status.frequency)
                else:
                    baud_freq_setting = (choice, 100)
                if set_baudrate_and_frequency(self.serial_com, baud_freq_setting):
                    print('baudrate and frequency set successfully ')
                else:
                    print('Failed to set baudrate and frequency')

            elif config_key == 'FREQUENCY':
                if self.im1r_status.baudrate is not None:
                    baud_freq_setting = (self.im1r_status.baudrate, choice)
                else:
                    baud_freq_setting = (115200, choice)
                if set_baudrate_and_frequency(self.serial_com, baud_freq_setting):
                    print('baudrate and frequency set successfully ')
                else:
                    print('Failed to set baudrate and frequency')

            elif config_key == 'GYRO_FREQ':
                if self.im1r_status.GyroRange is not None:
                    gyro_setting = (choice, self.im1r_status.GyroRange)
                else:
                    gyro_setting = (choice, 250)
                if set_gyro_status(self.serial_com, gyro_setting):
                    print('gyro status set successfully ')
                else:
                    print('Failed to set gyro status')

            elif config_key == 'GYRO_RANGE':
                if self.im1r_status.GyroFilterFreq is not None:
                    gyro_setting = (self.im1r_status.GyroFilterFreq, choice)
                else:
                    gyro_setting = (50, choice)
                if set_gyro_status(self.serial_com, gyro_setting):
                    print('gyro status set successfully ')
                else:
                    print('Failed to set gyro status')

            elif config_key == 'ACC_FREQ':
                if self.im1r_status.AcceRange is not None:
                    acc_setting = (choice, self.im1r_status.AcceRange)
                else:
                    acc_setting = (choice, 8)
                if set_acc_status(self.serial_com, acc_setting):
                    print('acc status set successfully ')
                else:
                    print('Failed to set acc status')

            elif config_key == 'ACC_RANGE':
                if self.im1r_status.AcceFilterFreq is not None:
                    acc_setting = (self.im1r_status.AcceFilterFreq, choice)
                else:
                    acc_setting = (16.7, choice)
                if set_acc_status(self.serial_com, acc_setting):
                    print('acc status set successfully ')
                else:
                    print('Failed to set acc status')
        except serial.SerialException as e:
            print(e)

# Function to print a string at a specific line in the curses window
def print_line(stdscr, idx, text):
    stdscr.move(idx, 0)
    stdscr.clrtoeol()
    stdscr.addstr(idx, 0, text)
    stdscr.refresh()

# Print the text with a newline at the current cursor position
def stdscr_addstr(stdscr, text):
    stdscr.addstr("%s\n" % text)
    stdscr.refresh()

# Main function to initialize the node and start the curses UI
def main(stdscr):
    rospy.init_node('daisch_im1r_config')
    stdscr.clear()
    serial_port = initialize_serial_port()
    serial_baudrate = DEFAULT_BAUDRATE
    serial_com = None

    try:
        stdscr_addstr(stdscr, "Try opening the SerialPort: %s" % serial_port)
        serial_com = RealTimeCOM(serial_port, serial_baudrate, timeout=1)
        if serial_com.open(stdscr):
            stdscr.clear()
            stdscr.addstr(14, 0, "****************************************")
            curses_output = CursesPrinter(stdscr, 15)
            sys.stdout = curses_output

            imu_menu = MainMenu(stdscr, serial_com, start_line=0, end_line=12)
            imu_menu.update_im1r_status()
            imu_menu.display_main_menu()
        else:
            stdscr_addstr(stdscr, "Try reconnecting the USB device and running the program again")
            return
    except serial.SerialException as e:
        stdscr.clear()
        stdscr_addstr(stdscr, "Serial error: %s" % e)
    except Exception as e:
        stdscr.clear()
        stdscr_addstr(stdscr, "Unexpected error: %s" % e)
    else:
        stdscr.clear()
    finally:
        if serial_com is not None:
            serial_com.close(stdscr)
        stdscr_addstr(stdscr, "Press any key to exit.")
        stdscr.getch()

# Start the curses application
if __name__ == "__main__":
    curses.wrapper(main)
