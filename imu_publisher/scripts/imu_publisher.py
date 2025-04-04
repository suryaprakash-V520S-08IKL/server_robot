#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import serial
import time
import threading
import matplotlib.pyplot as plt
from collections import deque
import math

# Setup serial port
SERIAL_PORT = '/dev/ttyUSB0'  # Change this to your Arduino's serial port
BAUD_RATE = 115200

# Constants for plotting
MAX_LENGTH = 100  # Maximum number of points to display in the plot

# Queues to hold data for plotting
roll_queue = deque(maxlen=MAX_LENGTH)
pitch_queue = deque(maxlen=MAX_LENGTH)
yaw_queue = deque(maxlen=MAX_LENGTH)
force_queue = deque(maxlen=MAX_LENGTH)  # Queue for force data

def plot_data():
    plt.ion()  # Interactive mode on
    fig, ax = plt.subplots(4, 1, figsize=(10, 10))  # Changed to 4 plots

    while not rospy.is_shutdown():
        ax[0].clear()
        ax[0].plot(roll_queue, label='Roll', color='r')
        ax[0].set_title('Roll Data')
        ax[0].set_ylim(-180, 180)
        ax[0].legend()

        ax[1].clear()
        ax[1].plot(pitch_queue, label='Pitch', color='g')
        ax[1].set_title('Pitch Data')
        ax[1].set_ylim(-180, 180)
        ax[1].legend()

        ax[2].clear()
        ax[2].plot(yaw_queue, label='Yaw', color='b')
        ax[2].set_title('Yaw Data')
        ax[2].set_ylim(-180, 180)
        ax[2].legend()

        ax[3].clear()
        ax[3].plot(force_queue, label='Force (N)', color='m')  # Added force plot
        ax[3].set_title('Force Sensor Data')
        ax[3].legend()

        plt.pause(0.1)  # Pause to update the plot

def imu_publisher():
    # Initialize ROS node
    rospy.init_node('imu_publisher', anonymous=True)

    # Define ROS publishers for roll, pitch, yaw, and force
    roll_pub = rospy.Publisher('/roll', Float32, queue_size=10)
    pitch_pub = rospy.Publisher('/pitch', Float32, queue_size=10)
    yaw_pub = rospy.Publisher('/yaw', Float32, queue_size=10)
    force_pub = rospy.Publisher('/force', Float32, queue_size=10)  # Publisher for force data

    # Set loop rate in Hz
    rate = rospy.Rate(10)  # 10Hz

    # Setup the serial connection
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
        rospy.loginfo("Serial port opened: %s" % SERIAL_PORT)
    except serial.SerialException as e:
        rospy.logerr("Error opening serial port: %s" % e)
        return

    # Start the plotting thread
    plotting_thread = threading.Thread(target=plot_data)
    plotting_thread.start()

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            # Read a line from the serial port
            line = ser.readline().decode('utf-8').strip()
            try:
                # Assuming data is formatted as "roll,pitch,yaw,force"
                roll, pitch, yaw, force = map(float, line.split(','))

                # Add data to the queues for plotting
                roll_queue.append(roll)

                # Check if pitch is a valid number before appending and publishing
                if not math.isnan(pitch):
                    pitch_queue.append(pitch)
                    pitch_msg = Float32()
                    pitch_msg.data = pitch
                    pitch_pub.publish(pitch_msg)
                else:
                    rospy.logwarn("Invalid pitch value received: %s", line)

                yaw_queue.append(yaw)
                force_queue.append(force)  # Add force data to the queue

                # Create Float32 messages for roll, yaw, and force
                roll_msg = Float32()
                yaw_msg = Float32()
                force_msg = Float32()

                roll_msg.data = roll
                yaw_msg.data = yaw
                force_msg.data = force  # Set force data

                # Publish roll, yaw, and force
                roll_pub.publish(roll_msg)
                yaw_pub.publish(yaw_msg)
                force_pub.publish(force_msg)  # Publish force data

                # Log the data to console
                rospy.loginfo("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Force: %.2f N", roll, pitch, yaw, force)

            except ValueError:
                rospy.logwarn("Invalid data received: %s" % line)

        # Sleep to maintain the loop rate
        rate.sleep()

    ser.close()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
