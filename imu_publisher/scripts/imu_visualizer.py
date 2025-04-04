#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D  # Import for 3D plotting

# Variables to store current roll, pitch, and yaw
current_roll = 0.0
current_pitch = 0.0
current_yaw = 0.0

# Callback functions for each angle topic
def roll_callback(msg):
    global current_roll
    current_roll = msg.data

def pitch_callback(msg):
    global current_pitch
    current_pitch = msg.data

def yaw_callback(msg):
    global current_yaw
    current_yaw = msg.data

def plot_3d_orientation():
    # Initialize 3D plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    while not rospy.is_shutdown():
        ax.clear()

        # Convert angles from degrees to radians for 3D plotting
        roll_rad = math.radians(current_roll)
        pitch_rad = math.radians(current_pitch)
        yaw_rad = math.radians(current_yaw)

        # Convert spherical to Cartesian coordinates for plotting
        x = math.cos(pitch_rad) * math.cos(yaw_rad)
        y = math.cos(pitch_rad) * math.sin(yaw_rad)
        z = math.sin(pitch_rad)

        # Plot orientation as a quiver (direction vector)
        ax.quiver(0, 0, 0, x, y, z, color='r', length=1, normalize=True)

        # Set limits and labels for the 3D plot
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_title('3D Orientation (Roll, Pitch, Yaw)')
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')

        plt.pause(0.1)  # Pause to update the plot

def imu_visualizer():
    # Initialize ROS node
    rospy.init_node('imu_visualizer', anonymous=True)

    # Subscribe to roll, pitch, and yaw topics
    rospy.Subscriber('/roll', Float32, roll_callback)
    rospy.Subscriber('/pitch', Float32, pitch_callback)
    rospy.Subscriber('/yaw', Float32, yaw_callback)

    # Start the 3D plotting
    plot_3d_orientation()

if __name__ == '__main__':
    try:
        imu_visualizer()
    except rospy.ROSInterruptException:
        pass

