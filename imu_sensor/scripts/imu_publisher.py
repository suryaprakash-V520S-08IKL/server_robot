#!/usr/bin/env python3

import rospy
from smbus2 import SMBus
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

# MPU6050 I2C address
MPU_ADDR = 0x68

# Registers for accelerometer data
ACCEL_XOUT_H = 0x3B

# MPU6050 Initialization
def initialize_mpu(bus):
    bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # Wake up the MPU6050
    rospy.loginfo("MPU6050 initialized")

# Read raw accelerometer data
def read_accelerometer(bus):
    data = bus.read_i2c_block_data(MPU_ADDR, ACCEL_XOUT_H, 6)
    accel_x = (data[0] << 8 | data[1])
    accel_y = (data[2] << 8 | data[3])
    accel_z = (data[4] << 8 | data[5])

    # Convert to signed values
    accel_x = accel_x - 65536 if accel_x > 32767 else accel_x
    accel_y = accel_y - 65536 if accel_y > 32767 else accel_y
    accel_z = accel_z - 65536 if accel_z > 32767 else accel_z

    # Scale to g's (sensitivity is 16384 LSB/g)
    accel_x /= 16384.0
    accel_y /= 16384.0
    accel_z /= 16384.0

    return accel_x, accel_y, accel_z

def publisher():
    rospy.init_node('mpu6050_publisher', anonymous=True)

    # Create publishers for acceleration, velocity, and position
    accel_pub = rospy.Publisher('/imu/acceleration', Vector3, queue_size=10)
    velocity_pub = rospy.Publisher('/imu/velocity', Vector3, queue_size=10)
    position_pub = rospy.Publisher('/imu/position', Vector3, queue_size=10)

    bus = SMBus(1)
    initialize_mpu(bus)

    rate = rospy.Rate(10)  # 10 Hz
    velocity = Vector3()
    position = Vector3()
    prev_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        # Get acceleration data
        accel_x, accel_y, accel_z = read_accelerometer(bus)
        accel_msg = Vector3(accel_x * 9.81, accel_y * 9.81, accel_z * 9.81)

        # Publish acceleration
        accel_pub.publish(accel_msg)

        # Time step for integration
        curr_time = rospy.Time.now().to_sec()
        delta_t = curr_time - prev_time
        prev_time = curr_time

        # Integrate acceleration to get velocity
        velocity.x += accel_msg.x * delta_t
        velocity.y += accel_msg.y * delta_t
        velocity.z += accel_msg.z * delta_t

        # Publish velocity
        velocity_pub.publish(velocity)

        # Integrate velocity to get position
        position.x += velocity.x * delta_t
        position.y += velocity.y * delta_t
        position.z += velocity.z * delta_t

        # Publish position
        position_pub.publish(position)

        rospy.loginfo(f"Acceleration: {accel_msg}, Velocity: {velocity}, Position: {position}")
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

