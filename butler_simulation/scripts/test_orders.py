#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import random
import time

rospy.init_node("order_publisher")
pub = rospy.Publisher("/butler_order", String, queue_size=10)
tables = ["table1", "table2", "table3"]

while not rospy.is_shutdown():
    table = random.choice(tables)
    rospy.loginfo(f"New order received for {table}")
    pub.publish(table)
    time.sleep(15)  # Send an order every 15 seconds

