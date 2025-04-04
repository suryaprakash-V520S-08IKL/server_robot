#!/usr/bin/env python3

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class ServerRobot:
    def __init__(self):
        rospy.init_node('server_robot', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server.")

        # Define key locations in the restaurant
        self.home = (0.0, 0.0, 0.0)  # x, y, yaw
        self.kitchen = (0.0, -3.0, 0.0)  # Kitchen location
        self.tables = {
            "1": (2.0, 2.0, 1.57),  # Table 1
            "2": (-2.0, -2.0, -1.57), # Table 2
            "3": (3.0, -3.0, 0.0)   # Table 3
        }

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            table = input("Enter table number (1-3) or 'q' to quit: ").strip()
            
            if table.lower() == 'q':
                rospy.loginfo("Shutting down server robot.")
                break

            if table in self.tables:
                self.deliver_order(table)
            else:
                rospy.logwarn("❌ Invalid table number! Please enter 1, 2, or 3.")

    def deliver_order(self, table):
        rospy.loginfo(f"🚀 Navigating to the kitchen to pick up the order for Table {table}...")
        self.navigate_to(self.kitchen)

        # Debug: Confirm reached kitchen
        rospy.loginfo("✅ Reached kitchen. Waiting for food preparation...")
        rospy.sleep(3)  # Simulate waiting for food

        rospy.loginfo(f"🚀 Delivering order to Table {table} at {self.tables[table]}...")
        self.navigate_to(self.tables[table])

        # Debug: Confirm reached table
        rospy.loginfo(f"✅ Order delivered to Table {table}. Waiting for confirmation...")
        rospy.sleep(3)

        rospy.loginfo("🏠 Returning to home position...")
        self.navigate_to(self.home)
        rospy.loginfo("✅ Ready for next order!")

    def navigate_to(self, position):
        rospy.loginfo(f"📍 Sending navigation goal: x={position[0]}, y={position[1]}, yaw={position[2]}")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]

        # Set correct orientation
        quaternion = tf.transformations.quaternion_from_euler(0, 0, position[2])
        rospy.loginfo(f"Quaternion: {quaternion}")  # Debugging quaternion values
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.client.send_goal(goal)
        success = self.client.wait_for_result(rospy.Duration(120))  # Increased timeout

        if success:
            rospy.loginfo(f"✅ Successfully reached x={position[0]}, y={position[1]}")
        else:
            rospy.logwarn(f"⚠️ Navigation to x={position[0]}, y={position[1]} failed! Retrying...")
            self.client.cancel_goal()
            rospy.sleep(2)
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration(60))

if __name__ == '__main__':
    try:
        ServerRobot()
    except rospy.ROSInterruptException:
        rospy.logerr("Server Robot node terminated.")

