#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import time

# Positions for home, kitchen, and tables
waypoints = {
    "home": [0, 0, 0],
    "kitchen": [2, 0, 0],
    "table1": [4, 2, 0],
    "table2": [4, -2, 0],
    "table3": [6, 0, 0],
}

class ButlerRobot:
    def __init__(self):
        rospy.init_node("butler_navigation")

        # Move Base Action Client for Navigation
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Move Base Server Connected!")

        # Order subscriber
        self.order_sub = rospy.Subscriber("/butler_order", String, self.order_callback)
        self.current_task = None
        self.cancel_requested = False

    def move_to(self, location):
        """Move the robot to a given location using move_base"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoints[location][0]
        goal.target_pose.pose.position.y = waypoints[location][1]
        goal.target_pose.pose.orientation.w = 1.0  # Facing forward

        rospy.loginfo(f"Moving to {location}...")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo(f"Reached {location}!")

    def order_callback(self, msg):
        """Handles incoming orders"""
        table = msg.data
        if table not in waypoints:
            rospy.logwarn(f"Invalid table: {table}")
            return

        self.current_task = table
        self.cancel_requested = False

        # Step 1: Move to Kitchen
        self.move_to("kitchen")

        # Step 2: Wait for Confirmation
        start_time = time.time()
        while (time.time() - start_time) < 10:
            if self.cancel_requested:
                self.move_to("home")
                return
            rospy.sleep(1)

        # Step 3: Move to Table
        self.move_to(table)

        # Step 4: Wait for Confirmation at Table
        start_time = time.time()
        while (time.time() - start_time) < 10:
            if self.cancel_requested:
                self.move_to("kitchen")
                self.move_to("home")
                return
            rospy.sleep(1)

        # Step 5: Return Home
        self.move_to("home")

    def cancel_order(self):
        """Cancel the current task and return home"""
        self.cancel_requested = True
        rospy.logwarn("Order Canceled. Returning Home.")

if __name__ == "__main__":
    butler = ButlerRobot()
    rospy.spin()

