Restaurant Bot
Overview
The Restaurant Bot package is a ROS-based implementation for autonomous service robots in restaurant environments. It facilitates order delivery, navigation, and table management using ROS.

Features
🛎️ Autonomous navigation to designated tables.

🗣️ Order handling via ROS topics.

🎯 Real-time location tracking using TF and sensor fusion.

📡 ROS-based communication for seamless integration.

Installation
Prerequisites
Ensure ROS Noetic is installed and your catkin workspace is set up:

bash
Copy
Edit
sudo apt update && sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
Clone the Repository
bash
Copy
Edit
cd ~/catkin_ws/src
git clone https://github.com/suryaprakash-V520S-08IKL/restaurant_bot.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
Package Structure
bash
Copy
Edit
restaurant_bot/
│-- launch/
│   ├── restaurant_navigation.launch  # Navigation setup
│   ├── restaurant_simulation.launch  # Simulation setup
│   ├── spawn_turtlebot.launch        # Spawns the robot
│-- maps/
│   ├── restaurant_map.pgm            # Map file
│   ├── restaurant_map.yaml           # Map metadata
│-- scripts/
│   ├── server_robot.py               # Main control script
│-- worlds/
│   ├── restaurant_world.world        # Gazebo world file
│-- CMakeLists.txt                    # CMake build configuration
│-- package.xml                       # Package metadata
│-- README.md                         # Documentation
Usage
1. Start the Simulation
Launch the restaurant environment in Gazebo:

bash
Copy
Edit
roslaunch restaurant_bot restaurant_simulation.launch
2. Enable Navigation
Activate autonomous navigation:

bash
Copy
Edit
roslaunch restaurant_bot restaurant_navigation.launch
3. Spawn the Robot
Manually spawn a TurtleBot:

bash
Copy
Edit
roslaunch restaurant_bot spawn_turtlebot.launch
4. Publish Robot State
Run the robot_state_publisher:

bash
Copy
Edit
rosrun robot_state_publisher robot_state_publisher
5. Start the Server Robot Script
Run the restaurant bot control node:

bash
Copy
Edit
rosrun restaurant_bot server_robot.py
ROS Topics
Topic Name	Message Type	Description
/cmd_vel	geometry_msgs/Twist	Controls robot movement
/robot_pose	geometry_msgs/Pose	Publishes robot position updates
Contributing
Feel free to submit issues or pull requests to enhance the package! 🚀

📧 Contact
📌 Developer: Surya Prakash
📩 Email: suryaprakashsp1999@gmail.com
