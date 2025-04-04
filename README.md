# 🏠 Restaurant Bot

## Overview  
The **Restaurant Bot** package is a ROS-based implementation for autonomous service robots in restaurant environments. It facilitates **order delivery**, **navigation**, and **table management** using ROS.

## Features  
✅ **Autonomous navigation** to tables  
✅ **Order processing** via ROS topics  
✅ **Real-time TF tracking** for positioning  
✅ **Seamless ROS communication**

---

## 📥 Installation  

### **Prerequisites**  
Ensure **ROS Noetic** is installed and set up:  
```bash
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
git clone https://github.com/suryaprakash-V520S-08IKL/server_robot.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
📂 Package Structure
bash
Copy
Edit
server_robot/
│-- launch/
│   ├── navigation.launch        # Navigation setup
│-- maps/
│   ├── restaurant_map.pgm       # Map file
│   ├── restaurant_map.yaml      # Map metadata
│-- scripts/
│   ├── server_robot.py          # Main control script
│-- worlds/
│   ├── restaurant_world.world   # Gazebo world file
│-- CMakeLists.txt               # CMake build configuration
│-- package.xml                  # Package metadata
│-- README.md                    # Documentation
🚀 Usage
1️⃣ Start the Simulation
bash
Copy
Edit
roslaunch server_robot restaurant_simulation.launch
2️⃣ Enable Navigation
bash
Copy
Edit
roslaunch server_robot navigation.launch
3️⃣ Publish Robot State
bash
Copy
Edit
rosrun robot_state_publisher robot_state_publisher
4️⃣ Start the Server Robot
bash
Copy
Edit
rosrun restaurant_bot server_robot.py
📡 ROS Topics
Topic Name	Message Type	Description
/cmd_vel	geometry_msgs/Twist	Controls robot movement
/robot_pose	geometry_msgs/Pose	Publishes robot position updates
🤝 Contributing
Feel free to submit issues or pull requests to improve the package! 🚀

📧 Developer: Surya Prakash
📩 Email: suryaprakashsp1999@gmail.com
