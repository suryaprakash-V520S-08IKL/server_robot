Restaurant Bot - ROS Package
🚀 Autonomous Navigation & Simulation for Restaurant Service Robots

This package enables a restaurant service robot to navigate autonomously using ROS, integrating mapping, localization, and path planning.

📂 Package Structure
bash
Copy
Edit
restaurant_bot/
├── launch/                  # Launch files for navigation & simulation
│   ├── restaurant_navigation.launch
│   ├── restaurant_simulation.launch
│   ├── spawn_turtlebot.launch
├── maps/                    # Predefined restaurant environment maps
│   ├── restaurant_map.pgm
│   ├── restaurant_map.yaml
├── scripts/                 # Python scripts for robot behavior
│   ├── server_robot.py
├── worlds/                  # Gazebo world files
│   ├── restaurant_world.world
├── CMakeLists.txt           # Build configuration
├── package.xml              # Package dependencies and metadata
└── README.md                # This file
🔧 Installation & Setup
Clone the package into your ROS workspace:

bash
Copy
Edit
cd ~/catkin_ws/src
git clone https://github.com/suryaprakash-V520S-08IKL/server_robot.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
Install dependencies:

bash
Copy
Edit
rosdep install --from-paths src --ignore-src -r -y
🚀 Usage
1️⃣ Launch Simulation
Start the restaurant environment in Gazebo:

bash
Copy
Edit
roslaunch restaurant_bot restaurant_simulation.launch
2️⃣ Launch Navigation
Enable autonomous navigation:

bash
Copy
Edit
roslaunch restaurant_bot restaurant_navigation.launch
3️⃣ Spawn the Robot
To manually spawn a TurtleBot in the restaurant world:

bash
Copy
Edit
roslaunch restaurant_bot spawn_turtlebot.launch
4️⃣ Run the Robot State Publisher
To publish the robot’s state:

bash
Copy
Edit
rosrun robot_state_publisher robot_state_publisher
5️⃣ Run the Server Robot Script
Execute the main robot control script:

bash
Copy
Edit
rosrun restaurant_bot server_robot.py
🛠️ Troubleshooting
If the simulation does not start, check Gazebo installation:

bash
Copy
Edit
sudo apt-get install ros-noetic-gazebo-ros
If the robot does not move, verify that the navigation stack is running:

bash
Copy
Edit
rostopic list


📧 Contact
📌 Developer: Surya Prakash
📩 Email: suryaprakashsp1999@gmail.com
