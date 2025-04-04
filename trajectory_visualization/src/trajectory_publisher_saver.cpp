#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_visualization/SaveTrajectory.h>
#include <fstream>
#include <vector>

// Publisher for visualization
ros::Publisher marker_pub;

// Store the trajectory data
std::vector<geometry_msgs::PoseStamped> trajectory;

// Callback to save the trajectory to a file
bool saveTrajectoryCallback(trajectory_visualization::SaveTrajectory::Request &req,
                            trajectory_visualization::SaveTrajectory::Response &res) {
    ROS_INFO("Received request to save trajectory to: %s", req.filename.c_str());

    // Open file
    std::ofstream file(req.filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", req.filename.c_str());
        res.success = false;
        res.message = "Failed to open file.";
        return true;
    }

    // Check if trajectory is empty
    if (trajectory.empty()) {
        ROS_WARN("No trajectory data available. File will be empty.");
        res.success = false;
        res.message = "No data to save.";
        return true;
    }

    // Get the current time
    ros::Time now = ros::Time::now();
    int count = 0;

    // Write trajectory data to file
    for (const auto& pose : trajectory) {
        if ((now - pose.header.stamp).toSec() <= req.duration) {
            file << pose.pose.position.x << "," << pose.pose.position.y << "\n";
            count++;
        }
    }

    file.close();

    if (count > 0) {
        ROS_INFO("Successfully saved %d trajectory points.", count);
        res.success = true;
        res.message = "Trajectory saved successfully!";
    } else {
        ROS_WARN("No trajectory points matched the time duration.");
        res.success = false;
        res.message = "No data to save.";
    }

    return true;
}

// Callback to receive robot's pose and store it
void trajectoryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    trajectory.push_back(*msg);

    // Publish trajectory as MarkerArray for RViz visualization
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";  // Change this if needed
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory";
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    for (const auto& pose : trajectory) {
        marker.points.push_back(pose.pose.position);
    }

    markers.markers.push_back(marker);
    marker_pub.publish(markers);
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher_saver");
    ros::NodeHandle nh;

    // Advertise visualization topic
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 10);

    // Advertise service for saving trajectory
    ros::ServiceServer service = nh.advertiseService("save_trajectory", saveTrajectoryCallback);

    // Subscribe to robot pose
    ros::Subscriber sub = nh.subscribe("robot_pose", 10, trajectoryCallback);

    ROS_INFO("Trajectory Publisher & Saver Node Started.");
    ros::spin();
    return 0;
}

