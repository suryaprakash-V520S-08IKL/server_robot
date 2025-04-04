#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <vector>

ros::Publisher marker_pub;

void publishTrajectory(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        ROS_ERROR("Could not open file.");
        return;
    }

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory";
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.g = 1.0;
    marker.color.a = 1.0;

    double x, y;
    while (file >> x >> y) {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        marker.points.push_back(p);
    }

    markers.markers.push_back(marker);
    marker_pub.publish(markers);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_reader_publisher");
    ros::NodeHandle nh;
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 10);

    publishTrajectory("/home/surya/trajectory.csv");

    ros::spin();
    return 0;
}

