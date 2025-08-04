#include "path_visualizer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>


PathVisualizer::PathVisualizer(ros::NodeHandle& n) {
    path_pub_ = n.advertise<nav_msgs::Path>("trajectry", 1);
    path_.header.frame_id = "map";
}

void PathVisualizer::publishPose(const Pose& pose) {
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "map";
    ps.pose.position.x = pose.x;
    ps.pose.position.y = pose.y;
    ps.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0,0,pose.theta);
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();

    path_.poses.push_back(ps);
    path_pub_.publish(path_);
}