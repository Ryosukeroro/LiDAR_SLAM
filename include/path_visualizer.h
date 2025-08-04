#ifndef PATH_VISUALIZER_H
#define PATH_VISUALIZER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include "pose.h"
#include "point.h"

class PathVisualizer{
    public:
    PathVisualizer(ros::NodeHandle& n);
 void publishPose(const Pose& pose);

 private:
 ros::Publisher path_pub_;
 nav_msgs::Path path_;
};

#endif