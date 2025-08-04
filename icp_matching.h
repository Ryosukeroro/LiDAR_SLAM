#pragma once
#include <vector>
#include "point.h"
#include "constants.h"
#include <cmath>
#include "point.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <cstdio>
#include <sstream>
#include <algorithm>
#include "constants.h"
#include "point.h"
#include "pose.h"
#include "differential.h"

int findClosestPoint(const Point& src, const std::vector<Point>& target);
std::vector<Point> icp_scan_matching(const std::vector<Point>& Source);

