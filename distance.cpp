#include "distance.h"
#include <cmath>

double squaredDistance(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}

double euclideanDistance(const Point& p1, const Point& p2) {
    return std::sqrt(squaredDistance(p1, p2));
}