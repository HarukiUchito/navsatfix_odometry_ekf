#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <geometry_msgs/Point.h>

#define ros_exception(message)                             \
{   std::string dump = "\nmessage: ";                      \
    dump += (message);                                     \
    dump += "\nat line: " + std::to_string(__LINE__);      \
    dump += "\nin function: " + std::string(__FUNCTION__); \
    dump += "\nin file: " + std::string(__FILE__);         \
    ROS_ERROR("%s", dump.c_str());                         \
    throw std::runtime_error(dump.c_str());                \
}

struct Point {
public:
    double x;
    double y;
    double z;
    Point() : x(0.0), y(0.0), z(0.0) {}
    Point(double ix, double iy, double iz)
        : x(ix), y(iy), z(iz) {}
    double norm(const Point&) const;    
};
// Binary operators for Point
Point operator-(const Point &p1, const Point &p2);

geometry_msgs::Point GetPointMsg(const double x, const double y, const double z);
geometry_msgs::Point GetPointMsg(const Point&);

double deg2rad(double degrees);

#endif