#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <ros/ros.h>
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

std::string to_string(const std::string& value);
template<typename T>
T getParamFromRosParam(const ros::NodeHandle& nh, std::string name) 
{
    T param;
    if (nh.getParam(name, param))
    {
        using namespace std;
        std::string sparam {to_string(param)};
        ROS_INFO("%s: %s", name.c_str(), sparam.c_str());
    }
    else
        ros_exception(name + " is not specified on rosparam");
    return param;
}

std::vector<double> getDoubleVectorParam(const ros::NodeHandle& nh, std::string name);

#endif