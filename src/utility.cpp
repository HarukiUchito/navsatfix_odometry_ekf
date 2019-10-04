#include "utility.hpp"

double Point::norm(const Point &p) const
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

Point operator-(const Point &p1, const Point &p2)
{
    return Point {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z};
}

geometry_msgs::Point GetPointMsg(const double x, const double y, const double z)
{
    geometry_msgs::Point p;
    p.x = x, p.y = y, p.z = z;
    return p;
}

geometry_msgs::Point GetPointMsg(const Point& p)
{
    geometry_msgs::Point mp;
    mp.x = p.x, mp.y = p.y, mp.z = p.z;
    return mp;
}

double deg2rad(double degrees) {
    return degrees * 4.0 * atan (1.0) / 180.0;
}

std::string to_string(const std::string& value)
{
    return value;
}

std::vector<double> getDoubleVectorParam(const ros::NodeHandle& nh, std::string name)
{
    if (not nh.hasParam(name))
    {
        std::string txt = "param " + name + " is not set";
        ros_exception(txt.c_str());
    }
    std::vector<double> ret;
    nh.getParam(name, ret);
    return ret;
}