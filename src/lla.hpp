#ifndef LLA_HPP
#define LLA_HPP

#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>

#include "utility.hpp"

constexpr double FE_WGS84 = 1.0 / 298.257223563; // earth flattening (WGS84)
constexpr double RE_WGS84 = 6378137.0; // earth semimajor axis (WGS84) (m)

// Transformation of ecef position to geodetic position
geometry_msgs::Point ECEF_to_Geodetic(const geometry_msgs::Point &ecef);

struct LLA : private Point {
    double latitude() const { return x; }
    double longitude() const { return y; }
    double altitude() const { return z; }
    LLA() : Point() {}
    LLA(double lat, double lon, double alt)
        : Point(lat, lon, alt) {}
    LLA(const sensor_msgs::NavSatFix nav)
        : Point(nav.latitude, nav.longitude, nav.altitude) {}
};

geometry_msgs::Point CalcRelativePosition(const LLA &initial, const LLA &current);

#endif