#ifndef NAVSATFIX_ODOMETRY_EKF_HPP
#define NAVSATFIX_ODOMETRY_EKF_HPP

#include <fstream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include "lla.hpp"

class NavsatfixOdometryEKF {
public:
    NavsatfixOdometryEKF();

    void run();
private:
    void subCallbackOdom(const nav_msgs::Odometry::ConstPtr&);
    void subCallbackNavsatfix(const sensor_msgs::NavSatFix::ConstPtr&);

    std::ofstream odom_file_, navsatfix_file_;
    LLA initial_lla_;
};

#endif