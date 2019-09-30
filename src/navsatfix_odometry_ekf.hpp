#ifndef NAVSATFIX_ODOMETRY_EKF_HPP
#define NAVSATFIX_ODOMETRY_EKF_HPP

#include <fstream>
#include <nav_msgs/Odometry.h>

class NavsatfixOdometryEKF {
public:
    NavsatfixOdometryEKF();

    void run();
private:
    void subCallback(const nav_msgs::Odometry::ConstPtr&);

    std::ofstream odom_file_;
};

#endif