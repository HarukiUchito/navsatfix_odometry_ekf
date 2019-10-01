#ifndef NAVSATFIX_ODOMETRY_EKF_HPP
#define NAVSATFIX_ODOMETRY_EKF_HPP

#include <utility>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>

#include "lla.hpp"

class NavsatfixOdometryEKF {
public:
    NavsatfixOdometryEKF();

    void run();
private:
    void subCallbackOdom(const nav_msgs::Odometry::ConstPtr&);
    void subCallbackNavsatfix(const sensor_msgs::NavSatFix::ConstPtr&);

    void dumpMeasurements(tf2_ros::Buffer&);

    std::ofstream odom_file_, navsatfix_file_;
    LLA initial_lla_; // is used to calculate relative position from recent lla measurement
    LLA recent_lla_;
    
    geometry_msgs::Point transformLLAtoOdomFrame(tf2_ros::Buffer&, const LLA&) const;

    geometry_msgs::Point recent_odom_xy_;
};

#endif