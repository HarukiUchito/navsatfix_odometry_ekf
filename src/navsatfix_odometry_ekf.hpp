#ifndef NAVSATFIX_ODOMETRY_EKF_HPP
#define NAVSATFIX_ODOMETRY_EKF_HPP

#include <utility>
#include <fstream>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>

#include "lla.hpp"
#include "wheelOdom_gnss_ekf.hpp"

class NavsatfixOdometryEKF {
public:
    NavsatfixOdometryEKF();

    void run();
private:
    tf2_ros::Buffer tf_buffer_;
    void subCallbackOdom(const nav_msgs::Odometry::ConstPtr&);
    void subCallbackNavsatfix(const sensor_msgs::NavSatFix::ConstPtr&);

    void dumpMeasurements();

    std::ofstream odom_file_, navsatfix_file_, ekf_file_;
    LLA initial_lla_; // is used to calculate relative position from recent lla measurement
    LLA recent_lla_;
    geometry_msgs::Point recent_gnss_xy_;
    
    geometry_msgs::Point transformLLAtoOdomFrame(const LLA&) const;

    bool got_first_odom_ {false};
    nav_msgs::Odometry recent_odom_;

    WheelOdomGNSSEKF ekf_;
};

#endif