#include "navsatfix_odometry_ekf.hpp"
#include "utility.hpp"

#include <iostream>
#include <typeinfo>

#include <ros/ros.h>

std::string to_string(const std::string& value)
{
    return value;
}

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

NavsatfixOdometryEKF::NavsatfixOdometryEKF()
{
    ros::NodeHandle nhp{"~"};

    odom_file_.open(getParamFromRosParam<std::string>(nhp, "dump_file_path_odom"));
    if (not odom_file_.is_open())
        ros_exception("odom file cannot be opened");

    navsatfix_file_.open(getParamFromRosParam<std::string>(nhp, "dump_file_path_navsatfix"));
    if (not navsatfix_file_.is_open())
        ros_exception("navsatfix file could not be opened");

    ros::NodeHandle nh;
    // Get initial LLA from rosparam
    double lat {getParamFromRosParam<double>(nh, "gnss_ini_lat")};
    double lon {getParamFromRosParam<double>(nh, "gnss_ini_lon")};
    double alt {getParamFromRosParam<double>(nh, "gnss_ini_alt")};

    initial_lla_ = LLA {lat, lon, alt};
}

void NavsatfixOdometryEKF::run()
{
    ros::NodeHandle nhp{"~"};
    std::string odom_topic {getParamFromRosParam<std::string>(nhp, "odom_topic")};
    std::string navsatfix_topic {getParamFromRosParam<std::string>(nhp, "navsatfix_topic")};

    ros::NodeHandle nh;
    ros::Subscriber sub_odom = nh.subscribe(odom_topic, 1, &NavsatfixOdometryEKF::subCallbackOdom, this);
    ros::Subscriber sub_navsatfix = nh.subscribe(navsatfix_topic, 1, &NavsatfixOdometryEKF::subCallbackNavsatfix, this);

    ros::Rate rate {10};
    while (ros::ok()) {

        ros::spinOnce();
        rate.sleep();
    }
}

void NavsatfixOdometryEKF::subCallbackOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
    double x {odom->pose.pose.position.x};
    double y {odom->pose.pose.position.y};

    odom_file_ << x << " " << y << std::endl;

    ROS_INFO("%f, %f is wrote", x, y);
}

void NavsatfixOdometryEKF::subCallbackNavsatfix(const sensor_msgs::NavSatFix::ConstPtr& navsatfix)
{
    sensor_msgs::NavSatFix cp = *navsatfix;
    LLA current_lla {cp};

    geometry_msgs::Point pos = CalcRelativePosition(initial_lla_, current_lla);
    ROS_INFO("new point lat: %f, lon: %f -> x: %f, y: %f", cp.latitude, cp.longitude, pos.x, pos.y);
    
    navsatfix_file_ << pos.x << " " << pos.y << std::endl;
}