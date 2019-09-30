#include "navsatfix_odometry_ekf.hpp"

#include <iostream>

#include <ros/ros.h>

#define ros_exception(message)                             \
{   std::string dump{"\nmessage: "};                      \
    dump += (message);                                     \
    dump += "\nat line: " + std::to_string(__LINE__);      \
    dump += "\nin function: " + std::string(__FUNCTION__); \
    dump += "\nin file: " + std::string(__FILE__);         \
    ROS_ERROR("%s", dump.c_str());                         \
    throw std::runtime_error(dump.c_str());                \
}

NavsatfixOdometryEKF::NavsatfixOdometryEKF()
{
    ros::NodeHandle nhp{"~"};
    std::string dump_file_path;
    if (nhp.getParam("dump_file_path", dump_file_path))
        ROS_INFO("dump file path: %s", dump_file_path.c_str());
    else ros_exception("dump file path is not specified on rosparam");
    
    odom_file_.open(dump_file_path);
    if(not odom_file_.is_open()) {
        ros_exception("file cannot be opened");
    }
}

void NavsatfixOdometryEKF::run()
{
    ros::NodeHandle nhp{"~"};
    std::string topic_name;
    if (nhp.getParam("odom_topic", topic_name))
        ROS_INFO("odom topic to be fuse: %s", topic_name.c_str());
    else ros_exception("odometry topic is not specified on rosparam");

    ros::NodeHandle nh;
    ros::Subscriber sub_odom = nh.subscribe(topic_name, 1, &NavsatfixOdometryEKF::subCallback, this);

    ros::Rate rate {10};
    while (ros::ok()) {

        ros::spinOnce();
        rate.sleep();
    }
}

void NavsatfixOdometryEKF::subCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    double x{odom->pose.pose.position.x};
    double y{odom->pose.pose.position.y};

    odom_file_ << x << " " << y << std::endl;

    ROS_INFO("%f, %f is wrote", x, y);
}
