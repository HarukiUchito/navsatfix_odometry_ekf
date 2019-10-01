#include "navsatfix_odometry_ekf.hpp"
#include "utility.hpp"

#include <iostream>
#include <typeinfo>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    recent_lla_ = initial_lla_;
}

void NavsatfixOdometryEKF::run()
{
    ros::NodeHandle nhp{"~"};
    std::string odom_topic {getParamFromRosParam<std::string>(nhp, "odom_topic")};
    std::string navsatfix_topic {getParamFromRosParam<std::string>(nhp, "navsatfix_topic")};

    ros::NodeHandle nh;
    ros::Subscriber sub_odom = nh.subscribe(odom_topic, 1, &NavsatfixOdometryEKF::subCallbackOdom, this);
    ros::Subscriber sub_navsatfix = nh.subscribe(navsatfix_topic, 1, &NavsatfixOdometryEKF::subCallbackNavsatfix, this);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer); // listening starts

    ros::Rate rate {10};
    while (ros::ok()) {

        dumpMeasurements(tf_buffer);

        ros::spinOnce();
        rate.sleep();
    }
}

void NavsatfixOdometryEKF::subCallbackOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
    recent_odom_xy_ = GetPointMsg(odom->pose.pose.position.x, odom->pose.pose.position.y, 0.0);
}

void NavsatfixOdometryEKF::subCallbackNavsatfix(const sensor_msgs::NavSatFix::ConstPtr& navsatfix)
{
    sensor_msgs::NavSatFix cp = *navsatfix;
    recent_lla_ = LLA {cp};
}

geometry_msgs::Point NavsatfixOdometryEKF::transformLLAtoOdomFrame(tf2_ros::Buffer& tf_buf, const LLA& lla) const
{
    // lla -> enu
    geometry_msgs::Point pos = CalcRelativePosition(initial_lla_, recent_lla_);

    // enu -> odom
    while (true)
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped = tf_buf.lookupTransform("odom_with_imu", "gps", ros::Time(0));
            geometry_msgs::Point ret;
            tf2::doTransform(pos, ret, transformStamped);
            return pos;//ret;
        }
        catch(const tf2::TransformException &e)
        {
            std::cout << e.what() << std::endl;
            continue;
        }
    }
}

void NavsatfixOdometryEKF::dumpMeasurements(tf2_ros::Buffer& tf_buf)
{
    double x {recent_odom_xy_.x};
    double y {recent_odom_xy_.y};
    odom_file_ << x << " " << y << std::endl;
    ROS_INFO("%f, %f is wrote", x, y);

    geometry_msgs::Point gnss_xy_in_odom = transformLLAtoOdomFrame(tf_buf, recent_lla_);
    navsatfix_file_ << gnss_xy_in_odom.x << " " << gnss_xy_in_odom.y << std::endl;
}