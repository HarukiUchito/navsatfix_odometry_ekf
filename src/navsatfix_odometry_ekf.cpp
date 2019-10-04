#include "navsatfix_odometry_ekf.hpp"
#include "utility.hpp"

#include <iostream>
#include <typeinfo>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

NavsatfixOdometryEKF::NavsatfixOdometryEKF()
{
    ros::NodeHandle nhp{"~"};

    odom_file_.open(getParamFromRosParam<std::string>(nhp, "dump_file_path_odom"));
    if (not odom_file_.is_open())
        ros_exception("odom file cannot be opened");

    navsatfix_file_.open(getParamFromRosParam<std::string>(nhp, "dump_file_path_navsatfix"));
    if (not navsatfix_file_.is_open())
        ros_exception("navsatfix file could not be opened");

    ekf_file_.open(getParamFromRosParam<std::string>(nhp, "dump_file_path_ekf"));
    if (not ekf_file_.is_open())
        ros_exception("ekf file could not be opened");

    ros::NodeHandle nh;
    // Get initial LLA from rosparam
    double lat{getParamFromRosParam<double>(nh, "gnss_ini_lat")};
    double lon{getParamFromRosParam<double>(nh, "gnss_ini_lon")};
    double alt{getParamFromRosParam<double>(nh, "gnss_ini_alt")};

    initial_lla_ = LLA{lat, lon, alt};
    recent_lla_ = initial_lla_;
}

void NavsatfixOdometryEKF::run()
{
    ros::NodeHandle nhp{"~"};
    std::string odom_topic{getParamFromRosParam<std::string>(nhp, "odom_topic")};
    std::string navsatfix_topic{getParamFromRosParam<std::string>(nhp, "navsatfix_topic")};

    ros::NodeHandle nh;
    ros::Subscriber sub_odom = nh.subscribe(odom_topic, 1, &NavsatfixOdometryEKF::subCallbackOdom, this);
    ros::Subscriber sub_navsatfix = nh.subscribe(navsatfix_topic, 1, &NavsatfixOdometryEKF::subCallbackNavsatfix, this);

    tf2_ros::TransformListener tf_listener(tf_buffer_); // listening starts

    ros::Rate rate{10};
    while (ros::ok())
    {

        dumpMeasurements();

        ros::spinOnce();
        rate.sleep();
    }
}

void NavsatfixOdometryEKF::subCallbackOdom(const nav_msgs::Odometry::ConstPtr &odom)
{
    if (got_first_odom_)
    {
        ros::Time recent = recent_odom_.header.stamp;;
        ros::Time current = odom->header.stamp;
        double delta = (current - recent).toSec();
        double odom_v = odom->twist.twist.linear.x;
        double odom_omega = odom->twist.twist.angular.z;
        ekf_.predict(odom_v, odom_omega, delta);
    }
    else
    {
        got_first_odom_ = true;
        ekf_.setTheta(tf2::getYaw(odom->pose.pose.orientation));
    }
    recent_odom_ = *odom;
}

void NavsatfixOdometryEKF::subCallbackNavsatfix(const sensor_msgs::NavSatFix::ConstPtr &navsatfix)
{
    sensor_msgs::NavSatFix cp = *navsatfix;
    recent_lla_ = LLA{cp};

    recent_gnss_xy_ = transformLLAtoOdomFrame(recent_lla_);

    ekf_.correct(recent_gnss_xy_.x, recent_gnss_xy_.y);
}

geometry_msgs::Point NavsatfixOdometryEKF::transformLLAtoOdomFrame(const LLA &lla) const
{
    // lla -> enu
    geometry_msgs::Point pos = CalcRelativePosition(initial_lla_, recent_lla_);

    // enu -> odom
    while (true)
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform("odom_with_imu", "gps", ros::Time(0));
            geometry_msgs::Point ret;
            tf2::doTransform(pos, ret, transformStamped);
            return ret;
        }
        catch (const tf2::TransformException &e)
        {
            std::cout << e.what() << std::endl;
            continue;
        }
    }
}

void NavsatfixOdometryEKF::dumpMeasurements()
{
    double x {recent_odom_.pose.pose.position.x};
    double y {recent_odom_.pose.pose.position.y};
    odom_file_ << x << " " << y << std::endl;
    ROS_INFO("%f, %f is wrote", x, y);

    navsatfix_file_ << recent_gnss_xy_.x << " " << recent_gnss_xy_.y << std::endl;

    ekf_file_ << ekf_.x() << " " << ekf_.y() << " " << ekf_.theta() << std::endl;
}