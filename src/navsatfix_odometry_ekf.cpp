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
    map_frame_id_ = getParamFromRosParam<std::string>(nhp, "map_frame_id");
    gps_frame_id_ = getParamFromRosParam<std::string>(nhp, "gps_frame_id");

    odom_file_.open(getParamFromRosParam<std::string>(nhp, "dump_file_path_odom"));
    if (not odom_file_.is_open())
        ros_exception("odom file cannot be opened");
    navsatfix_file_.open(getParamFromRosParam<std::string>(nhp, "dump_file_path_navsatfix"));
    if (not navsatfix_file_.is_open())
        ros_exception("navsatfix file could not be opened");
    ekf_file_.open(getParamFromRosParam<std::string>(nhp, "dump_file_path_ekf"));
    if (not ekf_file_.is_open())
        ros_exception("ekf file could not be opened");

    ekf_.set_covariances(
        getDoubleVectorParam(nhp, "initial_covariance"),
        getDoubleVectorParam(nhp, "process_noise_covariance"),
        getDoubleVectorParam(nhp, "measurement_noise_covariance")
    );

    ros::NodeHandle nh;
    // Get initial LLA from rosparam
    double lat {getParamFromRosParam<double>(nh, "gnss_ini_lat")};
    double lon {getParamFromRosParam<double>(nh, "gnss_ini_lon")};
    double alt {getParamFromRosParam<double>(nh, "gnss_ini_alt")};

    initial_lla_ = LLA{lat, lon, alt};
    recent_lla_ = initial_lla_;
}

void NavsatfixOdometryEKF::run()
{
    ros::NodeHandle nhp {"~"};
    std::string odom_topic {getParamFromRosParam<std::string>(nhp, "odom_topic")};
    std::string navsatfix_topic {getParamFromRosParam<std::string>(nhp, "navsatfix_topic")};

    ros::NodeHandle nh;
    ros::Subscriber sub_odom {nh.subscribe(odom_topic, 1, &NavsatfixOdometryEKF::subCallbackOdom, this)};
    ros::Subscriber sub_navsatfix {nh.subscribe(navsatfix_topic, 1, &NavsatfixOdometryEKF::subCallbackNavsatfix, this)};

    std::string filtered_topic {"/NavsatfixOdometryEKF/odometry"};
    ros::Publisher pub_filtered {nh.advertise<nav_msgs::Odometry>(filtered_topic, 1)};

    tf2_ros::TransformListener tf_listener {tf_buffer_}; // listening starts

    ros::Rate rate {10};
    while (ros::ok())
    {
        pub_filtered.publish(createFilteredOdometryMsg());
        //dumpMeasurements();

        ros::spinOnce();
        rate.sleep();
    }
}

nav_msgs::Odometry NavsatfixOdometryEKF::createFilteredOdometryMsg() const
{
    nav_msgs::Odometry odom {recent_odom_};
    odom.header.frame_id = map_frame_id_;
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = ekf_.x();
    odom.pose.pose.position.y = ekf_.y();
    tf2::Quaternion q;
    q.setRPY(0, 0, ekf_.theta());
    odom.pose.pose.orientation = tf2::toMsg(q);

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            int idx_i = i, idx_j = j;
            if (i == 2) idx_i += 3;
            if (j == 2) idx_j += 3;
            odom.pose.covariance[idx_i * 6 + idx_j] = ekf_.getCurrentCovariance(i, j);
        }
    }

    return odom;
}

void NavsatfixOdometryEKF::subCallbackOdom(const nav_msgs::Odometry::ConstPtr &odom)
{
    if (got_first_odom_)
    {
        ros::Time recent {recent_odom_.header.stamp};
        ros::Time current {odom->header.stamp};
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

enum SolutionStatus {
	FIX,
	FLOAT,
	DGNSS,
	SINGLE,
};

void NavsatfixOdometryEKF::subCallbackNavsatfix(const sensor_msgs::NavSatFix::ConstPtr &navsatfix)
{
    recent_lla_ = LLA {*navsatfix};

    recent_gnss_xy_ = transformLLAtoOdomFrame(recent_lla_);

    if (navsatfix->status.status == SolutionStatus::FIX or navsatfix->status.status == SolutionStatus::DGNSS)
        ekf_.correct(recent_gnss_xy_.x, recent_gnss_xy_.y);
}

geometry_msgs::Point NavsatfixOdometryEKF::transformLLAtoOdomFrame(const LLA &lla) const
{
    // lla -> enu
    geometry_msgs::Point pos {CalcRelativePosition(initial_lla_, recent_lla_)};

    // enu -> odom
    while (true)
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped {tf_buffer_.lookupTransform(map_frame_id_, gps_frame_id_, ros::Time(0))};
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