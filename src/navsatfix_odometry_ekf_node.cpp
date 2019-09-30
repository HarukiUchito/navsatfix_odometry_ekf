/*
    This package subscribes
        - nav_msgs/Odometry
        - sensor_msgs/NavSatFix
    and fuses, publishes
        - nav_msgs/Odometry
    message.
*/

#include <ros/ros.h>
#include "navsatfix_odometry_ekf.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navsatfix_odometry_ekf_node");
    
    NavsatfixOdometryEKF noen;
    noen.run();
    
    return 0;
}
