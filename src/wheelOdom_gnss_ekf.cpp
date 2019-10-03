#include "wheelOdom_gnss_ekf.hpp"
#include <iostream>

WheelOdomGNSSEKF::WheelOdomGNSSEKF()
{
    current_state_.setZero();

    current_covariance_ <<
        0.001, 0.00, 0.00,
        0.00, 0.001, 0.00,
        0.00, 0.00, 0.001;
    
    process_noise_covariance <<
        0.01, 0.0, 0.0,
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.1;

    measurement_noise_covariance <<
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 0.0001;
    
    matH_ <<
        1, 0, 0,
        0, 1, 0,
        0, 0, 1;
}

void WheelOdomGNSSEKF::predict(double odom_v, double odom_omega, double delta)
{
    double yaw = current_state_.z() + odom_omega * delta;
    current_state_ += Eigen::Vector3d {
        odom_v * cos(yaw) * delta,
        odom_v * sin(yaw) * delta,
        odom_omega * delta
    };

    Eigen::Matrix3d jacobian;
    jacobian <<
        1, 0, -odom_v * sin(yaw) * delta,
        0, 1,  odom_v * cos(yaw) * delta,
        0, 0, 1;

    current_covariance_ =
        jacobian * current_covariance_ * jacobian.transpose() +
        process_noise_covariance;
}

void WheelOdomGNSSEKF::correct(double gnss_x, double gnss_y, double yaw)
{
    double dx = gnss_x - x(), dy = gnss_y - y();
    double distance = sqrt(dx * dx + dy * dy);
    if (distance >= 3.0)
        return;

    Eigen::MatrixXd measurement_residual = 
        Eigen::Vector3d {gnss_x, gnss_y, yaw} - matH_ * current_state_;
    
    Eigen::MatrixXd measurement_residual_covariance =
        matH_ * current_covariance_ * matH_.transpose() +
        measurement_noise_covariance;

    Eigen::MatrixXd kalman_gain =
        current_covariance_ * matH_.transpose() * measurement_residual_covariance.inverse();
    
    double before = current_state_(2);
    current_state_ += kalman_gain * measurement_residual;
    current_state_ += Eigen::Vector3d {1e-8, 1e-8, 1e-8};
    double diff = current_state_(2) - before;

    if (diff > M_PI)
    {
        std::cout << "\nyaw" << std::endl;
        std::cout << "before: " << std::endl << before << std::endl;
        std::cout << "after: " << std::endl << current_state_(2) << std::endl;
        std::cout << "diff: " << diff << std::endl;
    }

    current_covariance_ =
        (Eigen::Matrix3d::Identity() - kalman_gain * matH_) * current_covariance_;
}