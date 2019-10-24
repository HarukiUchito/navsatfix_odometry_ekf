#include "wheelOdom_gnss_ekf.hpp"
#include <iostream>

WheelOdomGNSSEKF::WheelOdomGNSSEKF()
{
    current_state_.setZero();

    // relation between state and measurement
    matH_ <<
        1, 0, 0,
        0, 1, 0;
}

void WheelOdomGNSSEKF::set_covariances(
    const std::vector<double> &initial_covariance,
    const std::vector<double> &process_noise_covariance,
    const std::vector<double> &measurement_noise_covariance
)
{
    if (initial_covariance.size() != 9 or
        process_noise_covariance.size() != 9 or
        measurement_noise_covariance.size() != 4)
    {
        throw std::runtime_error("Covariance size is wrong");
    }

    auto copy = [](int n, const std::vector<double> &from, auto &to)
    {
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                to(i, j) = from[n * i + j];
    };

    copy(3, initial_covariance, current_covariance_);
    copy(3, process_noise_covariance, process_noise_covariance_);
    copy(2, measurement_noise_covariance, measurement_noise_covariance_);
    std::cout << "initial_covariance:" << std::endl << current_covariance_ << std::endl;
    std::cout << "process_noise_covariance:" << std::endl << process_noise_covariance_ << std::endl;
    std::cout << "measurement_noise_covariance:" << std::endl << measurement_noise_covariance_ << std::endl;

    covariances_set_ = true;
}

double WheelOdomGNSSEKF::mahalanobisDistance(double gnss_x, double gnss_y) const
{
    Eigen::Matrix2d p; p <<
        current_covariance_(0, 0), current_covariance_(0, 1),
        current_covariance_(1, 0), current_covariance_(1, 1);

    Eigen::Vector2d xd;
    xd << x() - gnss_x, y() - gnss_y;

    double ret = sqrt(xd.transpose() * p.inverse() * xd);
    return ret;
}

void WheelOdomGNSSEKF::predict(double odom_v, double odom_omega, double delta)
{
    if (not covariances_set_)
        throw std::runtime_error("Covariances are not set");

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
        process_noise_covariance_;
}

void WheelOdomGNSSEKF::correct(double gnss_x, double gnss_y)
{
    if (not covariances_set_)
        throw std::runtime_error("Covariances are not set");

    if (mahalanobisDistance(gnss_x, gnss_y) >= 1.0)
        return;

    Eigen::MatrixXd measurement_residual = 
        Eigen::Vector2d {gnss_x, gnss_y} - matH_ * current_state_;
    
    Eigen::MatrixXd measurement_residual_covariance =
        matH_ * current_covariance_ * matH_.transpose() +
        measurement_noise_covariance_;

    Eigen::MatrixXd kalman_gain =
        current_covariance_ * matH_.transpose() * measurement_residual_covariance.inverse();
    
    current_state_ += kalman_gain * measurement_residual;

    current_covariance_ =
        (Eigen::Matrix3d::Identity() - kalman_gain * matH_) * current_covariance_;
}