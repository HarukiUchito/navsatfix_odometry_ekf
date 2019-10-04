#ifndef WHEELODOM_GNSS_EKF_HPP
#define WHEELODOM_GNSS_EKF_HPP

#include <eigen3/Eigen/Dense>

class WheelOdomGNSSEKF {
public:
    WheelOdomGNSSEKF();

    void predict(double odom_v, double odom_omega, double delta);
    void correct(double gnss_x, double gnss_y);

    double setTheta(double th) { current_state_(2) = th; }
    double x() const { return current_state_.x(); }
    double y() const { return current_state_.y(); }
    double theta() const { return current_state_.z(); };
private:
    Eigen::Vector3d current_state_;
    Eigen::Matrix3d current_covariance_;

    Eigen::Matrix3d process_noise_covariance_;
    Eigen::Matrix2d measurement_noise_covariance_;
    Eigen::Matrix<double, 2, 3> matH_;
};

#endif