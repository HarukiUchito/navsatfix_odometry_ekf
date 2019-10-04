#ifndef WHEELODOM_GNSS_EKF_HPP
#define WHEELODOM_GNSS_EKF_HPP

#include <eigen3/Eigen/Dense>

class WrongCovarianceSizeException {
private:
    const char* msg {"covariance size is wrong"};
public:
    WrongCovarianceSizeException() {}
    const char* what() { return msg; }
};

class WheelOdomGNSSEKF {
public:
    WheelOdomGNSSEKF();

    // throws exception 
    void set_covariances(
        const std::vector<double> &initial_covariance, // 3x3
        const std::vector<double> &process_noise_covariance, // 3x3
        const std::vector<double> &measurement_noise_covariance // 2x2
    );

    void predict(double odom_v, double odom_omega, double delta);
    void correct(double gnss_x, double gnss_y);

    double setTheta(double th) { current_state_(2) = th; }
    double x() const { return current_state_.x(); }
    double y() const { return current_state_.y(); }
    double theta() const { return current_state_.z(); };
private:
    Eigen::Vector3d current_state_;
    Eigen::Matrix3d current_covariance_;

    // following flag must be set
    // before runnning by calling set_covariances function
    bool covariances_set_ {false};
    Eigen::Matrix3d process_noise_covariance_;
    Eigen::Matrix2d measurement_noise_covariance_;
    Eigen::Matrix<double, 2, 3> matH_;
};

#endif