#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
    DONE:
      * predict the state
    */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    DONE:
      * update the state by using Kalman Filter equations
    */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    DONE:
      * update the state by using Extended Kalman Filter equations
    */
    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);
    double rho = sqrt(pow(px, 2) + pow(py, 2));
    // bearing: referenced counter-clockwise from the x-axis
    double phi = 0;
    if (fabs(px) > 0.001) {
        phi = atan2(py, px);
    }

    // range rate: the projection of the velocity,
    double rho_dot = 0;
    // avoid divide by zero exception
    if (fabs(rho) > 0.0001) {
        rho_dot = (px * vx + py * vy) / rho;
    }

    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;
    VectorXd y = z - z_pred;
    if (fabs(y(1)) > 2 * M_PI) {
        std::cout << "phi in y = z - h(x): " << y(1) << std::endl;
        std::cout << "atan2() returns values between -pi and pi.\n"
                "Normalizing angles to make sure phi in y = z - h(x) for radar measurements is in (-pi, pi)"
                  << std::endl;
        y(1) = y(1) > 0 ? y(1) - 2 * M_PI : y(1) + 2 * M_PI;
    }

    std::cout << "y = " << y << std::endl;
    std::cout << "z = " << z << std::endl;
    std::cout << "z_pred = " << z_pred << std::endl;


    // have initialized H_ with Hj jacobian before
    MatrixXd Ht = H_.transpose();
//  std::cout << Ht << std::endl << std::endl;
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    // above can be simply written as:
//    MatrixXd S = H_ * P_ * H_.transpose() + R_;
//    MatrixXd K = P_ * H_.transpose() * S.inverse();


    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;

}
