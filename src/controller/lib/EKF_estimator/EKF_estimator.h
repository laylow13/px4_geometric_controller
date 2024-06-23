//
// Created by lay on 24-1-25.
//

#ifndef BUILD_EKF_ESTIMATOR_H
#define BUILD_EKF_ESTIMATOR_H

#include "eigen3/Eigen/Eigen"

using namespace Eigen;

class EKF_estimator {
    struct States {
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Eigen::Vector3d acc;
        Eigen::Quaterniond att;
        Eigen::Vector3d ang_vel;
        Eigen::Vector3d ang_acc;
        Eigen::Matrix3d var_ang_vel;
    };
    struct Inputs {
        double thrust;
        Eigen::Vector3d torque;
    };
    struct Params {
        double mass;
        double var_mass;
        Eigen::Vector3d inertia;
        Eigen::Matrix3d var_inertia;
    };
    struct Initial_val {
        double m;
        double var_m, R_m, Q_m;
        Matrix<double, 6, 1> x2;
        Matrix<double, 6, 6> var_x2, R_x2;
        Matrix3d Q_x2, var_torque;
    };
public:
    explicit EKF_estimator(double sample_T_ = 0.01);

    void estimate(const States &measurements, const Inputs &inputs);

    void get_estimates(Params &param_estimates, States &state_estimates) const;

    void reinit(Initial_val &init_val);

    void set_var_torque(const Matrix3d &_var) { var_torque = _var; }

private:
    Vector3d acc_meas, ang_vel, ang_vel_meas;
    Quaterniond att_meas;
    Vector3d M;//torque~px4 torque;thrust~px4 thrust;M~moments to use
    double T;//thrust to use
    double m;//mass-x1
    Matrix<double, 6, 1> x2;//Omega inertia
    Vector3d j;//inertia
    double sample_T;
    double var_m, R_m, H_m, K_m, Q_m;
    Matrix<double, 6, 6> var_x2, R_x2, F_x2;
    Matrix<double, 6, 3> K_x2, G_x2;
    Matrix<double, 3, 6> H_x2;
    Matrix3d Q_x2, var_torque;

    void pre_process(const States &measurements, const Inputs &inputs);

    void calculate_F();

    void calculate_G();

    void calculate_H();

    void calculate_K();

    double dynamic_m();

    void dynamic_x2();
};


#endif //BUILD_EKF_ESTIMATOR_H
