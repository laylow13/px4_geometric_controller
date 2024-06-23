//
// Created by lay on 24-1-23.
//

#include "EKF_estimator.h"

EKF_estimator::EKF_estimator(double sample_T_) {
    sample_T = sample_T_;
    m = 0.001f;
    var_m = 0.001f;
    R_m = 0.001f;
    Q_m = 1.0f;

    x2 = VectorXd::Ones(6, 1) * 1;
    var_x2 = Matrix<double, 6, 6>::Constant(1e-4) + Matrix<double, 6, 6>::Identity();
    R_x2 << 0.01, 0, 0, 0, 0, 0,
            0, 0.01, 0, 0, 0, 0,
            0, 0, 0.01, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.1;
//    R_x2 = Matrix<double, 6, 6>::Identity() * 0.1;
    Q_x2 = Matrix3d::Identity() * 0.01;
    var_torque = Matrix3d::Zero();
}

void EKF_estimator::estimate(const States &measurements, const Inputs &inputs) {
    acc_meas = measurements.acc;
    ang_vel_meas = measurements.ang_vel;
    att_meas = measurements.att;
    M = inputs.torque;
    T = inputs.thrust;
    //1.predict
    var_m = var_m + R_m;

    calculate_F();
    calculate_G();
    dynamic_x2();//state transition for x2
    var_x2 = F_x2 * var_x2 * F_x2.transpose() + G_x2 * var_torque * G_x2.transpose() + R_x2;
    //2.measurement
    calculate_H();
    calculate_K();

    m = m + K_m * (acc_meas(2) - dynamic_m());
    var_m = (1.0f - K_m * H_m) * var_m;

    auto I = MatrixXd::Identity(6, 6);
    x2 = x2 + K_x2 * (ang_vel_meas - x2.segment(0, 3));
    var_x2 = (I - K_x2 * H_x2) * var_x2;
    /////////////////////////////////////////////////////////////////////
    double *M_in = M.data();
    double *ang_vel_in = ang_vel_meas.data();
    double out[42];
//    InertiaEstimator.estimateInertia(M_in, sample_T, ang_vel_in, out);
    j << out[3], out[4], out[5];
}

void EKF_estimator::get_estimates(Params &param_estimates, States &state_estimates) const {
    param_estimates.mass = m;
    param_estimates.var_mass = var_m;
    param_estimates.inertia = x2.segment(3, 3);
    param_estimates.var_inertia = var_x2.block<3, 3>(3, 3);
    state_estimates.ang_vel = x2.segment(0, 3);
    state_estimates.var_ang_vel = var_x2.block<3, 3>(0, 0);
}

void EKF_estimator::reinit(Initial_val &init_val) {
    m = init_val.m;
    var_m = init_val.var_m;
    R_m = init_val.R_m;
    Q_m = init_val.Q_m;

    x2 = init_val.x2;
    var_x2 = init_val.var_x2;
    R_x2 = init_val.R_x2;
    Q_x2 = init_val.Q_x2;
    var_torque = init_val.var_torque;
}

void EKF_estimator::pre_process(const States &measurements, const Inputs &inputs) {
//    acc_meas = measurements.acc;
//    ang_vel_meas = measurements.ang_vel;
//    att_meas = measurements.att;
//    Vector3d torque = inputs.torque;
//    Vector3d thrust = inputs.thrust;
//    Vector4d actuator = inputs.actuator;
//    Vector4d actuator_sim = inputs.actuator_sim;
//    Vector4d actuator_cmd = inputs.actuator_cmd;
//    Vector3d axes = {0, 0, -1};
//    std::array<Vector3d, 4> rotor_pos = {Vector3d(0.15, 0.25, 0), Vector3d(-0.15, -0.19, 0), Vector3d(0.15, -0.25, 0),
//                                         Vector3d(-0.15, 0.19, 0)};
//    double ct = 6.5;
//    double km[4] = {0.05, 0.05, -0.05, -0.05};
//    M = Vector3d::Zero();
//    T = 0;
//    actuator_sim = actuator_sim.array() * actuator_sim.array();//use \omega_i^2
//    for (int i = 0; i < 4; i++) {
//        M += (ct * rotor_pos[i].cross(axes) - ct * km[i] * axes) * actuator_sim(i);
//        T += actuator_sim(i);
//    }
//    RCLCPP_INFO(rclcpp::get_logger("debug"), "M:%2.6f,%2.6f,%2.6f", M(0), M(1), M(2));
//    double l1x = 0.25;
//    double l1y = 0.15;
//    double l2x = 0.19;
//    double l2y = 0.15;
//    double l3x = 0.25;
//    double l3y = 0.15;
//    double l4x = 0.19;
//    double l4y = 0.15;

//    actuator_sim = actuator_sim.array() * actuator_sim.array();//use \omega_i^2
//    auto &omega_1 = actuator_sim(0);
//    auto &omega_2 = actuator_sim(1);
//    auto &omega_3 = actuator_sim(2);
//    auto &omega_4 = actuator_sim(3);
//    M << (omega_2 * l2x + omega_3 * l3x - omega_1 * l1x - omega_4 * l4x), (omega_1 * l1y + omega_3 * l3y -
//                                                                           omega_2 * l2y - omega_4 * l4y),
//            0.06 * (omega_1 +
//                    omega_2 -
//                    omega_3 -
//                    omega_4);
//    T = (omega_1 + omega_2 + omega_3 + omega_4);
//    RCLCPP_INFO(rclcpp::get_logger("debug"), "M:%2.6f,%2.6f,%2.6f", M(0), M(1), M(2));
//    RCLCPP_INFO(rclcpp::get_logger("debug"), "ang_vel:%2.6f,%2.6f,%2.6f", ang_vel_meas(0), ang_vel_meas(1),
//                ang_vel_meas(2));
//    T = thrust(2) > 0 ? 0 : thrust(2);
//    T *= -1.0f;
//    M = torque;
}

//calculate jacobian of the dynamic_x2
void EKF_estimator::calculate_F() {
    const auto &Omega_x = x2(0);
    const auto &Omega_y = x2(1);
    const auto &Omega_z = x2(2);
    const auto &I_x = x2(3);
    const auto &I_y = x2(4);
    const auto &I_z = x2(5);
    const auto &M_x = M(0);
    const auto &M_y = M(1);
    const auto &M_z = M(2);

    F_x2 << 1, (Omega_z * sample_T * (I_y - I_z)) / I_x, (Omega_y * sample_T * (I_y - I_z)) / I_x,
            -(sample_T * (M_x + Omega_y * Omega_z * (I_y - I_z))) / (I_x * I_x), (Omega_y * Omega_z * sample_T) / I_x,
            -(Omega_y * Omega_z * sample_T) / I_x,
            -(Omega_z * sample_T * (I_x - I_z)) / I_y, 1, -(Omega_x * sample_T * (I_x - I_z)) / I_y,
            -(Omega_x * Omega_z * sample_T) / I_y, -(sample_T * (M_y - Omega_x * Omega_z * (I_x - I_z))) / (I_y * I_y),
            (Omega_x * Omega_z * sample_T) / I_y,
            (Omega_y * sample_T * (I_x - I_y)) / I_z, (Omega_x * sample_T * (I_x - I_y)) / I_z, 1,
            (Omega_x * Omega_y * sample_T) / I_z, -(Omega_x * Omega_y * sample_T) / I_z,
            -(sample_T * (M_z + Omega_x * Omega_y * (I_x - I_y))) / (I_z * I_z),
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
}

void EKF_estimator::calculate_G() {
    const auto &I_x = x2(3);
    const auto &I_y = x2(4);
    const auto &I_z = x2(5);
    G_x2 << sample_T / I_x, 0, 0,
            0, sample_T / I_y, 0,
            0, 0, sample_T / I_z,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0;
}

void EKF_estimator::calculate_H() {
    Vector3d e_z{0, 0, 1};//[0 0 1]
    auto body_z = att_meas.toRotationMatrix() * e_z;
    H_m = T * body_z(2) / (m * m);

    H_x2 = Matrix<double, 3, 6>::Zero();
    H_x2(0, 0) = 1;
    H_x2(1, 1) = 1;
    H_x2(2, 2) = 1;
}

void EKF_estimator::calculate_K() {
    K_m = var_m * H_m / (H_m * var_m * H_m + Q_m);

    auto var_innov = H_x2 * var_x2 * H_x2.transpose() + Q_x2;
    K_x2 = var_x2 * H_x2.transpose() * var_innov.inverse();
}

//as measurement function for m
double EKF_estimator::dynamic_m() {
    Vector3d e_z{0, 0, 1};//[0 0 1]
    auto body_z = att_meas.toRotationMatrix() * e_z;
    double acc_hat = -T * body_z(2) / m + 9.8;
    return acc_hat;
}

//as state transition function, just update Omega(x2(0:3))
void EKF_estimator::dynamic_x2() {
    Matrix3d inertia;
    inertia << x2(3), 0, 0,
            0, x2(4), 0,
            0, 0, x2(5);
    auto inertia_inv = inertia.inverse();
    Vector3d ang_vel_temp = x2.segment(0, 3);
    Vector3d ang_vel_dot = inertia_inv * M - inertia_inv * (ang_vel_temp.cross(inertia * ang_vel_temp));
    x2.segment(0, 3) += ang_vel_dot * sample_T;
}
