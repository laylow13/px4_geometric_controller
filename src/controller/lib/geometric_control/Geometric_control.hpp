//
// Created by lay on 24-6-8.
//

#ifndef BUILD_GEOMETRIC_CONTROL_H
#define BUILD_GEOMETRIC_CONTROL_H

#include "Control_base.hpp"

class geometric_param_t : public param_t {
public:
    //control params
    bool use_decoupled_yaw;
    Matrix3d kX;
    Matrix3d kV;
    Matrix3d kR;
    Matrix3d kW;
    //integral params
    bool use_integral;
    double int_limit;
    double kIX; //position integral scale
    double kIR; //attitude integral scale
    double kI;  // pitch and roll integral scale
    double kyI; //yaw integral scale
    double c1;
    double c2;
    double c3;
    //custom
    double attctrl_tau;
};

template<typename T>
class Integrator {
public:
    explicit Integrator(bool use_limit_ = false) : use_limit(use_limit_) {
        set_zero();
    }

    void set_limit(const T min_limit_, const T max_limit_) {
        min_limit = min_limit_;
        max_limit = max_limit_;
    }

    void integrate(const T current_val, const double dt) {
        integral += (last_val + current_val) * dt / 2;
        if (use_limit) limit();
        last_val = current_val;
    }

    T get_integral() {
        return integral;
    }

    void set_zero() {
        if constexpr (std::is_same<T, double>::value) {
            last_val = 0;
            integral = 0;
        } else {
            last_val = T::Zero();
            integral = T::Zero();
        }
    }

private:

    void limit() {
        if constexpr (std::is_same<T, double>::value) {
            // 对于double类型的数据
            integral = std::max(min_limit, std::min(integral, max_limit));
        } else {
            // 对于Eigen::Vector3d类型的数据
            integral = integral.cwiseMax(min_limit).cwiseMin(max_limit);
        }
    }

    bool use_limit;
    T last_val;
    T integral;
    T min_limit;
    T max_limit;
};

class Geometric_control {
public:
    Geometric_control(shared_ptr<command_t>, shared_ptr<state_t>, shared_ptr<geometric_param_t>);

    void compute_control_output();

    void get_fM_cmd(double &thrust_cmd_, Vector3d &torque_cmd_, bool is_normalized) const;

    void get_actuator_cmd(Vector4d &actuator_cmd_, bool is_normalized) const;

    void get_attitude_cmd(double &thrust_cmd_, Quaterniond &attitude_cmd_, bool is_normalized) const;

    void get_angular_velocity_cmd(double &thrust_cmd_, Vector3d &ang_vel_cmd_, bool is_normalized) const;

    void get_positional_tracking_error(Vector3d &eX_, Vector3d &eV_) const;

    void get_rotational_tracking_error(Vector3d &eR_, Vector3d &eW_) const;

private:
    void position_control();

    void attitude_control();

    void attitude_control_decoupled_yaw();

    static Matrix3d hat(const Vector3d &v);

    static Vector3d vee(const Matrix3d &V);

    static void deriv_unit_vector(const Vector3d &A, const Vector3d &A_dot, const Vector3d &A_ddot, \
    Vector3d &q, Vector3d &q_dot, Vector3d &q_ddot);

private:
    shared_ptr<command_t> command;
    shared_ptr<state_t> state;
    shared_ptr<geometric_param_t> geometric_param;
    Vector3d force_disturbance; //dF
    Vector3d torque_disturbance; //dM

    // for integral controller
//    double dt;//TODO
    Integrator<Vector3d> eIR{true}; /**< Attitude integral error */
    Integrator<Vector3d> eIX{true}; /**< Position integral error */
    Integrator<double> eIr{true}; /**< Attitude integral error for roll axis */
    Integrator<double> eIp{true}; /**< Attitude integral error for pitch axis */
    Integrator<double> eIy{true}; /**< Attitude integral error for yaw axis */

    //for saving
    double f_total{};
    Vector3d M = Vector3d::Zero();  /**< Control moments */
    Vector3d eX = Vector3d::Zero(); /**< Position error */
    Vector3d eV = Vector3d::Zero(); /**< Velocity error */
    Vector3d eR = Vector3d::Zero(); /**< Attitude error */
    Vector3d eW = Vector3d::Zero(); /**< Angular rate error */

    //for attitude control
    Vector3d b3d = Vector3d::Zero();
    Vector3d b3d_dot = Vector3d::Zero();
    Vector3d b3d_ddot = Vector3d::Zero();
    Vector3d Wd = Vector3d::Zero();
    Vector3d Wd_dot = Vector3d::Zero();
//    Vector3d Wd_2dot = Vector3d::Zero();
    Matrix3d R = Matrix3d::Identity();
    Matrix3d Rd = Matrix3d::Identity();
    Vector3d b1c = Vector3d::Zero();
    double wc3 = 0.0;
    double wc3_dot = 0.0;
};


#endif //BUILD_GEOMETRIC_CONTROL_H
