//
// Created by lay on 24-6-8.
//

#ifndef BUILD_GEOMETRIC_CONTROL_H
#define BUILD_GEOMETRIC_CONTROL_H

#include "Control_base.hpp"

class geometric_param_t : public param_t {
public:
    bool use_decoupled_yaw;
    Matrix3d kX;
    Matrix3d kV;
    Matrix3d kR;
    Matrix3d kW;

    bool use_integral;
    double int_limit;
    double kIX;
    double ki;
    double kIR;
    double kI;
    double kyI;
    double c1;
    double c2;
    double c3;
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

class Geometric_control : public Control_base {
public:
    Geometric_control();

    void init(shared_ptr<command_t>, shared_ptr<state_t>, shared_ptr<param_t>) override;

    void compute_control_output() override;

    void get_fM_cmd(Vector4d &fM_cmd_, bool is_normalized) const override;

    void get_actuator_cmd(Vector4d &actuator_cmd_, bool is_normalized) const override;

private:
    void position_control();

    void attitude_control();

    void attitude_control_decoupled_yaw();

    static Matrix3d hat(const Vector3d &v);

    static Vector3d vee(const Matrix3d &V);

    static void deriv_unit_vector(const Vector3d &A, const Vector3d &A_dot, const Vector3d &A_ddot, \
    Vector3d &q, Vector3d &q_dot, Vector3d &q_ddot);

private:
    shared_ptr<geometric_param_t> geometric_param;
    Matrix3d R;
    double f_total{};
    Vector3d M = Vector3d::Zero();  /**< Control moments */
    // for integral controller
    Integrator<Vector3d> eIR; /**< Attitude integral error */
    Integrator<Vector3d> eIX; /**< Position integral error */
    Integrator<double> eIr; /**< Attitude integral error for roll axis */
    Integrator<double> eIp; /**< Attitude integral error for pitch axis */
    Integrator<double> eIy; /**< Attitude integral error for yaw axis */

    Vector3d e1, e2, e3;
    Vector3d eX = Vector3d::Zero(); /**< Position error */
    Vector3d eV = Vector3d::Zero(); /**< Velocity error */
    Vector3d eR = Vector3d::Zero(); /**< Attitude error */
    Vector3d eW = Vector3d::Zero(); /**< Angular rate error */

    Vector3d ei = Vector3d::Zero(); /**< Position integral error */

//    Vector3d b1 = Vector3d::Zero(); /**< Direction of the first body axis */
//    Vector3d b2 = Vector3d::Zero(); /**< Direction of the second body axis */
//    Vector3d b3 = Vector3d::Zero(); /**< Direction of the third body axis */
//    Vector3d b3_dot = Vector3d::Zero(); /**< Desired rate of change of b3 axis */

    Vector3d b3d = Vector3d::Zero();
    Vector3d b3d_dot = Vector3d::Zero();
    Vector3d b3d_ddot = Vector3d::Zero();
    Vector3d Wd = Vector3d::Zero();
    Vector3d Wd_dot = Vector3d::Zero();
    Vector3d Wd_2dot = Vector3d::Zero();
    Matrix3d Rd = Matrix3d::Identity();
    Vector3d b1c = Vector3d::Zero();
    double wc3 = 0.0;
    double wc3_dot = 0.0;
    double dt{};
};


#endif //BUILD_GEOMETRIC_CONTROL_H
