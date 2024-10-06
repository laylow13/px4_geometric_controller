//
// Created by lay on 24-6-8.
//

#include "Geometric_control.hpp"

Geometric_control::Geometric_control(shared_ptr<command_t> command_, shared_ptr<state_t> state_,
                                     shared_ptr<geometric_param_t> geometric_param_) : command(command_),
                                                                                       state(state_),
                                                                                       geometric_param(
                                                                                               geometric_param_) {
}


void Geometric_control::compute_control_output() {
    position_control();
    // This is set through the config
    if (geometric_param->use_decoupled_yaw) {
        attitude_control_decoupled_yaw();
    } else {
        attitude_control();
    }
}

void Geometric_control::get_attitude_cmd(double &thrust_cmd_, Quaterniond &attitude_cmd_, bool is_normalized) const {
    thrust_cmd_ = -f_total;
    if (is_normalized)
        thrust_cmd_ = std::max(-1.0, std::min(thrust_cmd_ / geometric_param->thrust_scale, 0.0));
    attitude_cmd_ = Quaterniond(Rd);
}

void
Geometric_control::get_angular_velocity_cmd(double &thrust_cmd_, Vector3d &ang_vel_cmd_, bool is_normalized) const {
    thrust_cmd_ = -f_total;
    if (is_normalized)
        thrust_cmd_ = std::max(-1.0, std::min(thrust_cmd_ / geometric_param->thrust_scale, 0.0));
    ang_vel_cmd_ =
            eR * 2.0 / geometric_param->attctrl_tau; //Ref https://github.com/Jaeyoung-Lim/mavros_controllers/issues/230
}

void Geometric_control::get_fM_cmd(double &thrust_cmd_, Vector3d &torque_cmd_, bool is_normalized) const {
    thrust_cmd_ = -f_total;
    torque_cmd_ << M(0), M(1), M(2);
    if (is_normalized) {
        thrust_cmd_ /= geometric_param->thrust_scale;
        thrust_cmd_ = std::max(-1.0, std::min(thrust_cmd_, 0.0)); //constrain normalized f to -1-0
        torque_cmd_ /= geometric_param->torque_scale;
        torque_cmd_ = torque_cmd_.cwiseMax(-1.0).cwiseMin(1.0); //constrain normalized M to -1-1
    }
}

void Geometric_control::get_actuator_cmd(Vector4d &actuator_cmd_, bool is_normalized) const {
//    actuator_cmd_ = actuator_cmd;
    if (is_normalized) {

    }
}

void Geometric_control::get_positional_tracking_error(Vector3d &eX_, Vector3d &eV_) const {
    eX_ = eX;
    eV_ = eV;
}


void Geometric_control::get_rotational_tracking_error(Vector3d &eR_, Vector3d &eW_) const {
    eR_ = eR;
    eW_ = eW;
}

void Geometric_control::position_control() {
    // translational error functions
    eX = state->x - command->x;     // position error - eq (11)
    eV = state->x_dot - command->x_dot; // velocity error - eq (12)

    // position integral terms
    // "use_integral" must be set through the config file
    if (geometric_param->use_integral) {
        eIX.set_limit(Vector3d::Constant(-geometric_param->int_limit),
                      Vector3d::Constant(geometric_param->int_limit));
        eIX.integrate(geometric_param->c1 * eX + eV, 1 / geometric_param->frequency); // eq (13)
    } else {
        eIX.set_zero();
    }

    auto e3 = Vector3d::UnitZ();
    // force 'f' along negative b3-axis - eq (14)
    // this term equals to R.e3
    Vector3d A = -geometric_param->kX * eX
                 - geometric_param->kV * eV
                 - geometric_param->kIX * eIX.get_integral()
                 - geometric_param->m * geometric_param->g * e3
                 + geometric_param->m * command->x_2dot;

    R = state->q.toRotationMatrix();
    Vector3d b3 = R * e3;
    Vector3d b3_dot = R * hat(state->w) * e3; // eq (22)
    f_total = -A.dot(b3);

    // intermediate terms for rotational errors
    Vector3d ea = geometric_param->g * e3 - f_total / geometric_param->m * b3 - command->x_2dot;
    Vector3d A_dot = -geometric_param->kX * eV - geometric_param->kV * ea + geometric_param->m * command->x_3dot;

    double fdot = -A_dot.dot(b3) - A.dot(b3_dot);
    Vector3d eb = -fdot / geometric_param->m * b3 - f_total / geometric_param->m * b3_dot - command->x_3dot;
    Vector3d A_ddot = -geometric_param->kX * ea - geometric_param->kV * eb + geometric_param->m * command->x_4dot;

    Vector3d b3c, b3c_dot, b3c_ddot;
    deriv_unit_vector(-A, -A_dot, -A_ddot, b3c, b3c_dot, b3c_ddot);

    Vector3d A2 = -hat(command->b1d) * b3c;
    Vector3d A2_dot = -hat(command->b1d_dot) * b3c - hat(command->b1d) * b3c_dot;
    Vector3d A2_ddot = -hat(command->b1d_2dot) * b3c
                       - 2.0 * hat(command->b1d_dot) * b3c_dot
                       - hat(command->b1d) * b3c_ddot;

    Vector3d b2c, b2c_dot, b2c_ddot;
    deriv_unit_vector(A2, A2_dot, A2_ddot, b2c, b2c_dot, b2c_ddot);

    b1c = hat(b2c) * b3c;
    Vector3d b1c_dot = hat(b2c_dot) * b3c + hat(b2c) * b3c_dot;
    Vector3d b1c_ddot = hat(b2c_ddot) * b3c
                        + 2.0 * hat(b2c_dot) * b3c_dot
                        + hat(b2c) * b3c_ddot;

    Matrix3d Rddot, Rdddot;

    Rd << b1c, b2c, b3c;
    Rddot << b1c_dot, b2c_dot, b3c_dot;
    Rdddot << b1c_ddot, b2c_ddot, b3c_ddot;

    Wd = vee(Rd.transpose() * Rddot);
    Wd_dot = vee(Rd.transpose() * Rdddot
                 - hat(Wd) * hat(Wd));

    // roll / pitch
    b3d = b3c;
    b3d_dot = b3c_dot;
    b3d_ddot = b3c_ddot;

    // yaw
    wc3 = e3.dot(R.transpose() * Rd * Wd);
    wc3_dot = (e3).dot(R.transpose() * Rd * Wd_dot) - e3.dot(hat(state->w) * R.transpose() * Rd * Wd);
}

void Geometric_control::attitude_control() {
    //  This uses the controller defined in "Control of Complex Maneuvers
    //  for a Quadrotor UAV using Geometric Methods on SE(3)"
    //  URL: https://arxiv.org/pdf/1003.2005.pdf
    Matrix3d RdtR = Rd.transpose() * R;
    eR = 0.5 * vee(RdtR - RdtR.transpose());
    eW = state->w - R.transpose() * Rd * Wd;

    if (geometric_param->use_integral) {
        eIR.set_limit(Vector3d::Constant(-geometric_param->int_limit),
                      Vector3d::Constant(geometric_param->int_limit));
        eIR.integrate(eW + geometric_param->c2 * eR, 1 / geometric_param->frequency);
    } else {
        eIR.set_zero();
    }

    M = -geometric_param->kR * eR
        - geometric_param->kW * eW
        - geometric_param->kIR * eIR.get_integral()
        + hat(R.transpose() * Rd * Wd) * geometric_param->J *
          R.transpose() * Rd * Wd
        + geometric_param->J * R.transpose() * Rd * Wd_dot;
}

void Geometric_control::attitude_control_decoupled_yaw() {
    // This uses the controller defined in "Geometric Controls of a Quadrotor
    // with a Decoupled Yaw Control"
    // URL: https://doi.org/10.23919/ACC.2019.8815189
    Vector3d b1 = R * Vector3d::UnitX();
    Vector3d b2 = R * Vector3d::UnitY();
    Vector3d b3 = R * Vector3d::UnitZ();

    double ky = geometric_param->kR(2, 2);
    double kwy = geometric_param->kW(2, 2);

    // roll/pitch angular velocity vector
    Vector3d W_12 = state->w(0) * b1 + state->w(1) * b2;
    Vector3d b3_dot = hat(W_12) * b3; // eq (26)

    Vector3d W_12d = hat(b3d) * b3d_dot;
    Vector3d W_12d_dot = hat(b3d) * b3d_ddot;

    Vector3d eb = hat(b3d) * b3;           // eq (27)
    Vector3d ew = W_12 + hat(b3) * hat(b3) * W_12d; // eq (28)

    // yaw
    double ey = -b2.dot(b1c);
    double ewy = state->w(2) - wc3;

    // control moment for the roll/pitch dynamics - eq (31)
    Vector3d tau;
    tau = -geometric_param->kR(0, 0) * eb - geometric_param->kW(0, 0) * ew
          - geometric_param->J(0, 0) * b3.transpose() * W_12d * b3_dot
          - geometric_param->J(0, 0) * hat(b3) * hat(b3) * W_12d_dot;

    if (geometric_param->use_integral) {
        // attitude integral terms
        Vector3d eI = ew + geometric_param->c2 * eb;
        eIr.set_limit(-geometric_param->int_limit, geometric_param->int_limit);
        eIp.set_limit(-geometric_param->int_limit, geometric_param->int_limit);
        eIr.integrate(eI.dot(b1), 1 / geometric_param->frequency); // b1 axis - eq (29)
        eIp.integrate(eI.dot(b2), 1 / geometric_param->frequency); // b2 axis - eq (30)
        tau += -geometric_param->kI * eIr.get_integral() * b1 - geometric_param->kI * eIp.get_integral() * b2;
    } else {
        eIr.set_zero();
        eIp.set_zero();
    }

    double M1, M2, M3;

    // control moment around b1 axis - roll - eq (24)
    M1 = b1.transpose() * tau + geometric_param->J(2, 2) * state->w(2) * state->w(1);

    // control moment around b2 axis - pitch - eq (24)
    M2 = b2.transpose() * tau - geometric_param->J(2, 2) * state->w(2) * state->w(0);

    // control moment around b3 axis - yaw - eq (52)
    M3 = -ky * ey - kwy * ewy + geometric_param->J(2, 2) * wc3_dot;
    if (geometric_param->use_integral) {
        eIy.set_limit(-geometric_param->int_limit, geometric_param->int_limit);
        eIy.integrate(ewy + geometric_param->c3 * ey, 1 / geometric_param->frequency);
        M3 += -geometric_param->kyI * eIy.get_integral();
    } else {
        eIy.set_zero();
    }

    M << M1, M2, M3;

    // for saving:
    Matrix3d RdtR = Rd.transpose() * R;
    eR = 0.5 * vee(RdtR - RdtR.transpose());
    eW = state->w - R.transpose() * Rd * Wd;
}

void Geometric_control::deriv_unit_vector(
        const Vector3d &A, const Vector3d &A_dot, const Vector3d &A_ddot,
        Vector3d &q, Vector3d &q_dot, Vector3d &q_ddot
) {
    double nA = A.norm();
    double nA3 = pow(nA, 3);
    double nA5 = pow(nA, 5);

    q = A / nA;
    q_dot = A_dot / nA
            - A * A.dot(A_dot) / nA3;

    q_ddot = A_ddot / nA
             - A_dot / nA3 * (2 * A.dot(A_dot))
             - A / nA3 * (A_dot.dot(A_dot) + A.dot(A_ddot))
             + 3 * A / nA5 * pow(A.dot(A_dot), 2);
}

Matrix3d Geometric_control::hat(const Vector3d &v) {
    Matrix3d V;
    V.setZero();

    V(2, 1) = v(0);
    V(1, 2) = -V(2, 1);
    V(0, 2) = v(1);
    V(2, 0) = -V(0, 2);
    V(1, 0) = v(2);
    V(0, 1) = -V(1, 0);

    return V;
}

Vector3d Geometric_control::vee(const Matrix3d &V) {
    // TODO: improve code by: https://codereview.stackexchange.com/questions/77546/multiply-vector-elements-by-a-scalar-value-using-stl-and-templates
    Vector3d v;
    Matrix3d E;

    v.setZero();
    E = V + V.transpose();

    v(0) = V(2, 1);
    v(1) = V(0, 2);
    v(2) = V(1, 0);

    return v;
}
