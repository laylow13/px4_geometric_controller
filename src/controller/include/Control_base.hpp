//
// Created by lay on 24-6-3.
//

#ifndef BUILD_CONTROL_BASE_H
#define BUILD_CONTROL_BASE_H

#include <type_definitions.hpp>
#include "memory"

using namespace Eigen;
using std::shared_ptr;

class Control_base {
public:
    Control_base(shared_ptr<command_t> command_, shared_ptr<state_t> state_, shared_ptr<param_t> param_) {
        fM_cmd.setZero();
        fM_disturbance.setZero();
        actuator_cmd.setZero();
        command = command_;
        state = state_;
        param = param_;
    }

//    virtual void reinit(shared_ptr<command_t>, shared_ptr<state_t>, shared_ptr<param_t>) = 0;

    virtual void set_fM_disturbance(const Vector4d &disturbance_) {
        fM_disturbance = disturbance_;
    }

    virtual void get_positional_tracking_error(Vector3d &eX, Vector3d &eV) const= 0;

    virtual void get_rotational_tracking_error(Vector3d &eR, Vector3d &eW) const= 0;

    virtual void get_attitude_cmd(double &thrust_cmd_, Quaterniond &attitude_cmd_, bool is_normalized) const= 0;

    virtual void get_angular_velocity_cmd(double &thrust_cmd_, Vector3d &ang_vel_cmd_, bool is_normalized) const= 0;

    virtual void get_fM_cmd(Vector4d &fM_cmd_, bool is_normalized) const = 0;

    virtual void get_actuator_cmd(Vector4d &actuator_cmd_, bool is_normalized) const = 0;

    virtual void compute_control_output() = 0;

protected:
    shared_ptr<command_t> command;
    shared_ptr<state_t> state;
    shared_ptr<param_t> param;
    Vector4d fM_cmd; //f,Mx,My,Mz
    Vector4d fM_disturbance; //df,dMx,dMy,dMz
    Vector4d actuator_cmd; //a1,a2,a3,a4
};

#endif //BUILD_CONTROL_BASE_H
