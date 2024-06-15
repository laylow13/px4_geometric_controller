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
    Control_base() {
        fM_cmd.setZero();
        fM_disturbance.setZero();
        actuator_cmd.setZero();
    }

    virtual void init(shared_ptr<command_t>, shared_ptr<state_t>, shared_ptr<param_t>) = 0;

    virtual void set_fM_disturbance(const Vector4d &disturbance_) {
        fM_disturbance = disturbance_;
    }

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
