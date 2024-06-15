//
// Created by lay on 24-6-8.
//
#include "Geometric_control.hpp"
#include "memory"
#include "iostream"

using namespace Eigen;
using namespace std;

int main() {
    auto command = make_shared<command_t>();
    auto state = make_shared<state_t>();
    auto param = make_shared<geometric_param_t>();

    param->J.diagonal() << 0.02, 0.02, 0.04;
    param->m = 1.95;
    param->g = 9.81;

    param->use_decoupled_yaw = true;
    param->kX.diagonal() << 16.0, 16.0, 16.0;
    param->kV.diagonal() << 13.0, 13.0, 13.0;
    param->kR.diagonal() << 1.6, 1.6, 0.60;
    param->kW.diagonal() << 0.40, 0.40, 0.10;
    param->c_tf = 0.0135;
    param->l = 0.23;

    param->use_integral = true;
    param->kIX = 4.0;
    param->ki = 0.01;
    param->kIR = 0.015;
    param->kI = 0.01;
    param->kyI = 0.02;
    param->c1 = 1.0;
    param->c2 = 1.0;
    param->c3 = 1.0;


    auto geometric_controller = Geometric_control();
    geometric_controller.init(command, state, param);
    for (int i = 0; i < 10; ++i) {
        state->x << i, i, i;
        state->x_dot.setZero();
        state->x_2dot.setZero();
        state->q.setIdentity();
        state->w.setZero();

        command->x.setZero();
        command->x_dot.setZero();
        command->x_2dot.setZero();
        command->x_3dot.setZero();
        command->x_4dot.setZero();
        command->b1d << 1.0, 0.0, 0.0;
        command->b1d_dot.setZero();
        command->b1d_2dot.setZero();

        geometric_controller.compute_control_output();
        Eigen::Vector4d fM_cmd;
        geometric_controller.get_fM_cmd(fM_cmd, false);
        std::cout << "i = " << i << ":\tf = " << fM_cmd << std::endl;
    }
    return 0;
}