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

    param->use_decoupled_yaw = false;
    param->kX.diagonal() << 16.0, 16.0, 16.0;
    param->kV.diagonal() << 13.0, 13.0, 13.0;
    param->kR.diagonal() << 1.6, 1.6, 0.60;
    param->kW.diagonal() << 0.40, 0.40, 0.10;
    param->c_tf = 0.0135;
    param->l = 0.23;

    param->use_integral = false;
    param->kIX = 4.0;
    param->ki = 0.01;
    param->kIR = 0.015;
    param->kI = 0.01;
    param->kyI = 0.02;
    param->c1 = 1.0;
    param->c2 = 1.0;
    param->c3 = 1.0;


    auto geometric_controller = Geometric_control(command, state, param);
//    geometric_controller.init();
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


/*
i = 0:  f = -19.1295
       0
      -0
       0
i = 1:  f =  -35.1295
-0.981254
 0.981254
-0.144619
i = 2:  f =  -51.1295
 -1.31339
  1.31339
-0.243132
i = 3:  f =  -67.1295
 -1.45421
  1.45421
-0.281332
i = 4:  f =  -83.1295
 -1.52977
  1.52977
-0.298069
i = 5:  f =  -99.1295
 -1.57645
  1.57645
-0.306327
i = 6:  f =   -115.13
 -1.60802
  1.60802
-0.310758
i = 7:  f =   -131.13
 -1.63076
  1.63076
-0.313272
i = 8:  f =   -147.13
 -1.64789
  1.64789
-0.314748
i = 9:  f =  -163.13
-1.66126
 1.66126
-0.31563
 * */