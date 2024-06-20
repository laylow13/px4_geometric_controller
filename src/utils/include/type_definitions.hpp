#ifndef _STRUCTS_H
#define _STRUCTS_H

#include "Eigen/Eigen"

using namespace Eigen;
#define FRAME_WORLD_NED 0
#define FRAME_WORLD_ENU 1

#define FRAME_BODY_FRD 2
#define FRAME_BODY_FLU 3

class state_t {
public:
    struct TimeStamp {
        int32_t sec;
        uint64_t nanosec;
    } timestamp;
    uint8_t world_frame;
    uint8_t body_frame;
    Vector3d x;
    Vector3d x_dot;
    Vector3d x_2dot;
    Vector3d x_3dot;
    Vector3d x_4dot;
    Quaterniond q;
    Vector3d w;
    Vector3d w_dot;
};

class command_t {
public:
    struct TimeStamp {
        int32_t sec;
        uint32_t nanosec;
    } timestamp;
    uint8_t world_frame;
    uint8_t body_frame;
    Vector3d x;
    Vector3d x_dot;
    Vector3d x_2dot;
    Vector3d x_3dot;
    Vector3d x_4dot;
    Vector3d b1d;
    Vector3d b1d_dot;
    Vector3d b1d_2dot;
};

class param_t {
public:
    Matrix3d J;
    double m;
    double g;
    double c_tf;
    double l;

    double thrust_scale;
    double torque_scale;
    double frequency;
};

#endif
