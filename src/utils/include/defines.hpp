#ifndef _STRUCTS_H
#define _STRUCTS_H

#include "Eigen/Eigen"

#define FRAME_WORLD_NED 0
#define FRAME_WORLD_ENU 1

#define FRAME_BODY_FRD 2
#define FRAME_BODY_FLU 3

struct state_t {
    struct TimeStamp {
        int32_t sec;
        uint32_t nanosec;
    } timestamp;
    uint8_t world_frame;
    uint8_t body_frame;
    Eigen::Vector3d x;
    Eigen::Vector3d x_dot;
    Eigen::Vector3d x_2dot;
    Eigen::Vector3d x_3dot;
    Eigen::Vector3d x_4dot;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d w_dot;
};

struct command_t {
    struct TimeStamp {
        int32_t sec;
        uint32_t nanosec;
    } timestamp;
    uint8_t world_frame;
    uint8_t body_frame;
    Eigen::Vector3d x;
    Eigen::Vector3d x_dot;
    Eigen::Vector3d x_2dot;
    Eigen::Vector3d x_3dot;
    Eigen::Vector3d x_4dot;
    Eigen::Vector3d b1d;
    Eigen::Vector3d b1d_dot;
};

#endif
