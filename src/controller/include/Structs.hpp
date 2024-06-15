#ifndef _STRUCTS_H
#define _STRUCTS_H

#include "Eigen/Eigen"

struct UAV_motion_linear_t
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Vector3d jerk;
};

struct UAV_motion_angular_t
{
    Eigen::Quaterniond q;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
};

struct UAV_motion_t
{
    UAV_motion_linear_t linear;
    UAV_motion_angular_t angular;
};

#endif