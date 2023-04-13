#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "base_env/msg/uav_motion.hpp"

using namespace base_env::msg;
using namespace std::chrono_literals;

float t;
int period = 20; // ms
rclcpp::Publisher<UAVMotion>::SharedPtr traj_pub;
std::shared_ptr<rclcpp::Node> traj_node;

void timer_callback();
void pos(UAVMotion &_traj, float _t);
void round_traj(UAVMotion &_traj, float _r, float _w, float _t);

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    traj_node = std::make_shared<rclcpp::Node>("trajectory_generator");
    traj_node->declare_parameter<float>("traj_r", 1.0);
    traj_node->declare_parameter<float>("traj_w", 1.54);
    traj_pub = traj_node->create_publisher<UAVMotion>("/SCIT_drone/UAV_motion_expect", 10);
    rclcpp::TimerBase::SharedPtr timer = traj_node->create_wall_timer(
        std::chrono::milliseconds(period), &timer_callback);
    rclcpp::spin(traj_node);
    rclcpp::shutdown();
    return 0;
}

void timer_callback()
{
    UAVMotion traj;
    float r, w;
    r = traj_node->get_parameter("traj_r").get_value<float>();
    w = traj_node->get_parameter("traj_w").get_value<float>();
    round_traj(traj, r, w, t);
    // pos(traj,t);
    t += period / 1000.0f;
    traj_pub->publish(traj);
    RCLCPP_DEBUG(traj_node->get_logger(), "Timer event");
}

void round_traj(UAVMotion &_traj, float _r, float _w, float _t)
{
    _traj.linear.pos.x = _r * cos(_w * _t);
    _traj.linear.pos.y = _r * sin(_w * _t);
    _traj.linear.pos.z = 1.0;
    _traj.linear.vel.x = -_w * _r * sin(_w * _t);
    _traj.linear.vel.y = _w * _r * cos(_w * _t);
    _traj.linear.vel.z = 0.0;
    _traj.linear.acc.x = -_w * _w * _r * cos(_w * _t);
    _traj.linear.acc.y = -_w * _w * _r * sin(_w * _t);
    _traj.linear.acc.z = 0.0;
}

void pos(UAVMotion &_traj, float _t)
{
    _traj.linear.pos.x = 0.0;
    _traj.linear.pos.y = 0.0;
    _traj.linear.pos.z = 2.0;
    _traj.linear.vel.x = 0.0;
    _traj.linear.vel.y = 0.0;
    _traj.linear.vel.z = 0.0;
    _traj.linear.acc.x = 0.0;
    _traj.linear.acc.y = 0.0;
    _traj.linear.acc.z = 0.0;
}


// void ENU2NED(base_env::msg::UAVMotion &_src, base_env::msg::UAVMotion &_target)
// {
//     using namespace Eigen;

//     Matrix3d R = AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix(); // from NED to ENU
//     Vector3d pos{_src.linear.pos.x, _src.linear.pos.y, _src.linear.pos.z};
//     Vector3d vel{_src.linear.vel.x, _src.linear.vel.y, _src.linear.vel.z};
//     Vector3d acc{_src.linear.acc.x, _src.linear.acc.y, _src.linear.acc.z};
//     // Quaterniond q{_src.angular.q.w, _src.angular.q.x, _src.angular.q.y, _src.angular.q.z};

//     pos = R * pos;
//     vel = R * vel;
//     acc = R * acc;
//     // q = Quaterniond(R).normalized() * q;

//     _target.linear.pos.x = pos(0);
//     _target.linear.pos.y = pos(1);
//     _target.linear.pos.z = pos(2);
//     _target.linear.vel.x = vel(0);
//     _target.linear.vel.y = vel(1);
//     _target.linear.vel.z = vel(2);
//     _target.linear.acc.x = acc(0);
//     _target.linear.acc.y = acc(1);
//     _target.linear.acc.z = acc(2);
//     // _target.angular.q.x = q.x();
//     // _target.angular.q.y = q.y();
//     // _target.angular.q.z = q.z();
//     // _target.angular.q.w = q.w();
// }