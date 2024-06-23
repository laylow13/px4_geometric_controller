#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "utils/msg/uav_command.hpp"
#include "type_definitions.hpp"

using namespace utils::msg;
using namespace std::chrono_literals;

float t;
int period = 10; // ms
rclcpp::Publisher<UAVCommand>::SharedPtr traj_pub;
std::shared_ptr<rclcpp::Node> traj_node;

void timer_callback();

void pos(UAVCommand &_traj, float _t);

void round_traj(UAVCommand &_traj, float _r, float _w, float _t);

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    traj_node = std::make_shared<rclcpp::Node>("trajectory_generator");
    traj_node->declare_parameter<float>("traj_r", 1.0);
    traj_node->declare_parameter<float>("traj_w", 0.5);
    traj_pub = traj_node->create_publisher<UAVCommand>("/SCIT_drone/UAV_trajectory_command", 10);
    rclcpp::TimerBase::SharedPtr timer = traj_node->create_wall_timer(
            std::chrono::milliseconds(period), &timer_callback);
    rclcpp::spin(traj_node);
    rclcpp::shutdown();
    return 0;
}

void timer_callback() {
    UAVCommand traj;
    float r, w;
    r = traj_node->get_parameter("traj_r").get_value<float>();
    w = traj_node->get_parameter("traj_w").get_value<float>();
//    round_traj(traj, r, w, t);
    pos(traj, t);
    t += period / 1000.0f;
    traj_pub->publish(traj);
    RCLCPP_DEBUG(traj_node->get_logger(), "Timer event");
}

void round_traj(UAVCommand &_traj, float _r, float _w, float _t) {
    _traj.header.stamp = traj_node->get_clock()->now().operator builtin_interfaces::msg::Time();
    _traj.world_frame = FRAME_WORLD_NED;
    _traj.body_frame = FRAME_BODY_FRD;
    _traj.pos.x = _r * cos(_w * _t);
    _traj.pos.y = _r * sin(_w * _t);
    _traj.pos.z = -2.0;
    _traj.vel.x = -_w * _r * sin(_w * _t);
    _traj.vel.y = _w * _r * cos(_w * _t);
    _traj.vel.z = 0.0;
    _traj.acc.x = -_w * _w * _r * cos(_w * _t);
    _traj.acc.y = -_w * _w * _r * sin(_w * _t);
    _traj.acc.z = 0.0;
    _traj.jerk.x = pow(_w, 3) * _r * sin(_w * _t);
    _traj.jerk.y = -pow(_w, 3) * _r * cos(_w * _t);
    _traj.jerk.z = 0.0;
    _traj.snap.x = pow(_w, 4) * _r * cos(_w * _t);
    _traj.snap.y = pow(_w, 4) * _r * sin(_w * _t);
    _traj.snap.z = 0.0;

    _traj.heading.x = 0.0;
    _traj.heading.y = 1.0;
    _traj.heading.z = 0.0;
    _traj.heading_dot.x = 0.0;
    _traj.heading_dot.y = 0.0;
    _traj.heading_dot.z = 0.0;
    _traj.heading_2dot.x = 0.0;
    _traj.heading_2dot.y = 0.0;
    _traj.heading_2dot.z = 0.0;
}

void pos(UAVCommand &_traj, float _t) {
    _traj.header.stamp = traj_node->get_clock()->now().operator builtin_interfaces::msg::Time();
    _traj.world_frame = FRAME_WORLD_NED;
    _traj.body_frame = FRAME_BODY_FRD;

    _traj.pos.x = 1.0;
    _traj.pos.y = 1.0;
    _traj.pos.z = -1.0;
    _traj.vel.x = 0.0;
    _traj.vel.y = 0.0;
    _traj.vel.z = 0.0;
    _traj.acc.x = 0.0;
    _traj.acc.y = 0.0;
    _traj.acc.z = 0.0;
    _traj.jerk.x = 0.0;
    _traj.jerk.y = 0.0;
    _traj.jerk.z = 0.0;
    _traj.snap.x = 0.0;
    _traj.snap.y = 0.0;
    _traj.snap.z = 0.0;

    _traj.heading.x = 0.0;
    _traj.heading.y = 1.0;
    _traj.heading.z = 0.0;
    _traj.heading_dot.x = 0.0;
    _traj.heading_dot.y = 0.0;
    _traj.heading_dot.z = 0.0;
    _traj.heading_2dot.x = 0.0;
    _traj.heading_2dot.y = 0.0;
    _traj.heading_2dot.z = 0.0;
}