#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "frame_transforms/frame_transforms.h"
#include "base_env/msg/uav_motion.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

using std::placeholders::_1;
base_env::msg::UAVMotion motion;
rclcpp::Publisher<base_env::msg::UAVMotion>::SharedPtr state_pub;

void timer_callback();
void att_cb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
void pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
void NED2ENU(base_env::msg::UAVMotion &_src, base_env::msg::UAVMotion &_target);
void ENU2NED(base_env::msg::UAVMotion &_src, base_env::msg::UAVMotion &_target);

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    auto sim_base = std::make_shared<rclcpp::Node>("state_data_convart");
    auto att_sub = sim_base->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos, &att_cb);
    auto pos_sub = sim_base->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos, &pos_cb);
    state_pub = sim_base->create_publisher<base_env::msg::UAVMotion>("/SCIT_drone/UAV_motion", 10);
    auto timer = sim_base->create_wall_timer(
        std::chrono::milliseconds(20),
        &timer_callback);
    rclcpp::spin(sim_base);
    rclcpp::shutdown();
    return 0;
}

void timer_callback()
{
    base_env::msg::UAVMotion target;
    NED2ENU(motion, target);
    state_pub->publish(target);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Timer event");
}
void att_cb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    motion.angular.q.w = msg->q[0];
    motion.angular.q.x = msg->q[1];
    motion.angular.q.y = msg->q[2];
    motion.angular.q.z = msg->q[3];
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "att_cb received message");
}
void pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    motion.timestamp = msg->timestamp;
    motion.linear.pos.x = msg->x;
    motion.linear.pos.y = msg->y;
    motion.linear.pos.z = msg->z;
    motion.linear.vel.x = msg->vx;
    motion.linear.vel.y = msg->vy;
    motion.linear.vel.z = msg->vz;
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "pos_cb received message");
}
void NED2ENU(base_env::msg::UAVMotion &_src, base_env::msg::UAVMotion &_target)
{
    using namespace Eigen;

    Matrix3d R = AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix(); // from ENU to NED
    Vector3d pos{_src.linear.pos.x, _src.linear.pos.y, _src.linear.pos.z};
    Vector3d vel{_src.linear.vel.x, _src.linear.vel.y, _src.linear.vel.z};
    Quaterniond q{_src.angular.q.w, _src.angular.q.x, _src.angular.q.y, _src.angular.q.z};

    pos = R * pos;
    vel = R * vel;
    q = Quaterniond(R).normalized() * q;

    _target.linear.pos.x = pos(0);
    _target.linear.pos.y = pos(1);
    _target.linear.pos.z = pos(2);
    _target.linear.vel.x = vel(0);
    _target.linear.vel.y = vel(1);
    _target.linear.vel.z = vel(2);
    _target.angular.q.x = q.x();
    _target.angular.q.y = q.y();
    _target.angular.q.z = q.z();
    _target.angular.q.w = q.w();
}
void ENU2NED(base_env::msg::UAVMotion &_src, base_env::msg::UAVMotion &_target)
{
}