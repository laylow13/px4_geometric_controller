/*
get state information from px4 topic, and publish to controller after frame transformation
*/

#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "frame_transforms/frame_transforms.h"
#include "base_env/msg/uav_motion.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

using std::placeholders::_1;
base_env::msg::UAVMotion motion;
rclcpp::Publisher<base_env::msg::UAVMotion>::SharedPtr state_pub;
Eigen::Vector3d last_vel;
uint64_t last_timestamp;

void timer_callback();

void motion_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

void NED2ENU(base_env::msg::UAVMotion &_src, base_env::msg::UAVMotion &_target);

void ENU2NED(base_env::msg::UAVMotion &_src, base_env::msg::UAVMotion &_target);

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    auto sim_base = std::make_shared<rclcpp::Node>("state_data_convart");
    auto motion_sub = sim_base->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, &motion_cb);
    state_pub = sim_base->create_publisher<base_env::msg::UAVMotion>("/SCIT_drone/UAV_motion", 10);
    // TODO: change loop rate
    auto timer = sim_base->create_wall_timer(
            std::chrono::milliseconds(20),
            &timer_callback);
    rclcpp::spin(sim_base);
    rclcpp::shutdown();
    return 0;
}

void timer_callback() {
    base_env::msg::UAVMotion target;
    NED2ENU(motion, target);
    state_pub->publish(target);
    RCLCPP_DEBUG(rclcpp::get_logger("sim_base"), "[NED X Y Z]:%f,%f,%f\n[ENU X Y Z]:%f,%f,%f",
                 motion.linear.pos.x, motion.linear.pos.y, motion.linear.pos.z,
                 target.linear.pos.x, target.linear.pos.y, target.linear.pos.z);
    RCLCPP_DEBUG(rclcpp::get_logger("sim_base"), "Timer event");
}

void motion_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    motion.timestamp = msg->timestamp;
    double dt = (msg->timestamp - last_timestamp) * 1e-6;
    last_timestamp = msg->timestamp;
    motion.linear.pos.x = msg->position[0];
    motion.linear.pos.y = msg->position[1];
    motion.linear.pos.z = msg->position[2];
    motion.linear.vel.x = msg->velocity[0];
    motion.linear.vel.y = msg->velocity[1];
    motion.linear.vel.z = msg->velocity[2];
    motion.linear.acc.x = (msg->velocity[0] - last_vel(0)) / dt;
    motion.linear.acc.y = (msg->velocity[1] - last_vel(1)) / dt;
    motion.linear.acc.z = (msg->velocity[2] - last_vel(2)) / dt;
    last_vel << msg->velocity[0], msg->velocity[1], msg->velocity[2];
    motion.angular.q.w = msg->q[0];
    motion.angular.q.x = msg->q[1];
    motion.angular.q.y = msg->q[2];
    motion.angular.q.z = msg->q[3];
    RCLCPP_DEBUG(rclcpp::get_logger("sim_base"), "Received message");
}

void NED2ENU(base_env::msg::UAVMotion &_src, base_env::msg::UAVMotion &_target) {
    /*NED2ENU map: Controller_local:ENU -> PX4_local:NED ->  PX4_body:FRD/NED -> Controller_body:FLU/ENU
        _src: PX4_local:NED ->  PX4_body:FRD/NED
        _target: Controller_local: ENU -> PX4_local:NED ->  PX4_body:FRD/NED -> Controller_body:FLU/ENU*/
    using namespace Eigen;

    _target = _src;

    Matrix3d R = AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix(); // from ENU to NED
    Vector3d pos{_src.linear.pos.x, _src.linear.pos.y, _src.linear.pos.z};
    Vector3d vel{_src.linear.vel.x, _src.linear.vel.y, _src.linear.vel.z};
    Vector3d acc{_src.linear.acc.x, _src.linear.acc.y, _src.linear.acc.z};
    Quaterniond q{_src.angular.q.w, _src.angular.q.x, _src.angular.q.y, _src.angular.q.z};

    pos = R * pos;
    vel = R * vel;
    acc = R * acc;
    q = Quaterniond(R).normalized() * q * Quaterniond(R.transpose()).normalized();

    _target.linear.pos.x = pos(0);
    _target.linear.pos.y = pos(1);
    _target.linear.pos.z = pos(2);
    _target.linear.vel.x = vel(0);
    _target.linear.vel.y = vel(1);
    _target.linear.vel.z = vel(2);
    _target.linear.acc.x = acc(0);
    _target.linear.acc.y = acc(1);
    _target.linear.acc.z = acc(2);
    _target.angular.q.x = q.x();
    _target.angular.q.y = q.y();
    _target.angular.q.z = q.z();
    _target.angular.q.w = q.w();
}

void ENU2NED(base_env::msg::UAVMotion &_src, base_env::msg::UAVMotion &_target) {
}