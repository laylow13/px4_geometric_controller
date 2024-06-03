//
// Created by lay on 24-5-15.
//
#include <chrono>
#include <memory>
#include <Eigen/Eigen>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "se3_controller/controller.hpp"
#include "utils/msg/uav_motion.hpp"
#include "utils/msg/uav_thrust.hpp"
#include "utils/msg/uav_disturbance.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/hover_thrust_estimate.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_thrust_setpoint.hpp"
#include "px4_msgs/msg/vehicle_torque_setpoint.hpp"
#include "fdcl/control.hpp"


class Controller_node : public rclcpp::Node {
public:
    Controller_node() : Node("geometric_controller_node"), is_posctl(false) {
        using std::placeholders::_1;
        using namespace std::chrono_literals;
        geo_state = new fdcl::state_t();
        geo_command = new fdcl::command_t();
        config_file = new fdcl::param();
        config_file->open("../uav.cfg");
        geometric_controller = fdcl::control(geo_state, geo_command, config_file);
        thrust_cmd_pub = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
                "/fmu/in/vehicle_thrust_setpoint", 10);
        torque_cmd_pub = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
                "/fmu/in/vehicle_torque_setpoint", 10);
        att_cmd_pub = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(cmd_topic, 10);
        offboard_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        thrust_pub = this->create_publisher<utils::msg::UAVThrust>("SCIT_drone/UAV_thrust", 10);
        state_sub = this->create_subscription<utils::msg::UAVMotion>(
                "/SCIT_drone/UAV_motion", 10, std::bind(&Controller_node::motion_sub_cb, this, _1));
        cmd_sub = this->create_subscription<utils::msg::UAVMotion>(
                "/SCIT_drone/UAV_motion_expect", 10, std::bind(&Controller_node::motion_expect_sub_cb, this, _1));
        hte_sub = this->create_subscription<px4_msgs::msg::HoverThrustEstimate>(
                "/fmu/in/hover_thrust_estimate", 10, std::bind(&Controller_node::hte_sub_cb, this, _1));
        disturbance_sub = this->create_subscription<utils::msg::UAVDisturbance>(
                "SCIT_drone/UAV_disturbance", 10, std::bind(&Controller_node::disturbance_sub_cb, this, _1));
        timesync_sub = this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", 10,
                                                                                [this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
                                                                                    //   timestamp.store(msg->timestamp);
                                                                                });
        mode_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", 10,
                                                                           [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
                                                                               is_posctl = msg->nav_state == 2;
                                                                               is_offboard = msg->nav_state == 14;
                                                                           });
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(int(1000 / param.ctrl_rate)),
                std::bind(&Controller_node::timer_callback, this));
    }

    ~ Controller_node() {
        delete geo_state;
        delete geo_command;
        delete config_file;
    }

private:
    bool is_posctl;
    bool is_offboard;
    std::atomic<uint64_t> timestamp;
    Controller controller;
    fdcl::state_t *geo_state;
    fdcl::command_t *geo_command;
    fdcl::param *config_file;
    fdcl::control geometric_controller;
    Parameter_t param;
    std::string cmd_topic;
    state_t state;
    command_t command;
    UAV_motion_t UAV_motion;
    UAV_motion_t UAV_motion_expect;
    attitude_sp_t att_sp;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr mode_sub;
    rclcpp::Subscription<utils::msg::UAVMotion>::SharedPtr state_sub;
    rclcpp::Subscription<utils::msg::UAVMotion>::SharedPtr cmd_sub;
    rclcpp::Subscription<utils::msg::UAVDisturbance>::SharedPtr disturbance_sub;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub;
    rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hte_sub;
    rclcpp::Publisher<utils::msg::UAVThrust>::SharedPtr thrust_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr att_cmd_pub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_cmd_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_cmd_pub;

    void parameter_init();

    void parameter_update();

    void timer_callback();

    void publish_offboard_control_mode();

    void publish_controller_cmd();

    void publish_thrust();

    void ENU2NED(attitude_sp_t &_src, attitude_sp_t &_target);

    void motion_sub_cb(const utils::msg::UAVMotion::SharedPtr msg);

    void motion_expect_sub_cb(const utils::msg::UAVMotion::SharedPtr msg);

    void hte_sub_cb(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg);

    void disturbance_sub_cb(const utils::msg::UAVDisturbance::SharedPtr msg);
};

void Controller_node::parameter_init() {
    this->declare_parameter<double>("Kp0", 0);
    this->declare_parameter<double>("Kp1", 0);
    this->declare_parameter<double>("Kp2", 0);
    this->declare_parameter<double>("Kv0", 0);
    this->declare_parameter<double>("Kv1", 0);
    this->declare_parameter<double>("Kv2", 0);
    this->declare_parameter<double>("Ki0", 0);
    this->declare_parameter<double>("Ki1", 0);
    this->declare_parameter<double>("Ki2", 0);
    this->declare_parameter<double>("mass", 0);
    this->declare_parameter<double>("gra", 0);
    this->declare_parameter<double>("hov_percent", 0);
    this->declare_parameter<double>("i_limit_max", 0);
    this->declare_parameter<double>("i_c", 0);
    this->declare_parameter<double>("ctrl_rate", 50);
    this->declare_parameter<std::string>("cmd_topic", "/fmu/in/vehicle_attitude_setpoint");

    param.ctrl_rate = this->get_parameter("ctrl_rate").get_value<double>();
    cmd_topic = this->get_parameter("cmd_topic").get_value<std::string>();
}

void Controller_node::parameter_update() {
    param.Kp(0, 0) = this->get_parameter("Kp0").get_value<double>();
    param.Kp(1, 1) = this->get_parameter("Kp1").get_value<double>();
    param.Kp(2, 2) = this->get_parameter("Kp2").get_value<double>();
    param.Kv(0, 0) = this->get_parameter("Kv0").get_value<double>();
    param.Kv(1, 1) = this->get_parameter("Kv1").get_value<double>();
    param.Kv(2, 2) = this->get_parameter("Kv2").get_value<double>();
    param.Ki(0, 0) = this->get_parameter("Ki0").get_value<double>();
    param.Ki(1, 1) = this->get_parameter("Ki1").get_value<double>();
    param.Ki(2, 2) = this->get_parameter("Ki2").get_value<double>();

    param.mass = this->get_parameter("mass").get_value<double>();
    param.gra = this->get_parameter("gra").get_value<double>();
    param.hov_percent = this->get_parameter("hov_percent").get_value<double>();
    param.i_limit_max = this->get_parameter("i_limit_max").get_value<double>();
    param.i_c = this->get_parameter("i_c").get_value<double>();
    controller.param = param;
}

void Controller_node::timer_callback() {
    parameter_update();
    timestamp.store(this->get_clock()->now().nanoseconds() / 1000);
    if (!is_posctl) {
        publish_controller_cmd();
        publish_offboard_control_mode();
        if (is_offboard) {
            publish_thrust();
            auto pos_err = UAV_motion.linear.pos - UAV_motion_expect.linear.pos;
            RCLCPP_INFO(this->get_logger(), "[pos_err] x:%.2f,y:%.2f,z:%.2f", pos_err(0), pos_err(1), pos_err(2));
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "Publishing Command!");
}

void Controller_node::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.actuator = true;
    msg.timestamp = timestamp.load();
    offboard_pub->publish(msg);
}

void Controller_node::publish_controller_cmd() {
//    state->x << 0.0, 0.0, i;
//    state->v.setZero();
//    state->a.setZero();
//    state->R.setIdentity();
//    state->W.setZero();
//
//    command->xd.setZero();
//    command->xd_dot.setZero();
//    command->xd_2dot.setZero();
//    command->xd_3dot.setZero();
//    command->xd_4dot.setZero();
//    command->b1d << 1.0, 0.0, 0.0;
//    command->b1d_dot.setZero();
//    command->b1d_ddot.setZero();
    double f_out;
    Vector3 M_out;
    geometric_controller.position_control();
    geometric_controller.output_fM(f_out, M_out);

    px4_msgs::msg::VehicleThrustSetpoint thrust_sp{};
    thrust_sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    thrust_sp.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
    thrust_sp.xyz = {0, 0, -0.8};
    thrust_cmd_pub->publish(thrust_sp);
    px4_msgs::msg::VehicleTorqueSetpoint torque_sp{};
    torque_sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    torque_sp.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
    torque_sp.xyz = {0, 0, 0.01};
    torque_cmd_pub->publish(torque_sp);
}

void Controller_node::publish_thrust() {
    utils::msg::UAVThrust msg;
    msg.timestamp = timestamp.load();
    msg.frame_type = utils::msg::UAVThrust::ENU;
    Eigen::Vector3d thrust = controller.getDesiredThrust();
    msg.thrust.x = thrust(0);
    msg.thrust.y = thrust(1);
    msg.thrust.z = thrust(2);
    thrust_pub->publish(msg);
}

/*
@brief ENU2NED() map: PX4_local:NED -> Controller_local:ENU -> Controller_body:FLU/ENU ->  PX4_body:FRD/NED
    _src: Controller_local:ENU -> Controller_body:FLU/ENU
    _target: PX4_local:NED ->  PX4_body:FRD/NED
*/
void Controller_node::ENU2NED(attitude_sp_t &_src, attitude_sp_t &_target) {
    using namespace Eigen;

    Matrix3d R = AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix(); // from NED to ENU
    Quaterniond q{_src.q[0], _src.q[1], _src.q[2], _src.q[3]};

    q = Quaterniond(R).normalized() * q * Quaterniond(R.transpose()).normalized();

    _target.thrust = -_src.thrust;
    _target.q[0] = q.w();
    _target.q[1] = q.x();
    _target.q[2] = q.y();
    _target.q[3] = q.z();
}

void Controller_node::motion_sub_cb(const utils::msg::UAVMotion::SharedPtr msg) {
    state.x << msg->linear.pos.x, msg->linear.pos.y, msg->linear.pos.z;
    state.x_dot << msg->linear.vel.x, msg->linear.vel.y, msg->linear.vel.z;
    state.x_2dot << msg->linear.acc.x, msg->linear.acc.y, msg->linear.acc.z;
    state.q = Eigen::Quaterniond(msg->angular.q.w, msg->angular.q.x, msg->angular.q.y, msg->angular.q.z);
    state.w << msg->angular.vel.x, msg->angular.vel.y, msg->angular.vel.z;
}

void Controller_node::motion_expect_sub_cb(const utils::msg::UAVMotion::SharedPtr msg) {
    command.x << msg->linear.pos.x, msg->linear.pos.y, msg->linear.pos.z;
    command.x_dot << msg->linear.vel.x, msg->linear.vel.y, msg->linear.vel.z;
    command.x_2dot << msg->linear.acc.x, msg->linear.acc.y, msg->linear.acc.z;
    command.x_3dot << msg->linear.jerk.x, msg->linear.jerk.y, msg->linear.jerk.z;
//    command.x_4dot<<;
//    command.b1d <<
//    command.b1d_dot <<
}

void Controller_node::hte_sub_cb(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg) {
    if (msg->valid) {
        param.hov_percent = msg->hover_thrust;
        controller.param.hov_percent = param.hov_percent;
        rclcpp::Parameter hte("hov_percent", param.hov_percent);
        this->set_parameter(hte);
    }
    RCLCPP_DEBUG(this->get_logger(), "hover_thrust updated!");
}

void Controller_node::disturbance_sub_cb(const utils::msg::UAVDisturbance::SharedPtr msg) {
    Eigen::Vector3d disturbance{msg->df.x, msg->df.y, msg->df.z};
    controller.setDisturbance(disturbance);
    RCLCPP_INFO(this->get_logger(), "[df]: %f,%f,%f", msg->df.x, msg->df.y, msg->df.z);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller_node>());
    rclcpp::shutdown();
    return 0;
}
