//
// Created by lay on 24-5-15.
//
#include <chrono>
#include <memory>
#include <Eigen/Eigen>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "utils/msg/uav_command.hpp"
#include "utils/msg/uav_state_feedback.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_thrust_setpoint.hpp"
#include "px4_msgs/msg/vehicle_torque_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "geometric_control/Geometric_control.hpp"
#include "FTDO/FTDO.h"
#include "EKF_estimator/EKF_estimator.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Controller_node : public rclcpp::Node {
public:
    enum control_mode_t {
        ATTITUDE,
        ANG_RATE,
        TORQUE
    };

    Controller_node() : Node("geometric_controller_node"), is_posctl(false) {
        state = std::make_shared<state_t>();
        command = std::make_shared<command_t>();
        geometric_param = std::make_shared<geometric_param_t>();
        controller = std::make_shared<Geometric_control>(command, state, geometric_param);
//        controller->init(command, state, geometric_param);
        parameter_init();
        parameter_update();
        att_cmd_pub = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
                "/fmu/in/vehicle_attitude_setpoint", 10);
        rate_cmd_pub = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
                "/fmu/in/vehicle_rates_setpoint", 10);
        thrust_cmd_pub = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
                "/fmu/in/vehicle_thrust_setpoint", 10);
        torque_cmd_pub = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
                "/fmu/in/vehicle_torque_setpoint", 10);
        offboard_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        state_sub = this->create_subscription<utils::msg::UAVStateFeedback>(
                "/SCIT_drone/UAV_state_feedback", 10, std::bind(&Controller_node::state_sub_cb, this, _1));
        command_sub = this->create_subscription<utils::msg::UAVCommand>(
                "/SCIT_drone/UAV_trajectory_command", 10, std::bind(&Controller_node::command_sub_cb, this, _1));
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        mode_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos,
                                                                           [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
                                                                               is_posctl = msg->nav_state == 2;
                                                                               is_offboard = msg->nav_state == 14;
                                                                           });
        parameter_event_sub = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
                "/parameter_events", 10, std::bind(&Controller_node::parameter_cb, this, _1));

        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(int(1000 / geometric_param->frequency)),
                std::bind(&Controller_node::timer_callback, this));
    }

private:
    bool is_posctl;
    bool is_offboard;
    EKF_estimator param_estimator;
    std::map<std::string, double> ros_parameters_;
    std::atomic<uint64_t> timestamp;
    shared_ptr<Geometric_control> controller;
    shared_ptr<command_t> command;
    shared_ptr<state_t> state;
    shared_ptr<geometric_param_t> geometric_param;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr mode_sub;
    rclcpp::Subscription<utils::msg::UAVCommand>::SharedPtr command_sub;
    rclcpp::Subscription<utils::msg::UAVStateFeedback>::SharedPtr state_sub;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_cmd_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_cmd_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr att_cmd_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rate_cmd_pub;

    void parameter_init();

    void parameter_update();

    void timer_callback();

    void publish_offboard_control_mode(control_mode_t mode);

    void publish_thrust_torque_cmd();

    void publish_thrust_attitude_cmd();

    void publish_thrust_angular_rate_cmd();

    void state_sub_cb(const utils::msg::UAVStateFeedback::SharedPtr msg);

    void command_sub_cb(const utils::msg::UAVCommand::SharedPtr msg);

    void parameter_cb(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

};

void Controller_node::parameter_init() {
    ros_parameters_ = {{"J1",                1.0},
                       {"J2",                1.0},
                       {"J3",                1.0},
                       {"m",                 1.0},
                       {"g",                 9.81},
                       {"thrust_scale",      1.0},
                       {"torque_scale",      1.0},
                       {"frequency",         100.0},
                       {"c_tf",              0.0},
                       {"l",                 0.0},
                       {"use_decoupled_yaw", 0.0},
                       {"kX1",               1.0},
                       {"kX2",               1.0},
                       {"kX3",               1.0},
                       {"kV1",               1.0},
                       {"kV2",               1.0},
                       {"kV3",               1.0},
                       {"kR1",               1.0},
                       {"kR2",               1.0},
                       {"kR3",               1.0},
                       {"kW1",               1.0},
                       {"kW2",               1.0},
                       {"kW3",               1.0},
                       {"use_integral",      1.0},
                       {"int_limit",         0.0},
                       {"kIX",               0.0},
                       {"kIR",               0.0},
                       {"kI ",               0.0},
                       {"kyI",               0.0},
                       {"c1 ",               0.0},
                       {"c2 ",               0.0},
                       {"c3 ",               0.0},
                       {"attctrl_tau",       1.0},

    };
    this->declare_parameters<double>("geometric_control", ros_parameters_);
}

void Controller_node::parameter_update() {
    if (this->get_parameters<double>("geometric_control", ros_parameters_)) {
        geometric_param->J.diagonal() << ros_parameters_["J1"], ros_parameters_["J2"], ros_parameters_["J3"];
        geometric_param->m = ros_parameters_["m"];
        geometric_param->g = ros_parameters_["g"];
        geometric_param->c_tf = ros_parameters_["c_tf"];
        geometric_param->l = ros_parameters_["l"];
        geometric_param->thrust_scale = ros_parameters_["thrust_scale"];
        geometric_param->torque_scale = ros_parameters_["torque_scale"];
        geometric_param->frequency = ros_parameters_["frequency"];
        geometric_param->use_decoupled_yaw = int(ros_parameters_["use_decoupled_yaw"]);
        geometric_param->kX.diagonal() << ros_parameters_["kX1"], ros_parameters_["kX2"], ros_parameters_["kX3"];
        geometric_param->kV.diagonal() << ros_parameters_["kV1"], ros_parameters_["kV2"], ros_parameters_["kV3"];
        geometric_param->kR.diagonal() << ros_parameters_["kR1"], ros_parameters_["kR2"], ros_parameters_["kR3"];
        geometric_param->kW.diagonal() << ros_parameters_["kW1"], ros_parameters_["kW2"], ros_parameters_["kW3"];

        geometric_param->use_integral = int(ros_parameters_["use_integral"]);
        geometric_param->int_limit = ros_parameters_["int_limit"];
        geometric_param->kIX = ros_parameters_["kIX"];
        geometric_param->kIR = ros_parameters_["kIR"];
        geometric_param->kI = ros_parameters_["kI"];
        geometric_param->kyI = ros_parameters_["kyI"];
        geometric_param->c1 = ros_parameters_["c1"];
        geometric_param->c2 = ros_parameters_["c2"];
        geometric_param->c3 = ros_parameters_["c3"];

        geometric_param->attctrl_tau = ros_parameters_["attctrl_tau"];
    }
}

void Controller_node::timer_callback() {
    timestamp.store(this->get_clock()->now().nanoseconds() / 1000);
    if (!is_posctl) {
        publish_thrust_torque_cmd();
//        publish_thrust_attitude_cmd();
//        publish_thrust_angular_rate_cmd();
        publish_offboard_control_mode(TORQUE);

    }
}

void Controller_node::publish_offboard_control_mode(control_mode_t mode) {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = mode == ATTITUDE;
    msg.body_rate = mode == ANG_RATE;
    msg.actuator = mode == TORQUE;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_pub->publish(msg);
}

void Controller_node::publish_thrust_torque_cmd() {
    double thrust_cmd;
    Vector3d torque_cmd;
    controller->compute_control_output();
    controller->get_fM_cmd(thrust_cmd, torque_cmd, false);
    RCLCPP_INFO(this->get_logger(), "[raw f]:%.2f [raw M]:%.2f,%.2f,%.2f", thrust_cmd, torque_cmd(0), torque_cmd(1),
                torque_cmd(2));
    controller->get_fM_cmd(thrust_cmd, torque_cmd, true);
    RCLCPP_INFO(this->get_logger(), "[f]:%.2f [M]:%.2f,%.2f,%.2f", thrust_cmd, torque_cmd(0), torque_cmd(1),
                torque_cmd(2));
    Vector3d eX, eV, eR, eW;
    controller->get_positional_tracking_error(eX, eV);
    RCLCPP_INFO(this->get_logger(), "[eX]:%.2f,%.2f,%.2f [eV]:%.2f,%.2f,%.2f", eX(0), eX(1), eX(2), eV(0), eV(1),
                eV(2));
    controller->get_rotational_tracking_error(eR, eW);
    RCLCPP_INFO(this->get_logger(), "[eR]:%.2f,%.2f,%.2f [eW]:%.2f,%.2f,%.2f", eR(0), eR(1), eR(2), eW(0), eW(1),
                eW(2));
    px4_msgs::msg::VehicleThrustSetpoint thrust_sp{};
    thrust_sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    thrust_sp.timestamp_sample = state->timestamp.sec * uint64_t(1e6) + state->timestamp.nanosec / 1000;
    thrust_sp.xyz[0] = 0;
    thrust_sp.xyz[1] = 0;
    thrust_sp.xyz[2] = thrust_cmd;
    thrust_cmd_pub->publish(thrust_sp);
    px4_msgs::msg::VehicleTorqueSetpoint torque_sp{};
    torque_sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    torque_sp.timestamp_sample = state->timestamp.sec * uint64_t(1e6) + state->timestamp.nanosec / 1000;
    torque_sp.xyz[0] = torque_cmd(0);
    torque_sp.xyz[1] = torque_cmd(1);
    torque_sp.xyz[2] = torque_cmd(2);
    torque_cmd_pub->publish(torque_sp);
}

void Controller_node::publish_thrust_attitude_cmd() {
    double thrust_cmd;
    Quaterniond att_cmd;
    controller->compute_control_output();
    controller->get_attitude_cmd(thrust_cmd, att_cmd, true);
    px4_msgs::msg::VehicleAttitudeSetpoint att_sp;
    att_sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    att_sp.thrust_body[2] = thrust_cmd;
    att_sp.q_d[0] = att_cmd.w();
    att_sp.q_d[1] = att_cmd.x();
    att_sp.q_d[2] = att_cmd.y();
    att_sp.q_d[3] = att_cmd.z();
    att_cmd_pub->publish(att_sp);
}

void Controller_node::publish_thrust_angular_rate_cmd() {
    double thrust_cmd;
    Vector3d ang_rate_cmd;
    controller->compute_control_output();
    controller->get_angular_velocity_cmd(thrust_cmd, ang_rate_cmd, true);
    px4_msgs::msg::VehicleRatesSetpoint rate_sp;
    rate_sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    rate_sp.thrust_body[2] = thrust_cmd;
    rate_sp.roll = ang_rate_cmd(0);
    rate_sp.pitch = ang_rate_cmd(1);
    rate_sp.yaw = ang_rate_cmd(2);
    rate_cmd_pub->publish(rate_sp);

    RCLCPP_INFO(this->get_logger(), "[f]:%.2f [rate]:%.2f,%.2f,%.2f", thrust_cmd, ang_rate_cmd(0), ang_rate_cmd(1),
                ang_rate_cmd(2));
}

void Controller_node::state_sub_cb(const utils::msg::UAVStateFeedback::SharedPtr msg) {
    state->timestamp.sec = msg->header.stamp.sec;
    state->timestamp.nanosec = msg->header.stamp.nanosec;
    state->world_frame = msg->world_frame;
    state->body_frame = msg->body_frame;
    state->x << msg->pos.x, msg->pos.y, msg->pos.z;
    state->x_dot << msg->vel.x, msg->vel.y, msg->vel.z;
    state->x_2dot << msg->acc.x, msg->acc.y, msg->acc.z;
    state->q = Quaterniond(msg->q.w, msg->q.x, msg->q.y, msg->q.z);
    state->w << msg->ang_vel.x, msg->ang_vel.y, msg->ang_vel.z;
}

void Controller_node::command_sub_cb(const utils::msg::UAVCommand::SharedPtr msg) {
    command->timestamp.sec = msg->header.stamp.sec;
    command->timestamp.nanosec = msg->header.stamp.nanosec;
    command->world_frame = msg->world_frame;
    command->body_frame = msg->body_frame;
    command->x << msg->pos.x, msg->pos.y, msg->pos.z;
    command->x_dot << msg->vel.x, msg->vel.y, msg->vel.z;
    command->x_2dot << msg->acc.x, msg->acc.y, msg->acc.z;
    command->x_3dot << msg->jerk.x, msg->jerk.y, msg->jerk.z;
    command->x_4dot << msg->snap.x, msg->snap.y, msg->snap.z;
    command->b1d << msg->heading.x, msg->heading.y, msg->heading.z;
    command->b1d_dot << msg->heading_dot.x, msg->heading_dot.y, msg->heading_dot.z;
    command->b1d_2dot << msg->heading_2dot.x, msg->heading_2dot.y, msg->heading_2dot.z;
}

void Controller_node::parameter_cb(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
    parameter_update();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller_node>());
    rclcpp::shutdown();
    return 0;
}
