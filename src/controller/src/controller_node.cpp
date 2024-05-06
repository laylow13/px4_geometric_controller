#include <chrono>
#include <memory>
#include <Eigen/Eigen>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "se3_controller/controller.hpp"
#include "base_env/msg/uav_motion.hpp"
#include "base_env/msg/uav_thrust.hpp"
#include "base_env/msg/uav_disturbance.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/hover_thrust_estimate.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

class Controller_node : public rclcpp::Node
{
public:
    Controller_node() : Node("controller_node"), is_posctl(false)
    {
        using std::placeholders::_1;
        using namespace std::chrono_literals;
        parameter_init();
        parameter_update();
        cmd_pub = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(cmd_topic, 10);
        offboard_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        thrust_pub = this->create_publisher<base_env::msg::UAVThrust>("SCIT_drone/UAV_thrust", 10);
        state_sub = this->create_subscription<base_env::msg::UAVMotion>(
            "/SCIT_drone/UAV_motion", 10, std::bind(&Controller_node::motion_sub_cb, this, _1));
        cmd_sub = this->create_subscription<base_env::msg::UAVMotion>(
            "/SCIT_drone/UAV_motion_expect", 10, std::bind(&Controller_node::motion_expect_sub_cb, this, _1));
        hte_sub = this->create_subscription<px4_msgs::msg::HoverThrustEstimate>(
            "/fmu/in/hover_thrust_estimate", 10, std::bind(&Controller_node::hte_sub_cb, this, _1));
        disturbance_sub = this->create_subscription<base_env::msg::UAVDisturbance>(
            "SCIT_drone/UAV_disturbance", 10, std::bind(&Controller_node::disturbance_sub_cb, this, _1));
        timesync_sub = this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", 10,
                                                                          [this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg)
                                                                          {
                                                                              //   timestamp.store(msg->timestamp);
                                                                          });
        mode_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", 10,
                                                                           [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg)
                                                                           {
                                                                               is_posctl = msg->nav_state == 2;
                                                                               is_offboard = msg->nav_state == 14;
                                                                           });
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(1000 / param.ctrl_rate)), std::bind(&Controller_node::timer_callback, this));
    }

private:
    bool is_posctl;
    bool is_offboard;
    std::atomic<uint64_t> timestamp;
    Controller controller;
    Parameter_t param;
    std::string cmd_topic;
    UAV_motion_t UAV_motion;
    UAV_motion_t UAV_motion_expect;
    attitude_sp_t att_sp;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr mode_sub;
    rclcpp::Subscription<base_env::msg::UAVMotion>::SharedPtr state_sub;
    rclcpp::Subscription<base_env::msg::UAVMotion>::SharedPtr cmd_sub;
    rclcpp::Subscription<base_env::msg::UAVDisturbance>::SharedPtr disturbance_sub;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub;
    rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hte_sub;
    rclcpp::Publisher<base_env::msg::UAVThrust>::SharedPtr thrust_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr cmd_pub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub;

    void parameter_init();
    void parameter_update();
    void timer_callback();
    void publish_offboard_control_mode();
    void publish_controller_cmd();
    void publish_thrust();
    void ENU2NED(attitude_sp_t &_src, attitude_sp_t &_target);
    void motion_sub_cb(const base_env::msg::UAVMotion::SharedPtr msg);
    void motion_expect_sub_cb(const base_env::msg::UAVMotion::SharedPtr msg);
    void hte_sub_cb(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg);
    void disturbance_sub_cb(const base_env::msg::UAVDisturbance::SharedPtr msg);
};

void Controller_node::parameter_init()
{
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
void Controller_node::parameter_update()
{
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
void Controller_node::timer_callback()
{
    parameter_update();
    timestamp.store(this->get_clock()->now().nanoseconds() / 1000);
    if (!is_posctl)
    {
        publish_controller_cmd();
        publish_offboard_control_mode();
        if (is_offboard)
        {
            publish_thrust();
            auto pos_err = UAV_motion.linear.pos - UAV_motion_expect.linear.pos;
            RCLCPP_INFO(this->get_logger(), "[pos_err] x:%.2f,y:%.2f,z:%.2f", pos_err(0), pos_err(1), pos_err(2));
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "Publishing Command!");
}
void Controller_node::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;
    msg.body_rate = false;
    msg.timestamp = timestamp.load();
    offboard_pub->publish(msg);
}
void Controller_node::publish_controller_cmd()
{
    px4_msgs::msg::VehicleAttitudeSetpoint UAV_setpoint;
    controller.run(UAV_motion_expect, UAV_motion, att_sp);
    attitude_sp_t att_sp_NED;
    ENU2NED(att_sp, att_sp_NED);
    UAV_setpoint.timestamp = timestamp.load();
    UAV_setpoint.thrust_body[2] = att_sp_NED.thrust;
    std::copy(std::begin(att_sp_NED.q), std::end(att_sp_NED.q), std::begin(UAV_setpoint.q_d));
    cmd_pub->publish(UAV_setpoint);
}
void Controller_node::publish_thrust()
{
    base_env::msg::UAVThrust msg;
    msg.timestamp = timestamp.load();
    msg.frame_type = base_env::msg::UAVThrust::ENU;
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
void Controller_node::ENU2NED(attitude_sp_t &_src, attitude_sp_t &_target)
{
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
void Controller_node::motion_sub_cb(const base_env::msg::UAVMotion::SharedPtr msg)
{
    UAV_motion.linear.pos << msg->linear.pos.x, msg->linear.pos.y, msg->linear.pos.z;
    UAV_motion.linear.vel << msg->linear.vel.x, msg->linear.vel.y, msg->linear.vel.z;
    UAV_motion.linear.acc << msg->linear.acc.x, msg->linear.acc.y, msg->linear.acc.z;
    UAV_motion.linear.jerk << msg->linear.jerk.x, msg->linear.jerk.y, msg->linear.jerk.z;
    UAV_motion.angular.q = Eigen::Quaterniond(msg->angular.q.w, msg->angular.q.x, msg->angular.q.y, msg->angular.q.z);
    UAV_motion.angular.vel << msg->angular.vel.x, msg->angular.vel.y, msg->angular.vel.z;
    UAV_motion.angular.acc << msg->angular.acc.x, msg->angular.acc.y, msg->angular.acc.z;
    // timestamp = msg->timestamp;
}
void Controller_node::motion_expect_sub_cb(const base_env::msg::UAVMotion::SharedPtr msg)
{
    UAV_motion_expect.linear.pos << msg->linear.pos.x, msg->linear.pos.y, msg->linear.pos.z;
    UAV_motion_expect.linear.vel << msg->linear.vel.x, msg->linear.vel.y, msg->linear.vel.z;
    UAV_motion_expect.linear.acc << msg->linear.acc.x, msg->linear.acc.y, msg->linear.acc.z;
    UAV_motion_expect.linear.jerk << msg->linear.jerk.x, msg->linear.jerk.y, msg->linear.jerk.z;
    UAV_motion_expect.angular.q = Eigen::Quaterniond(msg->angular.q.w, msg->angular.q.x, msg->angular.q.y, msg->angular.q.z);
    UAV_motion_expect.angular.vel << msg->angular.vel.x, msg->angular.vel.y, msg->angular.vel.z;
    UAV_motion_expect.angular.acc << msg->angular.acc.x, msg->angular.acc.y, msg->angular.acc.z;
}
void Controller_node::hte_sub_cb(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg)
{
    if (msg->valid)
    {
        param.hov_percent = msg->hover_thrust;
        controller.param.hov_percent = param.hov_percent;
        rclcpp::Parameter hte("hov_percent", param.hov_percent);
        this->set_parameter(hte);
    }
    RCLCPP_DEBUG(this->get_logger(), "hover_thrust updated!");
}
void Controller_node::disturbance_sub_cb(const base_env::msg::UAVDisturbance::SharedPtr msg)
{
    Eigen::Vector3d disturbance{msg->df.x, msg->df.y, msg->df.z};
    controller.setDisturbance(disturbance);
    RCLCPP_INFO(this->get_logger(), "[df]: %f,%f,%f", msg->df.x, msg->df.y, msg->df.z);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller_node>());
    rclcpp::shutdown();
    return 0;
}
