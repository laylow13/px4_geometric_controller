#include <chrono>
#include <memory>
#include <Eigen/Eigen>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "se3_controller/controller.hpp"
#include "base_env/msg/uav_motion.hpp"
#include "base_env/msg/uav_thrust.hpp"
#include "base_env/msg/uav_disturbance.hpp"
#include "px4_msgs/msg/timesync.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/hover_thrust_estimate.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

class Controller_node : public rclcpp::Node
{
public:
    Controller_node() : Node("controller_node")
    {
        parameter_init();
        controller.param = param;
        cmd_pub = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(cmd_topic, 10);
        offboard_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/offboard_control_mode/in", 10);
        thrust_pub = this->create_publisher<base_env::msg::UAVThrust>("SCIT_drone/UAV_thrust", 10);
        state_sub = this->create_subscription<base_env::msg::UAVMotion>(
            "/SCIT_drone/UAV_motion", 10, std::bind(&Controller_node::motion_sub_cb, this, _1));
        cmd_sub = this->create_subscription<base_env::msg::UAVMotion>(
            "/SCIT_drone/UAV_motion_expect", 10, std::bind(&Controller_node::motion_expect_sub_cb, this, _1));
        hte_sub = this->create_subscription<px4_msgs::msg::HoverThrustEstimate>(
            "/fmu/hover_thrust_estimate/out", 10, std::bind(&Controller_node::hte_sub_cb, this, _1));
        disturbance_sub = this->create_subscription<base_env::msg::UAVDisturbance>(
            "SCIT_drone/UAV_disturbance", 10, std::bind(&Controller_node::disturbance_sub_cb, this, _1));
        timesync_sub = this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
                                                                          [this](const px4_msgs::msg::Timesync::UniquePtr msg)
                                                                          {
                                                                              timestamp.store(msg->timestamp);
                                                                          });
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(1000 / param.ctrl_rate)), std::bind(&Controller_node::timer_callback, this));
    }

private:
    std::atomic<uint64_t> timestamp;
    Controller controller;
    Parameter_t param;
    std::string cmd_topic;
    UAV_motion_t UAV_motion;
    UAV_motion_t UAV_motion_expect;
    attitude_sp_t att_sp;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<base_env::msg::UAVMotion>::SharedPtr state_sub;
    rclcpp::Subscription<base_env::msg::UAVMotion>::SharedPtr cmd_sub;
    rclcpp::Subscription<base_env::msg::UAVDisturbance>::SharedPtr disturbance_sub;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub;
    rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hte_sub;
    rclcpp::Publisher<base_env::msg::UAVThrust>::SharedPtr thrust_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr cmd_pub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub;

    void publish_offboard_control_mode()
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
    void publish_controller_cmd()
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
    void publish_thrust()
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
    void ENU2NED(attitude_sp_t &_src, attitude_sp_t &_target)
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
    void parameter_init()
    {
        this->declare_parameter<double>("Kp0", 0);
        this->declare_parameter<double>("Kp1", 0);
        this->declare_parameter<double>("Kp2", 0);
        this->declare_parameter<double>("Kv0", 0);
        this->declare_parameter<double>("Kv1", 0);
        this->declare_parameter<double>("Kv2", 0);
        this->declare_parameter<double>("Kvi0", 0);
        this->declare_parameter<double>("Kvi1", 0);
        this->declare_parameter<double>("Kvi2", 0);
        this->declare_parameter<double>("Ka0", 0);
        this->declare_parameter<double>("Ka1", 0);
        this->declare_parameter<double>("Ka2", 0);
        this->declare_parameter<double>("mass", 0);
        this->declare_parameter<double>("gra", 0);
        this->declare_parameter<double>("hov_percent", 0);
        this->declare_parameter<double>("i_limit_max", 0);
        this->declare_parameter<double>("i_speed_max", 0);
        this->declare_parameter<double>("ctrl_rate", 50);
        this->declare_parameter<std::string>("cmd_topic", "/fmu/vehicle_attitude_setpoint/in");

        param.Kp(0, 0) = this->get_parameter("Kp0").get_value<double>();
        param.Kp(1, 1) = this->get_parameter("Kp1").get_value<double>();
        param.Kp(2, 2) = this->get_parameter("Kp2").get_value<double>();
        param.Kv(0, 0) = this->get_parameter("Kv0").get_value<double>();
        param.Kv(1, 1) = this->get_parameter("Kv1").get_value<double>();
        param.Kv(2, 2) = this->get_parameter("Kv2").get_value<double>();
        param.Kvi(0, 0) = this->get_parameter("Kvi0").get_value<double>();
        param.Kvi(1, 1) = this->get_parameter("Kvi1").get_value<double>();
        param.Kvi(2, 2) = this->get_parameter("Kvi2").get_value<double>();
        param.Ka(0, 0) = this->get_parameter("Ka0").get_value<double>();
        param.Ka(1, 1) = this->get_parameter("Ka1").get_value<double>();
        param.Ka(2, 2) = this->get_parameter("Ka2").get_value<double>();
        param.mass = this->get_parameter("mass").get_value<double>();
        param.gra = this->get_parameter("gra").get_value<double>();
        param.hov_percent = this->get_parameter("hov_percent").get_value<double>();
        param.i_limit_max = this->get_parameter("i_limit_max").get_value<double>();
        param.i_speed_max = this->get_parameter("i_speed_max").get_value<double>();
        param.ctrl_rate = this->get_parameter("ctrl_rate").get_value<double>();
        cmd_topic = this->get_parameter("cmd_topic").get_value<std::string>();
    }
    void motion_sub_cb(const base_env::msg::UAVMotion &msg)
    {
        UAV_motion.linear.pos(0) = msg.linear.pos.x;
        UAV_motion.linear.pos(1) = msg.linear.pos.y;
        UAV_motion.linear.pos(2) = msg.linear.pos.z;

        UAV_motion.linear.vel(0) = msg.linear.vel.x;
        UAV_motion.linear.vel(1) = msg.linear.vel.y;
        UAV_motion.linear.vel(2) = msg.linear.vel.z;

        UAV_motion.linear.acc(0) = msg.linear.acc.x;
        UAV_motion.linear.acc(1) = msg.linear.acc.y;
        UAV_motion.linear.acc(2) = msg.linear.acc.z - param.gra;

        UAV_motion.linear.jerk(0) = msg.linear.jerk.x;
        UAV_motion.linear.jerk(1) = msg.linear.jerk.y;
        UAV_motion.linear.jerk(2) = msg.linear.jerk.z;

        UAV_motion.angular.q.x() = msg.angular.q.x;
        UAV_motion.angular.q.y() = msg.angular.q.y;
        UAV_motion.angular.q.z() = msg.angular.q.z;
        UAV_motion.angular.q.w() = msg.angular.q.w;

        UAV_motion.angular.vel(0) = msg.angular.vel.x;
        UAV_motion.angular.vel(1) = msg.angular.vel.y;
        UAV_motion.angular.vel(2) = msg.angular.vel.z;

        UAV_motion.angular.acc(0) = msg.angular.acc.x;
        UAV_motion.angular.acc(1) = msg.angular.acc.y;
        UAV_motion.angular.acc(2) = msg.angular.acc.z;
        // timestamp = msg.timestamp;
    }
    void motion_expect_sub_cb(const base_env::msg::UAVMotion &msg)
    {
        UAV_motion_expect.linear.pos(0) = msg.linear.pos.x;
        UAV_motion_expect.linear.pos(1) = msg.linear.pos.y;
        UAV_motion_expect.linear.pos(2) = msg.linear.pos.z;

        UAV_motion_expect.linear.vel(0) = msg.linear.vel.x;
        UAV_motion_expect.linear.vel(1) = msg.linear.vel.y;
        UAV_motion_expect.linear.vel(2) = msg.linear.vel.z;

        UAV_motion_expect.linear.acc(0) = msg.linear.acc.x;
        UAV_motion_expect.linear.acc(1) = msg.linear.acc.y;
        UAV_motion_expect.linear.acc(2) = msg.linear.acc.z;

        UAV_motion_expect.linear.jerk(0) = msg.linear.jerk.x;
        UAV_motion_expect.linear.jerk(1) = msg.linear.jerk.y;
        UAV_motion_expect.linear.jerk(2) = msg.linear.jerk.z;

        UAV_motion_expect.angular.q.x() = msg.angular.q.x;
        UAV_motion_expect.angular.q.y() = msg.angular.q.y;
        UAV_motion_expect.angular.q.z() = msg.angular.q.z;
        UAV_motion_expect.angular.q.w() = msg.angular.q.w;

        UAV_motion_expect.angular.vel(0) = msg.angular.vel.x;
        UAV_motion_expect.angular.vel(1) = msg.angular.vel.y;
        UAV_motion_expect.angular.vel(2) = msg.angular.vel.z;

        UAV_motion_expect.angular.acc(0) = msg.angular.acc.x;
        UAV_motion_expect.angular.acc(1) = msg.angular.acc.y;
        UAV_motion_expect.angular.acc(2) = msg.angular.acc.z;
    }
    void hte_sub_cb(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg)
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
    void disturbance_sub_cb(const base_env::msg::UAVDisturbance::SharedPtr msg)
    {
        Eigen::Vector3d disturbance{msg->df.x, msg->df.y, msg->df.z};
        controller.setDisturbance(disturbance);
        RCLCPP_INFO(this->get_logger(), "[df]: %f,%f,%f",msg->df.x, msg->df.y, msg->df.z);
    }
    void timer_callback()
    {
        publish_controller_cmd();
        publish_offboard_control_mode();
        publish_thrust();
        RCLCPP_DEBUG(this->get_logger(), "Publishing Command!");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller_node>());
    rclcpp::shutdown();
    return 0;
}