#include <memory>
#include <chrono>
#include <Eigen/Eigen>
#include <array>
#include "lpf1st.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/msg/uav_motion.hpp"
#include "utils/msg/uav_thrust.hpp"
#include "utils/msg/uav_disturbance.hpp"

using std::placeholders::_1;

class DOB_node : public rclcpp::Node
{
public:
    DOB_node() : Node("DOB"), df_filter({{{0.2, 0.02}, {0.2, 0.02}, {0.2, 0.02}}}), m(1.535)
    {
        // m = this->get_parameter("mass").get_value<double>();
        state_sub = this->create_subscription<utils::msg::UAVMotion>(
            "/SCIT_drone/UAV_motion", 10, std::bind(&DOB_node::state_sub_cb, this, _1));
        thrust_sub = this->create_subscription<utils::msg::UAVThrust>(
            "SCIT_drone/UAV_thrust", 10, std::bind(&DOB_node::thrust_sub_cb, this, _1));
        disturbance_pub = this->create_publisher<utils::msg::UAVDisturbance>("SCIT_drone/UAV_disturbance", 10);
        timer = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&DOB_node::timer_callback, this));
    }

private:
    rclcpp::Subscription<utils::msg::UAVMotion>::SharedPtr state_sub;
    rclcpp::Subscription<utils::msg::UAVThrust>::SharedPtr thrust_sub;
    rclcpp::Publisher<utils::msg::UAVDisturbance>::SharedPtr disturbance_pub;
    rclcpp::TimerBase::SharedPtr timer;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Matrix3d R;
    Eigen::Vector3d thrust;
    Eigen::Vector3d df;
    std::array<Lpf_1st, 3> df_filter;
    double m;

    void state_sub_cb(const utils::msg::UAVMotion::SharedPtr msg)
    {
        pos(0) = msg->linear.pos.x;
        pos(1) = msg->linear.pos.y;
        pos(2) = msg->linear.pos.z;

        vel(0) = msg->linear.vel.x;
        vel(1) = msg->linear.vel.y;
        vel(2) = msg->linear.vel.z;

        acc(0) = msg->linear.acc.x;
        acc(1) = msg->linear.acc.y;
        acc(2) = msg->linear.acc.z;

        Eigen::Quaterniond q{msg->angular.q.w, msg->angular.q.x, msg->angular.q.y, msg->angular.q.z};
        R = q.toRotationMatrix();
        // timestamp = msg->timestamp;
    }
    void thrust_sub_cb(const utils::msg::UAVThrust::SharedPtr msg)
    {
        thrust(0) = msg->thrust.x;
        thrust(1) = msg->thrust.y;
        thrust(2) = msg->thrust.z;
    }
    void calculate_df()
    {
        Eigen::Vector3d e3{0, 0, 1};
        df = m * acc - thrust + 9.81f * m * e3;
        for (int i = 0; i < 2; i++)
        {
            df(i) = df_filter.at(i).update(df(i));
        }
    }
    void timer_callback()
    {
        calculate_df();
        utils::msg::UAVDisturbance msg;
        msg.frame_type = utils::msg::UAVDisturbance::ENU;
        msg.df.x = df(0);
        msg.df.y = df(1);
        msg.df.z = df(2);
        disturbance_pub->publish(msg);
        RCLCPP_INFO(this->get_logger(), "[df]: %f,%f,%f", df(0), df(1), df(2));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DOB_node>());
    rclcpp::shutdown();
    return 0;
}
