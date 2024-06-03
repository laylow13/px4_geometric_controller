#include <map>
#include "defines.hpp"
#include "State_forward_base.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Mocap_state_forward : public State_forward_base {
public:
    Mocap_state_forward() : State_forward_base("Mocap_state_forward") {
        std::map<std::string, std::string> _parameters{{"pos_topic", ""},
                                                       {"vel_topic", ""}};
        this->declare_parameters<std::string>("", _parameters);
        auto pos_topic = this->get_parameter("pos_topic").get_value<std::string>();
        auto vel_topic = this->get_parameter("vel_topic").get_value<std::string>();
        auto pos_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                pos_topic, 10, std::bind(&Mocap_state_forward::mocap_pos_cb, this, _1));
        auto vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
                vel_topic, 10, std::bind(&Mocap_state_forward::mocap_vel_cb, this, _1));
    }

private:
    double last_timestamp;
    Vector3d last_vel;

    void mocap_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_state.timestamp.sec = msg->header.stamp.sec;
        current_state.timestamp.nanosec = msg->header.stamp.nanosec;
        current_state.world_frame = FRAME_WORLD_NED;
        current_state.body_frame = FRAME_BODY_FRD;
        current_state.x << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        current_state.q = Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                      msg->pose.orientation.z);
    }

    void mocap_vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        double timestamp = (msg->header.stamp.sec + msg->header.stamp.nanosec) * 1e-9;
        double dt = timestamp - last_timestamp;
        last_timestamp = timestamp;
        current_state.x_dot << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
        current_state.x_2dot = (current_state.x_dot - last_vel) / dt;
        last_vel = current_state.x_dot;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mocap_state_forward>());
    rclcpp::shutdown();
    return 0;
}
