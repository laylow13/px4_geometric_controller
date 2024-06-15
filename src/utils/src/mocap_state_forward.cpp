#include <map>
#include "type_definitions.hpp"
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
    state_t mocap_state;

    state_t &get_current_state() override {
        return mocap_state;
    }

    void mocap_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        mocap_state.timestamp.sec = msg->header.stamp.sec;
        mocap_state.timestamp.nanosec = msg->header.stamp.nanosec;
        mocap_state.world_frame = FRAME_WORLD_ENU;
        mocap_state.body_frame = FRAME_BODY_FLU;
        mocap_state.x << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        mocap_state.q = Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                    msg->pose.orientation.z);
    }

    void mocap_vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        mocap_state.x_dot << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
        mocap_state.w << msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mocap_state_forward>());
    rclcpp::shutdown();
    return 0;
}
