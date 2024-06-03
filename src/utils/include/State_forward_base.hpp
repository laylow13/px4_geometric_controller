#ifndef _STATEFORWARD_H
#define _STATEFORWARD_H

#include "rclcpp/rclcpp.hpp"
#include "string"
#include "defines.hpp"
#include "utils/msg/uav_state_feedback.hpp"
#include "Eigen/Eigen"

using namespace Eigen;

class State_forward_base : public rclcpp::Node {
public:
    State_forward_base(std::string node_name) : Node(node_name) {
        state_pub = this->create_publisher<utils::msg::UAVStateFeedback>("/SCIT_drone/UAV_state_feedback", 10);
        this->declare_parameter<int>("frequency", 50);
        this->declare_parameter<uint8_t>("world_frame", FRAME_WORLD_NED);
        this->declare_parameter<uint8_t>("body_frame", FRAME_BODY_FRD);
        frequency = this->get_parameter("frequency").get_value<int>();
        target_world_frame = this->get_parameter("world_frame").get_value<uint8_t>();
        target_body_frame = this->get_parameter("body_frame").get_value<uint8_t>();
        timer = this->create_wall_timer(std::chrono::milliseconds(int(1000 / frequency)),
                                        std::bind(&State_forward_base::timer_cb, this));
    }

private:
    int frequency;
    uint8_t target_world_frame;
    uint8_t target_body_frame;
    state_t current_state;
    rclcpp::Publisher<utils::msg::UAVStateFeedback>::SharedPtr state_pub;
    rclcpp::TimerBase::SharedPtr timer;

private:
    virtual state_t &get_current_state() = 0;

    void timer_cb() {
        current_state = get_current_state();
//        state_t target_state;
//        transform(current_state, target_state, target_world_frame, target_body_frame);

        utils::msg::UAVStateFeedback msg;
        msg.header.stamp.sec = current_state.timestamp.sec;
        msg.header.stamp.nanosec = current_state.timestamp.nanosec;
        msg.world_frame = current_state.world_frame;
        msg.body_frame = current_state.body_frame;
        msg.x.x = current_state.x(0);
        msg.x.y = current_state.x(1);
        msg.x.z = current_state.x(2);
        msg.x_dot.x = current_state.x_dot(0);
        msg.x_dot.y = current_state.x_dot(1);
        msg.x_dot.z = current_state.x_dot(2);
        msg.x_2dot.x = current_state.x_2dot(0);
        msg.x_2dot.y = current_state.x_2dot(1);
        msg.x_2dot.z = current_state.x_2dot(2);
        msg.x_3dot.x = current_state.x_3dot(0);
        msg.x_3dot.y = current_state.x_3dot(1);
        msg.x_3dot.z = current_state.x_3dot(2);
        msg.x_4dot.x = current_state.x_4dot(0);
        msg.x_4dot.y = current_state.x_4dot(1);
        msg.x_4dot.z = current_state.x_4dot(2);

        msg.q.w = current_state.q.w();
        msg.q.x = current_state.q.x();
        msg.q.y = current_state.q.y();
        msg.q.z = current_state.q.z();

        msg.w.x = current_state.w(0);
        msg.w.y = current_state.w(1);
        msg.w.z = target_state.w(2);
        msg.w_dot.x = target_state.w_dot(0);
        msg.w_dot.y = target_state.w_dot(1);
        msg.w_dot.z = target_state.w_dot(2);
        state_pub->publish(msg);
    }

    void transform(state_t &current_state_, state_t &target_state_, uint8_t target_world_frame_,
                   uint8_t target_body_frame_
    ) {
        auto R = DiagonalMatrix<double, 3>(1, -1, -1);
        target_state_ = current_state;
        if (current_state_.world_frame != target_world_frame_) {
            target_state_.x = R * current_state.x;
            target_state_.x_dot = R * current_state.x_dot;
            target_state_.x_2dot = R * current_state.x_2dot;
            target_state_.x_3dot = R * current_state.x_3dot;
            target_state_.x_4dot = R * current_state.x_4dot;
            target_state_.q = Quaterniond(R * current_state_.q.toRotationMatrix());
        }
        if (current_state_.body_frame != target_body_frame_) {
            target_state_.q = Quaterniond(current_state_.q.toRotationMatrix() * R);
            target_state_.w = R * current_state_.w;
            target_state_.w_dot = R * current_state_.w_dot;
        }
    }
};

#endif