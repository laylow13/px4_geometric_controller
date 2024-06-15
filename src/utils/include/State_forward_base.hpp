#ifndef _STATEFORWARD_H
#define _STATEFORWARD_H

#include "rclcpp/rclcpp.hpp"
#include "string"
#include "type_definitions.hpp"
#include "utils/msg/uav_state_feedback.hpp"
#include "Eigen/Eigen"
#include "Dirty_derivative.hpp"
#include "memory"

using namespace Eigen;

class State_forward_base : public rclcpp::Node {
public:
    State_forward_base(std::string node_name) : Node(node_name), tau(0.5) {
        state_pub = this->create_publisher<utils::msg::UAVStateFeedback>("/SCIT_drone/UAV_state_feedback", 10);
        this->declare_parameter<int>("frequency", 50);
        this->declare_parameter<uint8_t>("world_frame", FRAME_WORLD_NED);
        this->declare_parameter<uint8_t>("body_frame", FRAME_BODY_FRD);
        frequency = this->get_parameter("frequency").get_value<int>();
        target_world_frame = this->get_parameter("world_frame").get_value<uint8_t>();
        target_body_frame = this->get_parameter("body_frame").get_value<uint8_t>();
        x_2dot_differentiator = std::make_shared<Dirty_derivative<Vector3d>>(1, tau, 1. / frequency);
        x_3dot_differentiator = std::make_shared<Dirty_derivative<Vector3d>>(2, tau, 1. / frequency);
        x_4dot_differentiator = std::make_shared<Dirty_derivative<Vector3d>>(3, tau, 1. / frequency);
        w_dot_differentiator = std::make_shared<Dirty_derivative<Vector3d>>(1, tau, 1. / frequency);
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
    double tau;
    std::shared_ptr<Dirty_derivative<Vector3d>> x_2dot_differentiator;
    std::shared_ptr<Dirty_derivative<Vector3d>> x_3dot_differentiator;
    std::shared_ptr<Dirty_derivative<Vector3d>> x_4dot_differentiator;
    std::shared_ptr<Dirty_derivative<Vector3d>> w_dot_differentiator;

private:
    virtual state_t &get_current_state() = 0;

    void timer_cb() {
        current_state = get_current_state();
        current_state.x_2dot = x_2dot_differentiator->calculate(current_state.x_dot);
        current_state.x_3dot = x_3dot_differentiator->calculate(current_state.x_2dot);
        current_state.x_4dot = x_4dot_differentiator->calculate(current_state.x_3dot);
        current_state.w_dot = w_dot_differentiator->calculate(current_state.w);

        state_t target_state;
        transform(current_state, target_state, target_world_frame, target_body_frame);

        utils::msg::UAVStateFeedback msg;
        msg.header.stamp.sec = current_state.timestamp.sec;
        msg.header.stamp.nanosec = current_state.timestamp.nanosec;
        msg.world_frame = target_world_frame;
        msg.body_frame = target_body_frame;
        msg.pos.x = target_state.x(0);
        msg.pos.y = target_state.x(1);
        msg.pos.z = target_state.x(2);
        msg.vel.x = target_state.x_dot(0);
        msg.vel.y = target_state.x_dot(1);
        msg.vel.z = target_state.x_dot(2);
        msg.acc.x = target_state.x_2dot(0);
        msg.acc.y = target_state.x_2dot(1);
        msg.acc.z = target_state.x_2dot(2);
        msg.jerk.x = target_state.x_3dot(0);
        msg.jerk.y = target_state.x_3dot(1);
        msg.jerk.z = target_state.x_3dot(2);
        msg.snap.x = target_state.x_4dot(0);
        msg.snap.y = target_state.x_4dot(1);
        msg.snap.z = target_state.x_4dot(2);

        msg.q.w = target_state.q.w();
        msg.q.x = target_state.q.x();
        msg.q.y = target_state.q.y();
        msg.q.z = target_state.q.z();

        msg.ang_vel.x = target_state.w(0);
        msg.ang_vel.y = target_state.w(1);
        msg.ang_vel.z = target_state.w(2);
        msg.ang_acc.x = target_state.w_dot(0);
        msg.ang_acc.y = target_state.w_dot(1);
        msg.ang_acc.z = target_state.w_dot(2);

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