/*
get state information from px4 topic, and publish to controller after frame transformation
*/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "State_forward_base.hpp"
//#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_angular_velocity.hpp"
#include "Dirty_derivative.hpp"

using std::placeholders::_1;

class Sim_state_forward : public State_forward_base {
public:
    Sim_state_forward() : State_forward_base("simulation_state_forward") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
//        px4_odometry_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
//                "/fmu/out/vehicle_odometry", qos, std::bind(&Sim_state_forward::px4_odometry_cb, this, _1));
        px4_pos_sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
                                                                               std::bind(&Sim_state_forward::px4_pos_cb,
                                                                                         this, _1));
        px4_att_sub = create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
                                                                          std::bind(&Sim_state_forward::px4_att_cb,
                                                                                    this, _1));
        px4_angvel_sub = create_subscription<px4_msgs::msg::VehicleAngularVelocity>("/fmu/out/vehicle_angular_velocity",
                                                                                    qos, std::bind(
                        &Sim_state_forward::px4_angvel_cb,
                        this, _1));

        px4_state.world_frame = FRAME_WORLD_NED;
        px4_state.body_frame = FRAME_BODY_FRD;
    }

private:
//    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odometry_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_pos_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr px4_att_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr px4_angvel_sub;

    state_t px4_state;

    state_t &get_current_state() override {
        return px4_state;
    }

//    void px4_odometry_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
//        px4_state.timestamp.sec = msg->timestamp_sample / int(1e6);
//        px4_state.timestamp.nanosec = msg->timestamp_sample * int(1e3) - px4_state.timestamp.sec * int(1e9);
//        px4_state.world_frame = FRAME_WORLD_NED;
//        px4_state.body_frame = FRAME_BODY_FRD;
//        px4_state.x << msg->position[0], msg->position[1], msg->position[2];
//        px4_state.x_dot << msg->velocity[0], msg->velocity[1], msg->velocity[2];
//
//        px4_state.q = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
//        px4_state.w << msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2];
//    }

    void px4_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        px4_state.x << msg->x, msg->y, msg->z;
        px4_state.x_dot << msg->vx, msg->vy, msg->vz;
        px4_state.x_2dot << msg->ax, msg->ay, msg->az;
    }

    void px4_att_cb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        px4_state.timestamp.sec = msg->timestamp_sample / int(1e6);
        px4_state.timestamp.nanosec = msg->timestamp_sample * int(1e3) - px4_state.timestamp.sec * int(1e9);
        px4_state.q = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    }

    void px4_angvel_cb(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
        px4_state.w << msg->xyz[0], msg->xyz[1], msg->xyz[2];
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim_state_forward>());
    rclcpp::shutdown();
    return 0;
}