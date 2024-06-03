#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <Eigen/Eigen>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

struct mocap_data_t {
    Eigen::Vector3d x;    // position
    Eigen::Quaterniond q; // orientation
    Eigen::Vector3d v;    // linear_velocity
    Eigen::Vector3d w;    // angular_velocity

    mocap_data_t ENU2NED() {
        mocap_data_t temp;
        Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix(); // from NED to ENU
        temp.x = R * x;
        temp.v = R * v;
        temp.w = R * w;
        temp.q = Eigen::Quaterniond(R).normalized() * q * Eigen::Quaterniond(R.transpose()).normalized();
        return temp;
    };
};

class ExternalPose : public rclcpp::Node {
public:
    ExternalPose() : Node("external_pose_to_fcu"), data_valid(false) {
        using std::placeholders::_1;

        std::map<std::string, std::string> parameters_string{{"pos_topic",       ""},
                                                             {"vel_topic",       ""},
                                                             {"header_frame_id", ""}};
        std::map<std::string, float> parameters_float{{"rate_hz",    30},
                                                      {"offset_x",   0},
                                                      {"offset_y",   0},
                                                      {"offset_z",   0},
                                                      {"offset_yaw", 0}};
        this->declare_parameters<std::string>("", parameters_string);
        this->declare_parameters<float>("", parameters_float);

        std::string pos_topic = this->get_parameter("pos_topic").get_value<std::string>();
        std::string vel_topic = this->get_parameter("vel_topic").get_value<std::string>();
        header_frame_id = this->get_parameter("header_frame_id").get_value<std::string>();
        float rate_hz = this->get_parameter("rate_hz").get_value<float>();
        pos_offset[0] = this->get_parameter("offset_x").get_value<float>();
        pos_offset[1] = this->get_parameter("offset_y").get_value<float>();
        pos_offset[2] = this->get_parameter("offset_z").get_value<float>();
        yaw_offset = this->get_parameter("offset_yaw").get_value<float>();

        ext_pos_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                pos_topic, 10, std::bind(&ExternalPose::ext_pos_sub_cb, this, _1));
        ext_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
                vel_topic, 10, std::bind(&ExternalPose::ext_vel_sub_cb, this, _1));
        fcu_odom_pub = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry",
                                                                              10);
        timer = this->create_wall_timer(std::chrono::milliseconds(int(1000 / rate_hz)),
                                        std::bind(&ExternalPose::timer_callback, this));
    }

private:
    bool data_valid;
    std::string header_frame_id;
    float yaw_offset;
    mocap_data_t mocap_data;
    Eigen::Vector3d pos_offset;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ext_pos_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ext_vel_sub;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr fcu_odom_pub;

    void timer_callback();

    void ext_pos_sub_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void ext_vel_sub_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
};

void ExternalPose::ext_pos_sub_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (msg->header.frame_id == header_frame_id) {
        data_valid = true;
        Eigen::Vector3d pos_drone{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
        Eigen::Quaterniond q_drone{msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                   msg->pose.orientation.z};

        mocap_data.x = pos_drone + pos_offset;
        mocap_data.q = q_drone;
    } else {
        RCLCPP_INFO(this->get_logger(), "Wrong frame_id, please check it!");
        data_valid = false;
    }
}

void ExternalPose::ext_vel_sub_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (msg->header.frame_id == header_frame_id) {
        mocap_data.v << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
        mocap_data.w << msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
    } else {
        RCLCPP_INFO(this->get_logger(), "Wrong frame_id, please check it!");
    }
}

void ExternalPose::timer_callback() {
    if (data_valid) {
        mocap_data_t NED_mocap_data = mocap_data.ENU2NED();

        px4_msgs::msg::VehicleOdometry ext_odom;
        ext_odom.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
        ext_odom.pose_frame = 1;
        ext_odom.position[0] = NED_mocap_data.x[0];
        ext_odom.position[1] = NED_mocap_data.x[1];
        ext_odom.position[2] = NED_mocap_data.x[2];
        ext_odom.q[0] = NED_mocap_data.q.w();
        ext_odom.q[1] = NED_mocap_data.q.x();
        ext_odom.q[2] = NED_mocap_data.q.y();
        ext_odom.q[3] = NED_mocap_data.q.z();
        ext_odom.velocity_frame = 1;
        ext_odom.velocity[0] = NED_mocap_data.v[0];
        ext_odom.velocity[1] = NED_mocap_data.v[1];
        ext_odom.velocity[2] = NED_mocap_data.v[2];
        // TODO: mocap angular speed frame unknown
        // ext_odom.rollspeed = NED_mocap_data.w[0];
        // ext_odom.pitchspeed = NED_mocap_data.w[1];
        // ext_odom.yawspeed = NED_mocap_data.w[2];

        fcu_odom_pub->publish(ext_odom);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExternalPose>());
    rclcpp::shutdown();
    return 0;
}
