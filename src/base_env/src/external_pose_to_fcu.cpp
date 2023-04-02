#include <chrono>
#include <memory>
#include <string>
#include <Eigen/Eigen>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

class ExternalPose : public rclcpp::Node
{
public:
    ExternalPose() : Node("external_pose_to_fcu")
    {
        using std::placeholders::_1;
        topic_sub = this->get_parameter("topic_name").get_value<std::string>();
        header_frame_id = this->get_parameter("header_frame_id").get_value<std::string>();
        rate_hz = this->get_parameter("rate_hz").get_value<float>();
        pos_offset[0] = this->get_parameter("offset_x").get_value<float>();
        pos_offset[1] = this->get_parameter("offset_y").get_value<float>();
        pos_offset[2] = this->get_parameter("offset_z").get_value<float>();
        yaw_offset = this->get_parameter("offset_yaw").get_value<float>();

        this->create_subscription<nav_msgs::msg::Odometry>(
            topic_sub, 10, std::bind(&ExternalPose::external_pose_sub_cb, this, _1));
        fcu_pose_pub = this->create_publisher<px4_msgs::msg::VehicleOdometry>("vehicle_mocap_odometry", 10);
        this->create_wall_timer(std::chrono::milliseconds(int(1000/rate_hz)),
                                std::bind(&ExternalPose::timer_callback, this));
    }

private:
    std::string topic_sub;
    std::string header_frame_id;
    Eigen::Vector3d pos_drone; // 当前位置
    Eigen::Quaterniond q_drone;
    float rate_hz;
    float yaw_offset;
    Eigen::Vector3f pos_offset;
    std::string topic_name;
    px4_msgs::msg::VehicleOdometry ext_pose;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr fcu_pose_pub;
    
    void timer_callback()
    {
        fcu_pose_pub->publish(ext_pose);
    }
    
    void external_pose_sub_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (msg->header.frame_id == header_frame_id)
        {
            pos_drone = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            q_drone = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

            ext_pose.position[0] = pos_drone[0];
            ext_pose.position[1] = pos_drone[1];
            ext_pose.position[2] = pos_drone[2];

            ext_pose.q[0] = q_drone.x();
            ext_pose.q[1] = q_drone.y();
            ext_pose.q[2] = q_drone.z();
            ext_pose.q[3] = q_drone.w();
            //TO DO: timestamp test!!
            ext_pose.timestamp=rclcpp::Clock().now().nanoseconds()/1000;
            /*
            uint8 POSE_FRAME_UNKNOWN = 0
            uint8 POSE_FRAME_NED     = 1
            uint8 POSE_FRAME_FRD     = 2*/
            ext_pose.pose_frame=1;  
            // ext_pose.quality=
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Wrong hearder.frame_id, please check it!");
        }
    }
};

int main(int argc, char *argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExternalPose>());
  rclcpp::shutdown();
  return 0;
}

