#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "base_env/msg/uav_motion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

class Data_convert : public rclcpp::Node
{
public:
  Data_convert() : Node("mocap_data_convert"), last_timestamp(0.0)
  {
    using std::placeholders::_1;
    std::map<std::string, std::string> _parameters{{"pos_topic", ""}, {"vel_topic", ""}};
    this->declare_parameters<std::string>("", _parameters);
    auto pos_topic = this->get_parameter("pos_topic").get_value<std::string>();
    auto vel_topic = this->get_parameter("vel_topic").get_value<std::string>();
    motion_pub = this->create_publisher<base_env::msg::UAVMotion>("/SCIT_drone/UAV_motion", 10);
    pos_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pos_topic, 10, std::bind(&Data_convert::pos_sub_cb, this, _1));
    vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        vel_topic, 10, std::bind(&Data_convert::vel_sub_cb, this, _1));
    timer_ = this->create_wall_timer(
        20ms, std::bind(&Data_convert::timer_callback, this));
  }

private:
  void timer_callback()
  {
    motion_pub->publish(mav_motion);
    RCLCPP_DEBUG(this->get_logger(), "Publishing");
  }
  void pos_sub_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void vel_sub_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  double last_timestamp;
  Eigen::Vector3d last_vel;
  base_env::msg::UAVMotion mav_motion;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<base_env::msg::UAVMotion>::SharedPtr motion_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub;
};
void Data_convert::pos_sub_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  mav_motion.linear.pos.x = msg->pose.position.x;
  mav_motion.linear.pos.y = msg->pose.position.y;
  mav_motion.linear.pos.z = msg->pose.position.z;
  mav_motion.angular.q = msg->pose.orientation;
}

void Data_convert::vel_sub_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  double timestamp = (msg->header.stamp.sec + msg->header.stamp.nanosec) * 1e-9;
  double dt = timestamp - last_timestamp;
  last_timestamp = timestamp;
  mav_motion.linear.vel = msg->twist.linear;
  mav_motion.linear.acc.x = (mav_motion.linear.vel.x - last_vel(0)) / dt;
  mav_motion.linear.acc.y = (mav_motion.linear.vel.y - last_vel(1)) / dt;
  mav_motion.linear.acc.z = (mav_motion.linear.vel.z - last_vel(2)) / dt;
  last_vel << mav_motion.linear.vel.x, mav_motion.linear.vel.y, mav_motion.linear.vel.z;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Data_convert>());
  rclcpp::shutdown();
  return 0;
}
