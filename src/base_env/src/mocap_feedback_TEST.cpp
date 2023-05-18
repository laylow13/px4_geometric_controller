#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <Eigen/Eigen>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_visual_odometry.hpp"

class ExternalPoseTEST : public rclcpp::Node
{
public:
    ExternalPoseTEST() : Node("external_pose_to_fcu_test")
    {
        using std::placeholders::_1;
        fcu_odom_pub = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("/fmu/vehicle_visual_odometry/in", 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(int(1000 / 40)),
                                        std::bind(&ExternalPoseTEST::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr fcu_odom_pub;

    void timer_callback();
};
void ExternalPoseTEST::timer_callback()
{
    px4_msgs::msg::VehicleVisualOdometry ext_odom;
    ext_odom.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
    ext_odom.local_frame = 0;
    ext_odom.x = 1.0;
    ext_odom.y = 0.0;
    ext_odom.z = 0.0;
    ext_odom.q[0] = 1;
    ext_odom.q[1] = 0;
    ext_odom.q[2] = 0;
    ext_odom.q[3] = 0;
    fcu_odom_pub->publish(ext_odom);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExternalPoseTEST>());
    rclcpp::shutdown();
    return 0;
}
