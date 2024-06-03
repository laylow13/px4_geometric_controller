#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "fdcl/control.hpp"
#include "utils/msg/uav_state.hpp"
#include "utils/msg/uav_cmd.hpp"


class fdcl_controller_node : public rclcpp::Node
{
public:

    fdcl_controller_node() : Node("fdcl_se3_controller")
    {
        state = new fdcl::state_t();
        command = new fdcl::command_t();
        config_file = new fdcl::param();
        config_file->open("../cfg/uav.cfg");
        controller.reinit(state,command,config_file);

        state_sub = this->create_subscription<utils::msg::UAVState>(
            "uav_state", 10, std::bind(&fdcl_controller_node::state_sub_cb, this, _1));
        cmd_sub = this->create_subscription<utils::msg::UAVCmd>(
            "uav_command", 10, std::bind(&fdcl_controller_node::cmd_sub_cb, this, _1));
        fm_pub = this->create_publisher</* msg_type */>(" ", 10);

        this->create_wall_timer( std::chrono::seconds(1),
            std::bind(&fdcl_controller_node::timer_callback, this));

    }

    ~fdcl_controller_node()
    {
        delete state;
        delete command;
        delete config_file;
    }
private:

    rclcpp::Publisher</* msg_type */>::SharedPtr fm_pub;

    fdcl::state_t *state;
    fdcl::command_t *command;
    fdcl::param *config_file;
    fdcl::control controller;
    double f_out;
    Vector3 M_out;

    void cmd_sub_cb(const /* msg_type */::SharedPtr msg)
    {

        RCLCPP_INFO(this->get_logger(), "Received message");
    }

    void state_sub_cb(const /* msg_type */::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message");
    }

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Timer event");
        controller.position_control();
        controller.output_fM(f_out, M_out);
        //pub

    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fdcl_controller_node>());
    rclcpp::shutdown();
    return 0;
}