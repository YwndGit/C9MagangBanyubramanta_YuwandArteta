#include <rclcpp/rclcpp.hpp>
#include "controller/msg/control_command.hpp"
#include <asio.hpp>
#include <iostream>
#include <sstream>

class SercomToCmdvel : public rclcpp::Node {
public:
    SercomToCmdvel() : Node("sercom_to_cmdvel"), serial_(io_) {
        subscription_ = this->create_subscription<controller::msg::ControlCommand>(
            "cmd_vel", 10, std::bind(&SercomToCmdvel::callback, this, std::placeholders::_1));
        try {
            serial_.open("/dev/ttyACM0");
            serial_.set_option(asio::serial_port_base::baud_rate(115200));
            RCLCPP_INFO(this->get_logger(), "Serial port opened!");
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: %s", e.what());
        }
    }
private:
    void callback(const controller::msg::ControlCommand::SharedPtr msg) {
        if (!serial_.is_open()) return;
        
        std::ostringstream oss;
        oss << msg->x_cmd << "," 
            << msg->y_cmd << "," 
            << msg->yaw << "," 
            << msg->depth << "\n";
        
        asio::write(serial_, asio::buffer(oss.str()));
        RCLCPP_INFO(this->get_logger(), "Sent: %s", oss.str().c_str());
    }
    rclcpp::Subscription<controller::msg::ControlCommand>::SharedPtr subscription_;
    asio::io_context io_;
    asio::serial_port serial_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SercomToCmdvel>());
    rclcpp::shutdown();
    return 0;
}
