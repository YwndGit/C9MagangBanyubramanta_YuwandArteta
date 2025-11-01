#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "controller/msg/control_command.hpp"
#include <cmath>

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        // Initialize values
        x_cmd_ = 0.0;
        y_cmd_ = 0.0;
        accumulated_depth_ = 0.0;
        accumulated_yaw_ = 0.0;
        
        // Subscriber ke /joy
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&ControllerNode::joyCallback, this, std::placeholders::_1));
        
        // Publisher ke /cmd_vel
        cmd_pub_ = this->create_publisher<controller::msg::ControlCommand>("/cmd_vel", 10);
        
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 100ms = 10Hz (lebih lambat)
    std::bind(&ControllerNode::publishCommand, this));
        
        RCLCPP_INFO(this->get_logger(), "Controller node started!");
        RCLCPP_INFO(this->get_logger(), "Publishing to /cmd_vel at 50Hz");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() >= 6) {
            // ========== X & Y: Direct Mapping (Position-based) ==========
            // y_cmd: Depan/Mundur (Stick kiri Up/Down) - axes[1]
            y_cmd_ = msg->axes[1] * 250.0;
            
            // x_cmd: Kanan/Kiri (Stick kiri Left/Right) - axes[0]
            x_cmd_ = msg->axes[0] * -250.0;
            
            
            // ========== YAW: Incremental (Stick kanan Left/Right) ==========
            float yaw_input = msg->axes[3];
            
            if (std::abs(yaw_input) > 0.3) {  // Dead zone
                accumulated_yaw_ += yaw_input * -2.0;
                accumulated_yaw_ = clamp(accumulated_yaw_, -180.0, 180.0);
            }
            
            
            // ========== DEPTH: Incremental (Stick kanan Up/Down) ==========
            float depth_input = msg->axes[4];
            
            if (std::abs(depth_input) > 0.3) {  // Dead zone
                accumulated_depth_ += depth_input * -0.1;
                accumulated_depth_ = clamp(accumulated_depth_, 0.0, 10.0);
            }
            
        }
    }
    
    void publishCommand()
    {
        auto cmd = controller::msg::ControlCommand();
        cmd.x_cmd = x_cmd_;
        cmd.y_cmd = y_cmd_;
        cmd.yaw = accumulated_yaw_;
        cmd.depth = accumulated_depth_;
        
        cmd_pub_->publish(cmd);
    
    }
    
    float clamp(float value, float min, float max) {
        return std::max(min, std::min(value, max));
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<controller::msg::ControlCommand>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    float x_cmd_;
    float y_cmd_;
    float accumulated_depth_;
    float accumulated_yaw_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


