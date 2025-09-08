/*  功能：将MotorCtrlNormal消息转换为Twist消息
 *  
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_msgs/msg/motor_ctrl_normal.hpp"
using namespace std;
using namespace rclcpp;
using namespace std::placeholders;

class MotorCtrl2Twist : public Node
{
private:
    Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    Subscription<robot_msgs::msg::MotorCtrlNormal>::SharedPtr motor_ctrl_sub_;
    void motor_ctrl_callback(const robot_msgs::msg::MotorCtrlNormal::SharedPtr motor_ctrl_msg)
    {
        geometry_msgs::msg::Twist twist_msg;
        // 获得速度
        double speed = motor_ctrl_msg->run_speed;
        // 获得command消息
        auto command = motor_ctrl_msg->command;
        // 解析command消息
        if (strcmp(command.c_str(), "move_forward") == 0) // 向前，即x正方向
        {
            twist_msg.linear.x = speed;
            twist_msg.linear.y = 0;
        }
        else if (strcmp(command.c_str(), "move_back") == 0) // 向后，即x负方向
        {
            twist_msg.linear.x = -speed;
            twist_msg.linear.y = 0;
        }
        else if (strcmp(command.c_str(), "turn_left") == 0) // 向左，即y正方向
        {
            twist_msg.linear.x = 0;
            twist_msg.linear.y = speed;
        }
        else if (strcmp(command.c_str(), "turn_right") == 0) // 向右，即y负方向
        {
            twist_msg.linear.x = 0;
            twist_msg.linear.y = -speed;
        }
        else if (strcmp(command.c_str(), "move_stop") == 0) // 停止
        {
            twist_msg.linear.x = 0;
            twist_msg.linear.y = 0;
        }
        
        twist_pub_->publish(twist_msg);
    }

public:
    explicit MotorCtrl2Twist(const string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点 %s: 已启动.", name.c_str());
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        motor_ctrl_sub_ = this->create_subscription<robot_msgs::msg::MotorCtrlNormal>(
            "MotorCtrlNormal_topic", 10, bind(&MotorCtrl2Twist::motor_ctrl_callback, this, _1));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorCtrl2Twist>("motor_ctrl_2_twist");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}