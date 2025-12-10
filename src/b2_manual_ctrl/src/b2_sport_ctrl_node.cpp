#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree_api/msg/request.hpp>  // 假设这是Unitree的Request消息
#include <thread>  // 用于std::this_thread::sleep_for

class SportCtrlNode : public rclcpp::Node {
public:
    SportCtrlNode() : Node("sport_ctrl_node") {
        // 声明参数
        this->declare_parameter<double>("rate", 200.0);
        this->declare_parameter<std::string>("log_level", "info");

        // 设置日志级别
        std::string log_level = this->get_parameter("log_level").as_string();
        if (log_level == "debug") {
            this->get_logger().set_level(rclcpp::Logger::Level::Debug);
        } else if (log_level == "info") {
            this->get_logger().set_level(rclcpp::Logger::Level::Info);
        } else if (log_level == "warn") {
            this->get_logger().set_level(rclcpp::Logger::Level::Warn);
        } else if (log_level == "error") {
            this->get_logger().set_level(rclcpp::Logger::Level::Error);
        } else if (log_level == "fatal") {
            this->get_logger().set_level(rclcpp::Logger::Level::Fatal);
        } else {
            this->get_logger().set_level(rclcpp::Logger::Level::Info);
        }

        // 创建发布者
        publisher_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // 初始化序列
        RCLCPP_INFO(this->get_logger(), "Initializing B2 to visual assist mode...");
        
        // Step 1: BalanceStand to unlock joints (api_id=1002, no params)
        send_request(1002);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Step 2: VisionWalk(true) for visual assist mode (api_id=1101, {"data": true})
        send_request(1101, "{\"data\": true}");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        RCLCPP_INFO(this->get_logger(), "B2 initialized to visual assist mode. Ready for cmd_vel.");

        // 创建订阅者
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&SportCtrlNode::cmd_vel_callback, this, std::placeholders::_1)
        );
    }

private:
    void send_request(int api_id, std::string parameters = "") {
        unitree_api::msg::Request req;
        req.header.identity.api_id = api_id;
        if (!parameters.empty()) {
            req.parameter = parameters;
        }
        publisher_->publish(req);
        RCLCPP_INFO(this->get_logger(), "Published request with api_id=%d, parameters=%s",
                    api_id, parameters.empty() ? "none" : parameters.c_str());
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
                    msg->linear.x, msg->linear.y, msg->angular.z);
        
        // 转换为JSON参数：{"x": vx, "y": vy, "z": vyaw}
        std::string parameters = "{\"x\": " + std::to_string(msg->linear.x) +
                                 ", \"y\": " + std::to_string(msg->linear.y) +
                                 ", \"z\": " + std::to_string(msg->angular.z) + "}";
        
        // 发布Move请求 (api_id=1008)
        send_request(1008, parameters);
    }

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SportCtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
