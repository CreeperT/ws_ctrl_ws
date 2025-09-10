#include "ws_msg_manage.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <locale.h>  // 用于 setlocale

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    setlocale(LC_CTYPE, "zh_CN.utf8");

    // 使用 shared_ptr 创建节点实例，以符合 ROS2 规范
    auto ws_msgs_manager_client = std::make_shared<WSmsgs_Manager_Client>("ws_msgs_manage_client_node");

    // 调用 NodeSpinnerStartup()，保持原逻辑
    ws_msgs_manager_client->NodeSpinnerStartup();

    // 等待 spinning 线程完成（相当于 ROS1 的 ros::waitForShutdown()）
    ws_msgs_manager_client->joinSpinnerThread();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
