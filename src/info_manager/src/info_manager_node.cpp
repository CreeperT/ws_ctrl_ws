#include "info_manager.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <locale.h>  // 用于 setlocale

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    setlocale(LC_CTYPE, "zh_CN.utf8");

    // 使用 shared_ptr 创建节点实例，以符合 ROS2 规范
    auto info_manager_client = std::make_shared<Info_Manager>("info_manager_node");

    // 调用 NodeSpinnerStartup()，保持原逻辑
    info_manager_client->NodeSpinnerStartup();

    // 等待 spinning 线程完成（相当于 ROS1 的 ros::waitForShutdown()）
    info_manager_client->joinSpinnerThread();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}