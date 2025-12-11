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
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 15);
    executor.add_node(info_manager_client);
    executor.spin();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}