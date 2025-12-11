#include "ctrl_manager.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <locale.h>  // 用于 setlocale
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    setlocale(LC_CTYPE, "zh_CN.utf8");

    auto ws_msgs_manager_server = std::make_shared<Ctrl_Manager>("ctrl_manager_node");
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 15);
    executor.add_node(ws_msgs_manager_server);
    executor.spin();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}