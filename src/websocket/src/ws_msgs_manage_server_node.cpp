#include "ws_msg_manage.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <locale.h>  // 用于 setlocale
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    setlocale(LC_CTYPE, "zh_CN.utf8");

    auto ws_msgs_manager_server = std::make_shared<WSmsgs_Manager_Server>("ws_msgs_manager_server_node");
    ws_msgs_manager_server->NodeSpinnerStartup();

    ws_msgs_manager_server->joinSpinnerThread();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}