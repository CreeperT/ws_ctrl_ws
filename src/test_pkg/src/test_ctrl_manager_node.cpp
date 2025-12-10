#include "test_ctrl_manager.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    setlocale(LC_CTYPE, "zh_CN.utf8");

    auto test_ctrl_manager = std::make_shared<Ctrl_Manager>("test_ctrl_manager_node");
    test_ctrl_manager->NodeSpinnerStartup();

    test_ctrl_manager->joinSpinnerThread();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}