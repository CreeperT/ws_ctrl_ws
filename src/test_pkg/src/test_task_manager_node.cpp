#include "test_task_manager.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    setlocale(LC_CTYPE, "zh_CN.utf8");

    auto test_task_manager = std::make_shared<Task_Manager>("test_task_manager_node");
    test_task_manager->NodeSpinnerStartup();

    test_task_manager->joinSpinnerThread();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}