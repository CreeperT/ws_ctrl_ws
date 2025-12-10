#include "test_task_manager.h"
using std::placeholders::_1;
using std::placeholders::_2;

Task_Manager::Task_Manager(const char* node_name)
    : rclcpp::Node(node_name)
{
    NodeServiceServerInit();
    NodeSpinnerStartup();
}

void Task_Manager::NodeServiceServerInit()
{
    TaskExecuteStatusQuery_server = this->create_service<robot_msgs::srv::TaskExecuteStatusQuery>(
        "TaskExecuteStatusQuery_service", 
        std::bind(&Task_Manager::TaskExecuteStatusQueryHandle, this, _1, _2));

    ModeChangePauseTask_server = this->create_service<robot_msgs::srv::ModeChangePauseTask>(
        "ModeChangePauseTask_service",
        std::bind(&Task_Manager::ModeChangePauseTaskHandle, this, _1, _2));

    ModeChangeStartTask_server = this->create_service<robot_msgs::srv::ModeChangeStartTask>(
        "ModeChangeStartTask_service",
        std::bind(&Task_Manager::ModeChangeStartTaskHandle, this, _1, _2));
}

void Task_Manager::NodeSpinnerStartup()
{
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());
    executor.spin();
}

bool Task_Manager::TaskExecuteStatusQueryHandle(
    const std::shared_ptr<robot_msgs::srv::TaskExecuteStatusQuery::Request> req, 
    std::shared_ptr<robot_msgs::srv::TaskExecuteStatusQuery::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "get in TaskExecuteStatusQueryHandle");
    (void)req;
    res->task_execute = true;
    res->task_ticket = "";
    return true;
}

bool Task_Manager::ModeChangePauseTaskHandle(
    const std::shared_ptr<robot_msgs::srv::ModeChangePauseTask::Request> req, 
    std::shared_ptr<robot_msgs::srv::ModeChangePauseTask::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "get in ModeChangePauseTaskHandle");
    (void)req;
    res->execute_success = true;
    return true;
}

bool Task_Manager::ModeChangeStartTaskHandle(
    const std::shared_ptr<robot_msgs::srv::ModeChangeStartTask::Request> req, 
    std::shared_ptr<robot_msgs::srv::ModeChangeStartTask::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "get in ModeChangeStartTaskHandle");
    (void)req;
    res->execute_success = true;
    return true;
}
