#ifndef __TEST_TASK_MANAGER_H__
#define __TEST_TASK_MANAGER_H__

#include "rclcpp/rclcpp.hpp"
#include <cjson/cJSON.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <string>

#include "robot_msgs/srv/task_execute_status_query.hpp"
#include "robot_msgs/srv/mode_change_pause_task.hpp"
#include "robot_msgs/srv/mode_change_start_task.hpp"

class Task_Manager : public rclcpp::Node
{
public:
    Task_Manager(const char* node_name);

    void NodeServiceServerInit();
    void NodeSpinnerStartup();

    bool TaskExecuteStatusQueryHandle(robot_msgs::srv::TaskExecuteStatusQuery::Request::SharedPtr req, robot_msgs::srv::TaskExecuteStatusQuery::Response::SharedPtr res);
    bool ModeChangePauseTaskHandle(robot_msgs::srv::ModeChangePauseTask::Request::SharedPtr req, robot_msgs::srv::ModeChangePauseTask::Response::SharedPtr res);
    bool ModeChangeStartTaskHandle(robot_msgs::srv::ModeChangeStartTask::Request::SharedPtr req, robot_msgs::srv::ModeChangeStartTask::Response::SharedPtr res);

    void joinSpinnerThread()
    {
        if (spinner_thread_.joinable())
            spinner_thread_.join();
    }

public:
    rclcpp::Service<robot_msgs::srv::TaskExecuteStatusQuery>::SharedPtr TaskExecuteStatusQuery_server;
    rclcpp::Service<robot_msgs::srv::ModeChangePauseTask>::SharedPtr ModeChangePauseTask_server;
    rclcpp::Service<robot_msgs::srv::ModeChangeStartTask>::SharedPtr ModeChangeStartTask_server;

    std::thread spinner_thread_;
};


#endif