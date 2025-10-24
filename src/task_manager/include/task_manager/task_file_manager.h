#ifndef _TASK_FILE_MANAGER_H_
#define _TASK_FILE_MANAGER_H_

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <random>
#include <regex>
#include <algorithm>
#include <stdexcept>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <dirent.h>
#include <sys/types.h>
#include <cjson/cJSON.h>

#include "robot_msgs/srv/inspect_task_info_list_query.hpp"
#include "robot_msgs/srv/inspect_task_info_detail_query.hpp"
#include "robot_msgs/srv/inspect_task_info_configure_cmd.hpp"
#include "robot_msgs/srv/inspect_task_info_delete_cmd.hpp"

#include "robot_msgs/srv/inspect_task_plan_list_query.hpp"
#include "robot_msgs/srv/inspect_task_plan_add_cmd.hpp"
#include "robot_msgs/srv/inspect_task_plan_delete_cmd.hpp"

#include "robot_msgs/srv/inspect_task_instance_query.hpp"

#include <std_msgs/msg/string.hpp>

class Task_File_Manager : public rclcpp::Node
{
public:
    Task_File_Manager(const std::string id_device_, const std::string devel_mode_);

    // 初始化
    void getTaskFilesDir();

    std::string generateRandomUUID();

    // 任务信息
    bool TaskInfoListQuery(const robot_msgs::srv::InspectTaskInfoListQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoListQuery::Response::SharedPtr res);
    bool TaskInfoDetailQuery(const robot_msgs::srv::InspectTaskInfoDetailQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoDetailQuery::Response::SharedPtr res);
    bool TaskInfoConfigure(const robot_msgs::srv::InspectTaskInfoConfigureCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoConfigureCmd::Response::SharedPtr res);
    bool TaskInfoCreate(const robot_msgs::srv::InspectTaskInfoConfigureCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoConfigureCmd::Response::SharedPtr res);
    bool TaskInfoDelete(const robot_msgs::srv::InspectTaskInfoDeleteCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoDeleteCmd::Response::SharedPtr res);

    // 任务计划
    bool TaskPlanListQuery(const robot_msgs::srv::InspectTaskPlanListQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanListQuery::Response::SharedPtr res);
    bool TaskPlanAdd(const robot_msgs::srv::InspectTaskPlanAddCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanAddCmd::Response::SharedPtr res);
    bool TaskPlanDelete(const robot_msgs::srv::InspectTaskPlanDeleteCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanDeleteCmd::Response::SharedPtr res);

    // 任务实体
    bool TaskInstanceQuery(const robot_msgs::srv::InspectTaskInstanceQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInstanceQuery::Response::SharedPtr res);

    inline std::string getCurrentTimeStr()
    {
        time_t now = time(NULL);
        struct tm localt;
        localtime_r(&now, &localt);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);
        return time_buf;
    }

public:
    std::string TaskFilesDir;
    std::string id_device;
    std::string devel_mode;
};

#endif