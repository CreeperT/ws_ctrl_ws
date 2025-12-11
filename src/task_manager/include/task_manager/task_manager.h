#ifndef _TASK_MANAGER_H_
#define _TASK_MANAGER_H_

#include <rclcpp/rclcpp.hpp>

#include "task_file_manager.h"
#include "task_execute.h"

class Task_Manager : public rclcpp::Node
{
public:
    Task_Manager(const char* node_name);
    ~Task_Manager();

    bool DeviceParamInit();
    void NodePublisherInit();
    void NodeServiceServerInit();

    bool InspectTaskInfoListQueryHandle(robot_msgs::srv::InspectTaskInfoListQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoListQuery::Response::SharedPtr res);
    bool InspectTaskInfoDetailQueryHandle(robot_msgs::srv::InspectTaskInfoDetailQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoDetailQuery::Response::SharedPtr res);
    bool InspectTaskInfoConfigureCmdHandle(robot_msgs::srv::InspectTaskInfoConfigureCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoConfigureCmd::Response::SharedPtr res);
    bool InspectTaskInfoDeleteCmdHandle(robot_msgs::srv::InspectTaskInfoDeleteCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoDeleteCmd::Response::SharedPtr res);

    bool InspectTaskPlanListQueryHandle(robot_msgs::srv::InspectTaskPlanListQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanListQuery::Response::SharedPtr res);
    bool InspectTaskPlanAddCmdHandle(robot_msgs::srv::InspectTaskPlanAddCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanAddCmd::Response::SharedPtr res);
    bool InspectTaskPlanDeleteCmdHandle(robot_msgs::srv::InspectTaskPlanDeleteCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanDeleteCmd::Response::SharedPtr res);

    bool InspectTaskCtrlCmdHandle(robot_msgs::srv::InspectTaskCtrlCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr res);

    bool InspectTaskInstanceQueryHandle(robot_msgs::srv::InspectTaskInstanceQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInstanceQuery::Response::SharedPtr res);

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
    // Spin逻辑
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread spinner_thread_;

    rclcpp::Service<robot_msgs::srv::InspectTaskInfoListQuery>::SharedPtr InspectTaskInfoListQuery_server;
    rclcpp::Service<robot_msgs::srv::InspectTaskInfoDetailQuery>::SharedPtr InspectTaskInfoDetailQuery_server;
    rclcpp::Service<robot_msgs::srv::InspectTaskInfoConfigureCmd>::SharedPtr InspectTaskInfoConfigureCmd_server;
    rclcpp::Service<robot_msgs::srv::InspectTaskInfoDeleteCmd>::SharedPtr InspectTaskInfoDeleteCmd_server;

    rclcpp::Service<robot_msgs::srv::InspectTaskPlanListQuery>::SharedPtr InspectTaskPlanListQuery_server;
    rclcpp::Service<robot_msgs::srv::InspectTaskPlanAddCmd>::SharedPtr InspectTaskPlanAddCmd_server;
    rclcpp::Service<robot_msgs::srv::InspectTaskPlanDeleteCmd>::SharedPtr InspectTaskPlanDeleteCmd_server;

    rclcpp::Service<robot_msgs::srv::InspectTaskCtrlCmd>::SharedPtr InspectTaskCtrlCmd_server;

    rclcpp::Service<robot_msgs::srv::InspectTaskInstanceQuery>::SharedPtr InspectTaskInstanceQuery_server;

    std::string id_device;
    std::string device_type;
    std::string devel_mode;
    std::string TrackArmMode;

    uint16_t ImgDataCollectWaitTime;
    uint16_t TaskStartOrFinishResponseDuration;
    uint16_t WiperWorkDuration;

    std::shared_ptr<Task_File_Manager> pTaskFileManager;
    std::shared_ptr<Task_Execute> pTaskExecute;
};

#endif