#ifndef _TASK_EXECUTE_H_
#define _TASK_EXECUTE_H_

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <random>
#include <regex>
#include <cstdlib>
#include <cstdio>
#include <dirent.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <thread>
#include <sys/types.h>
#include <cjson/cJSON.h>

#include "robot_msgs/srv/motor_ctrlto_pos_srv.hpp"
#include "robot_msgs/srv/arm_ctrl_srv.hpp"
#include "robot_msgs/srv/lift_ctrl_srv.hpp"
#include "robot_msgs/srv/extend_dev_ctrl_srv.hpp"
#include "robot_msgs/srv/task_data_collect_srv.hpp"

#include "robot_msgs/srv/inspect_task_ctrl_cmd.hpp"

#include "robot_msgs/msg/inspect_task_execution_start.hpp"
#include "robot_msgs/msg/inspect_task_execution_complete.hpp"

#include "robot_msgs/srv/task_start_or_finish_response.hpp"

#include "robot_msgs/srv/task_execute_status_query.hpp"

#include "robot_msgs/srv/ctrl_mode_query.hpp"

#include "robot_msgs/srv/battery_charge_pause_task.hpp"
#include "robot_msgs/srv/battery_charge_start_task.hpp"
#include "robot_msgs/srv/mode_change_pause_task.hpp"
#include "robot_msgs/srv/mode_change_start_task.hpp"

#include "robot_msgs/srv/avoid_obstacles_pause_task.hpp"
#include "robot_msgs/srv/avoid_obstacles_start_task.hpp"

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

#include "cron_analyse.h"

typedef struct
{
    std::string task_execute_type;
    std::string task_ticket;
    std::string id_task_info;
    std::string task_execute_flag;
    std::string task_type;
    std::string task_name;
    std::string id_task_plan;
    uint64_t point_count;
    bool execute_monitor_point_lock;
    bool execute_robot_home_lock;
    bool pause_auto_execute_first_time;
    bool pause_manual_execute_fitst_time;
    bool arm_home;
    bool execute_stop_flag;
} task_execute_info;

typedef struct
{
    std::vector<Cron_Analyse::CronTask> cron_task;
    std::string id_task_info;
    std::string task_type;
    std::string task_name;
    std::string id_task_plan;
    std::string plan_type;
    bool first_analysed;
} CronPlanTask;

typedef struct
{
    std::time_t msg_send_time;
    std::string start_or_finish;
    bool response_flag;
    robot_msgs::msg::InspectTaskExecutionStart InspectTaskExecutionStartMsg;
    robot_msgs::msg::InspectTaskExecutionComplete InspectTaskExecutionCompleteMsg;
} TaskStartOrFinishCache;

class Task_Execute : public rclcpp::Node
{
public:
    Task_Execute(const char* node_name, const std::string id_device_, const std::string device_type_, const std::string devel_mode_, const std::string TrackArmMode_,
                 const uint16_t ImgDataCollectWaitTime_, const uint16_t TaskStartOrFinishResponseDuration_, const uint16_t WiperWorkDuration_);

    // 初始化
    void getTaskFilesDir();

    void NodePublisherInit();
    void NodeServiceClientInit();
    void NodeServiceServerInit();

    std::string generateRandomUUID();

    // 任务控制
    bool TaskCtrlStart(robot_msgs::srv::InspectTaskCtrlCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr res);
    bool TaskCtrlPause(robot_msgs::srv::InspectTaskCtrlCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr res);
    bool TaskCtrlRestart(robot_msgs::srv::InspectTaskCtrlCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr res);
    bool TaskCtrlStop(robot_msgs::srv::InspectTaskCtrlCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr res);

    // 任务执行
    void TaskExecute(const std::string task_ticket);
    void ExecuteMonitorPoint(const task_execute_info task_execute_info_local, const std::string monitor_point, const std::string monitor_point_last, const std::string monitor_point_next, uint64_t point_count_local);
    bool TaskStartAuto(const std::string task_ticket);
    bool TaskPauseAuto(const std::string task_ticket);
    void TaskStop(const std::string task_ticket);
    void TaskFinish(const task_execute_info task_execute_info_local);
    void TaskOuttime(const std::string task_ticket);
    void PlanTaskStartAuto(const task_execute_info task_execute_info_local);
    bool ExecuteRobotHomeStatus(const task_execute_info task_execute_info_local);

    // 任务计划
    void CronPlanTaskListUpdate();
    void CronPlanTaskTrigger();
    bool GetTaskPlanArray(std::vector<std::pair<std::string, std::vector<std::pair<std::string, std::string>>>>& task_plan_array);
    bool GetCronPlanTaskInfo(CronPlanTask& single_cron_plan_task);
    void CronPlanTaskStart(CronPlanTask& single_cron_plan_task);

    // 任务执行初始化
    void TaskExecuteInit();

    // 服务中断函数
    bool TaskExecuteStatusQueryHandle(robot_msgs::srv::TaskExecuteStatusQuery::Request::SharedPtr req, robot_msgs::srv::TaskExecuteStatusQuery::Response::SharedPtr res);

    bool BatteryChargePauseTaskHandle(robot_msgs::srv::BatteryChargePauseTask::Request::SharedPtr req, robot_msgs::srv::BatteryChargePauseTask::Response::SharedPtr res);
    bool BatteryChargeStartTaskHandle(robot_msgs::srv::BatteryChargeStartTask::Request::SharedPtr req, robot_msgs::srv::BatteryChargeStartTask::Response::SharedPtr res);
    bool ModeChangePauseTaskHandle(robot_msgs::srv::ModeChangePauseTask::Request::SharedPtr req, robot_msgs::srv::ModeChangePauseTask::Response::SharedPtr res);
    bool ModeChangeStartTaskHandle(robot_msgs::srv::ModeChangeStartTask::Request::SharedPtr req, robot_msgs::srv::ModeChangeStartTask::Response::SharedPtr res);
    bool AvoidObstaclesPauseTaskHandle(robot_msgs::srv::AvoidObstaclesPauseTask::Request::SharedPtr req, robot_msgs::srv::AvoidObstaclesPauseTask::Response::SharedPtr res);
    bool AvoidObstaclesStartTaskHandle(robot_msgs::srv::AvoidObstaclesStartTask::Request::SharedPtr req, robot_msgs::srv::AvoidObstaclesStartTask::Response::SharedPtr res);

    bool TaskStartExecuteCheckRobotHome();

    bool TaskStartOrFinishResponseHandle(robot_msgs::srv::TaskStartOrFinishResponse::Request::SharedPtr req, robot_msgs::srv::TaskStartOrFinishResponse::Response::SharedPtr res);

    void TaskStartOrFinishResponseCheck();

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
    // 客户端
    rclcpp::Client<robot_msgs::srv::MotorCtrltoPosSrv>::SharedPtr MotorCtrltoPosSrv_client;
    rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedPtr ArmCtrlSrv_client;
    rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedPtr LiftCtrlSrv_client;
    rclcpp::Client<robot_msgs::srv::ExtendDevCtrlSrv>::SharedPtr ExtendDevCtrlSrv_client;
    rclcpp::Client<robot_msgs::srv::TaskDataCollectSrv>::SharedPtr TaskDataCollectSrv_client;
    
    rclcpp::Client<robot_msgs::srv::CtrlModeQuery>::SharedPtr CtrlModeQuery_client;

    // 服务端
    rclcpp::Service<robot_msgs::srv::TaskExecuteStatusQuery>::SharedPtr TaskExecuteStatusQuery_server;
    rclcpp::Service<robot_msgs::srv::BatteryChargePauseTask>::SharedPtr BatteryChargePauseTask_server;
    rclcpp::Service<robot_msgs::srv::BatteryChargeStartTask>::SharedPtr BatteryChargeStartTask_server;
    rclcpp::Service<robot_msgs::srv::ModeChangePauseTask>::SharedPtr ModeChangePauseTask_server;
    rclcpp::Service<robot_msgs::srv::ModeChangeStartTask>::SharedPtr ModeChangeStartTask_server;
    rclcpp::Service<robot_msgs::srv::AvoidObstaclesPauseTask>::SharedPtr AvoidObstaclesPauseTask_server;
    rclcpp::Service<robot_msgs::srv::AvoidObstaclesStartTask>::SharedPtr AvoidObstaclesStartTask_server;

    rclcpp::Service<robot_msgs::srv::TaskStartOrFinishResponse>::SharedPtr TaskStartOrFinishResponse_server;

    // 发布者
    rclcpp::Publisher<robot_msgs::msg::InspectTaskExecutionStart>::SharedPtr InspectTaskExecutionStart_pub;
    rclcpp::Publisher<robot_msgs::msg::InspectTaskExecutionComplete>::SharedPtr InspectTaskExecutionComplete_pub;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr TaskFinishAndExecuteBatteryCharge_pub;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr TrackMotorStopCtrl_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr MotorStopCtrl_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ArmCtrlStopCmd_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr LiftCtrlStopCmd_pub;

    //回调组
    rclcpp::CallbackGroup::SharedPtr server_callback_group_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;

    // 参数
    std::vector<task_execute_info> task_execute_infos;
    std::vector<CronPlanTask> cron_plan_task;
    std::string task_ticket_pause_by_battery_charge;
    std::string task_ticket_pause_by_mode_change;
    std::string task_ticket_pause_by_avoid_obstacle;

    std::vector<TaskStartOrFinishCache> TaskStartOrFinishCache_vector;

    std::string TaskFilesDir;
    std::string id_device;
    std::string device_type;
    std::string devel_mode;
    std::string TrackArmMode;

    uint16_t ImgDataCollectWaitTime;
    uint16_t TaskStartOrFinishResponseDuration;
    uint16_t WiperWorkDuration;
};

#endif