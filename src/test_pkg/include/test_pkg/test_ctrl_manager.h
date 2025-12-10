#ifndef _TEST_CTRL_MANAGER_H_
#define _TEST_CTRL_MANAGER_H_

#include "rclcpp/rclcpp.hpp"
#include <cjson/cJSON.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <string>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include "robot_msgs/msg/motor_ctrl_normal.hpp" // 控制机器人移动msg
#include "robot_msgs/msg/motor_ctrlto_pos.hpp" // 机器人点到点移动msg
#include "robot_msgs/srv/ctrl_mode_query.hpp" // 查询机器人模式srv
#include "robot_msgs/msg/arm_ctrl.hpp" // 机械臂控制msg
#include "robot_msgs/msg/lift_ctrl.hpp" // 升降平台控制msg
#include "robot_msgs/msg/extend_dev_ctrl.hpp" // 外设控制msg
#include "robot_msgs/msg/robot_runtime_position.hpp" // 机器人运行时位置msg
#include "robot_msgs/msg/robot_battery_status.hpp" // 机器人电池状态msg
#include "robot_msgs/srv/change_ctrl_mode_cmd.hpp" // 切换机器人模式srv
#include "robot_msgs/srv/battery_charge_nav.hpp" // 电池充电导航srv
#include "robot_msgs/srv/task_execute_status_query.hpp" // 任务执行状态srv
#include "robot_msgs/srv/battery_charge_execute_program.hpp" // 电池充电执行程序srv
#include "robot_msgs/srv/battery_charge_pause_task.hpp" // 电池充电暂停srv
#include "robot_msgs/srv/battery_charge_start_task.hpp" // 电池充电开始srv
#include "robot_msgs/srv/mode_change_pause_task.hpp" // 切换模式暂停srv
#include "robot_msgs/srv/mode_change_start_task.hpp" // 切换模式开始srv
#include "robot_msgs/srv/system_time_sync_cmd.hpp" // 设备时间校准srv
#include "robot_msgs/msg/heartbeat_bag.hpp" // 机器人心跳包

class Ctrl_Manager : public rclcpp::Node
{
public:
    Ctrl_Manager(const char* node_name);

    // 初始化函数
    bool DeviceParamInit();
    void NodePublisherInit();
    void NodeSubscriberInit();
    void NodeServiceServerInit();
    void NodeServiceClientInit();
    void NodeSpinnerStartup();

    bool ChangeCtrlModeCmdHandle(const robot_msgs::srv::ChangeCtrlModeCmd::Request::SharedPtr req,
                                 robot_msgs::srv::ChangeCtrlModeCmd::Response::SharedPtr res);

    inline std::string getCurrentTimeStr()
    {
        time_t now = time(NULL);
        struct tm localt;
        localtime_r(&now, &localt);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);
        return time_buf;
    }

    void joinSpinnerThread() 
    {
        if (spinner_thread_.joinable()) 
            spinner_thread_.join();
    }

public:
    rclcpp::Service<robot_msgs::srv::ChangeCtrlModeCmd>::SharedPtr ChangeCtrlModeCmd_server;

    rclcpp::Client<robot_msgs::srv::TaskExecuteStatusQuery>::SharedPtr TaskExecuteStatusQuery_client;
    rclcpp::Client<robot_msgs::srv::ModeChangePauseTask>::SharedPtr ModeChangePauseTask_client;
    rclcpp::Client<robot_msgs::srv::ModeChangeStartTask>::SharedPtr ModeChangeStartTask_client;
    
    std::thread spinner_thread_;

    std::string device_type;
    std::string devel_mode;
    uint16_t ctrl_mode;
    uint16_t ManualMode2TaskModeDuration;
    uint16_t battery_percent_low_threshold;
    uint16_t battery_percent_high_threshold;
    uint16_t battery_percent_full_threshold;
    bool battery_charge_lock;
    std::time_t operate_manual_last_time;
};

#endif