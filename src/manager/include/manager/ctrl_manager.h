#ifndef _CTRL_MANAGER_H_
#define _CTRL_MANAGER_H_

#include "rclcpp/rclcpp.hpp"
#include <cjson/cJSON.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <string>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include "robot_msgs/msg/motor_ctrl_normal.hpp" // 控制机器人移动msg
#include "robot_msgs/srv/ctrl_mode_query.hpp" // 查询机器人模式srv
#include "robot_msgs/srv/change_ctrl_mode_cmd.hpp" // 切换机器人模式srv
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

    void RobotHeartbeatBagPub();
    

    // 中断函数
    bool ChangeCtrlModeCmdHandle(const robot_msgs::srv::ChangeCtrlModeCmd::Request::SharedPtr req,
                                 robot_msgs::srv::ChangeCtrlModeCmd::Response::SharedPtr res);
    bool CtrlModeQueryHandle(const robot_msgs::srv::CtrlModeQuery::Request::SharedPtr req,
                             robot_msgs::srv::CtrlModeQuery::Response::SharedPtr res);
    
    // 回调函数
    // void MotorCtrlNormalCmdCallback(const robot_msgs::msg::MotorCtrlNormal::SharedPtr msg);

    // 参数
    std::string device_type;
    std::string devel_mode;
    uint16_t battery_percent;
    uint16_t battery_percent_low_threshold;
    uint16_t battery_percent_high_threshold;
    uint16_t battery_percent_full_threshold;

    uint16_t ManualMode2TaskModeDuration;

    std::string runtime_pos_id_track;  
    uint64_t runtime_pos_track_pos;    
    uint16_t ctrl_mode;
    uint16_t runtime_status;
    std::string executing_task_ticket;  
    uint16_t charge_status;
    uint64_t timestamp;
    uint16_t error_code;


    void joinSpinnerThread() 
    {
        if (spinner_thread_.joinable()) 
            spinner_thread_.join();
    }

private:
    // 机器人控制
    rclcpp::Subscription<robot_msgs::msg::MotorCtrlNormal>::SharedPtr MotorCtrlNormalCmd_sub;
    rclcpp::Service<robot_msgs::srv::CtrlModeQuery>::SharedPtr CtrlModeQuery_server;
    rclcpp::Service<robot_msgs::srv::ChangeCtrlModeCmd>::SharedPtr ChangeCtrlModeCmd_server;

    // 机器人心跳包
    rclcpp::Publisher<robot_msgs::msg::HeartbeatBag>::SharedPtr HeartbeatBag_pub;
    rclcpp::TimerBase::SharedPtr HeartbeatBag_timer;
    

    // Spin逻辑
    std::thread spinner_thread_;
};

#endif