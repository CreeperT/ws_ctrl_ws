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

typedef struct
{
    double target_pos_x;
    double target_pos_y;
    double target_pos_z;
    double target_ori_z;
    double target_ori_w;
} BatteryChargeTargetNavPose;


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

    // 心跳包
    void RobotHeartbeatBagPub();
    
    // 机器人控制相关
    bool ChangeCtrlModeCmdHandle(const robot_msgs::srv::ChangeCtrlModeCmd::Request::SharedPtr req,
                                 robot_msgs::srv::ChangeCtrlModeCmd::Response::SharedPtr res);
    bool CtrlModeQueryHandle(const robot_msgs::srv::CtrlModeQuery::Request::SharedPtr req,
                             robot_msgs::srv::CtrlModeQuery::Response::SharedPtr res);
    void MotorCtrlNormalCmdCallback(const robot_msgs::msg::MotorCtrlNormal::SharedPtr msg);
    void MotorCtrltoPosCmdCallback(const robot_msgs::msg::MotorCtrltoPos::SharedPtr msg);
    void RobotGoHomeBatteryCharge();

    // 外设相关
    void ArmCtrlCmdCallback(const robot_msgs::msg::ArmCtrl::SharedPtr msg);
    // void LiftCtrlCmdCallback(const robot_msgs::msg::LiftCtrl::SharedPtr msg);
    void ExtendDevCtrlCmdCallback(const robot_msgs::msg::ExtendDevCtrl::SharedPtr msg);

    // 机器人状态
    void RobotRuntimePositionCallback(const robot_msgs::msg::RobotRuntimePosition::SharedPtr msg);
    void RobotBatteryStatusCallback(const robot_msgs::msg::RobotBatteryStatus::SharedPtr msg);

    void TaskFinishAndExecuteBatteryChargeCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // void AvoidObstaclesStartOrFinishCallback(const std_msgs::msg::String::SharedPtr msg);

    void BatteryPercentManage();

    void ManualMode2TaskModeAutoCheck();

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
    // 发布者
    rclcpp::Publisher<robot_msgs::msg::MotorCtrlNormal>::SharedPtr QuadrupedalMotorCtrlNormal_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorCtrltoPos>::SharedPtr QuadrupedalMotorCtrltoPos_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr QuadrupedalMotorStopCtrl_pub;
    rclcpp::Publisher<robot_msgs::msg::ArmCtrl>::SharedPtr ArmCtrlMoveCmd_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ArmCtrlStopCmd_pub;
    rclcpp::Publisher<robot_msgs::msg::ExtendDevCtrl>::SharedPtr ExtendDevCtrlInternal_pub;

    // 订阅者
    rclcpp::Subscription<robot_msgs::msg::MotorCtrlNormal>::SharedPtr MotorCtrlNormalCmd_sub;
    rclcpp::Subscription<robot_msgs::msg::MotorCtrltoPos>::SharedPtr MotorCtrltoPosCmd_sub;
    rclcpp::Subscription<robot_msgs::msg::ArmCtrl>::SharedPtr ArmCtrl_sub;
    rclcpp::Subscription<robot_msgs::msg::ExtendDevCtrl>::SharedPtr ExtendDevCtrlCmd_sub;
    rclcpp::Subscription<robot_msgs::msg::RobotRuntimePosition>::SharedPtr RobotRuntimePosition_sub;
    rclcpp::Subscription<robot_msgs::msg::RobotBatteryStatus>::SharedPtr RobotBatteryStatus_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr TaskFinishAndExecuteBatteryCharge_sub;


    // 服务端
    rclcpp::Service<robot_msgs::srv::CtrlModeQuery>::SharedPtr CtrlModeQuery_server;
    rclcpp::Service<robot_msgs::srv::ChangeCtrlModeCmd>::SharedPtr ChangeCtrlModeCmd_server;

    // 客户端
    rclcpp::Client<robot_msgs::srv::BatteryChargeNav>::SharedPtr BatteryChargeNav_client;
    rclcpp::Client<robot_msgs::srv::TaskExecuteStatusQuery>::SharedPtr TaskExecuteStatusQuery_client;
    rclcpp::Client<robot_msgs::srv::BatteryChargeExecuteProgram>::SharedPtr BatteryChargeExecuteProgram_client;
    rclcpp::Client<robot_msgs::srv::BatteryChargePauseTask>::SharedPtr BatteryChargePauseTask_client;
    rclcpp::Client<robot_msgs::srv::BatteryChargeStartTask>::SharedPtr BatteryChargeStartTask_client;
    rclcpp::Client<robot_msgs::srv::ModeChangePauseTask>::SharedPtr ModeChangePauseTask_client;
    rclcpp::Client<robot_msgs::srv::ModeChangeStartTask>::SharedPtr ModeChangeStartTask_client;

    // 机器人心跳包
    rclcpp::Publisher<robot_msgs::msg::HeartbeatBag>::SharedPtr HeartbeatBag_pub;

    // 定时器
    rclcpp::TimerBase::SharedPtr HeartbeatBag_timer;
    rclcpp::TimerBase::SharedPtr BatteryPercentManage_timer;
    rclcpp::TimerBase::SharedPtr ManualMode2TaskModeAutoCheck_timer;

    // 回调组
    rclcpp::CallbackGroup::SharedPtr HeartbeatBag_cb_group_;
    rclcpp::CallbackGroup::SharedPtr BatteryPercentManage_cb_group_;
    rclcpp::CallbackGroup::SharedPtr ManualMode2TaskModeAutoCheck_cb_group_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr server_cb_group_;

    // 参数
    std::string device_type;
    std::string devel_mode;

    uint16_t battery_percent;
    uint16_t battery_percent_low_threshold;
    uint16_t battery_percent_high_threshold;
    uint16_t battery_percent_full_threshold;
    bool battery_charge_lock;
    bool execute_battery_charge;
    bool finish_battery_charge_and_execute_task;

    BatteryChargeTargetNavPose batteryChargeTargetNavPose;

    std::time_t operate_manual_last_time;
    uint16_t ManualMode2TaskModeDuration;

    std::string runtime_pos_id_track;
    uint64_t runtime_pos_track_pos;
    uint16_t ctrl_mode; // 机器人控制模式: 0-任务模式，1-后台手动遥控模式，2-紧急定位模式，3-手持遥控模式
    uint16_t runtime_status; // 机器人运行状态：0-空闲，1-手动操作，2-执行巡检任务，3-充电状态
    std::string executing_task_ticket;
    uint16_t charge_status; // 机器人充电状态：0-不在充电，1-正在充电
    uint64_t timestamp;
    uint16_t error_code;

    std::string AvoidObstaclesPauseTaskOrMove;

    std::mutex mutex_;

};

#endif