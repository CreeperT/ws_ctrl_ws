#ifndef _WS_MSGS_MANAGE_H_
#define _WS_MSGS_MANAGE_H_

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

class WSmsgs_Manager : public rclcpp::Node
{
public:
    WSmsgs_Manager(const char* node_name);

    // 初始化函数
    bool DeviceParamInit();
    void NodePublisherInit();
    void NodeSubscriberInit();
    void NodeServiceClientInit();
    void NodeSpinnerStartup();

    // WebSocket通信
    void WSmsgsReceiveCallback(const std_msgs::msg::String::SharedPtr msg);
    void WSreceiveJsonParse(const std_msgs::msg::String::SharedPtr msg);
    void WSsendJsonBack(const cJSON* json_fun, const cJSON* rt_info, const cJSON* rt_data);
    void WSsendJsonCmd(const cJSON* json_fun, const cJSON* cmd_data);
    // void WSsendJsonUpload(const cJSON* json_fun, const cJSON* up_data);
    // void WSsendJsonHeartbeatBag(const cJSON* json_fun, const cJSON* heartbeat_bag);

    // 错误处理
    void WrongParamProcess(const char* err_msg, const cJSON* json_fun); // 1002
    void DeviceInternalCommunicationErrorProcess(const cJSON* json_fun); // 1003
    void RequestResourceNotExistProcess(const cJSON* json_fun); // 1005
    void WrongIdDeviceProcess(const cJSON* json_fun); // 1007
    void DeviceOperationFailedProcess(const cJSON* json_fun); // 2201
    // void DeviceNotSupportProcess(const cJSON* json_fun); // 2202
    void DeviceModeErrorProcess(const cJSON* json_fun); // 2203
    // void DeleteTaskFromDeviceFailProcess(const cJSON* json_fun); // 2601
    // void SendTask2DeviceFailProcess(const cJSON* json_fun); // 2602
    // void TaskNotExistProcess(const cJSON* json_fun); // 2603
    // void TaskInstanceNotExistProcess(const cJSON* json_fun); // 2604
    // void TaskNameExistProcess(const cJSON* json_fun); // 2605
    // void TaskPlanNotExistProcess(const cJSON* json_fun); // 2606

    // 机器人控制
    void DeviceCtrlCmdJsonParse(const cJSON* json_fun, const char* sub_function, const cJSON* cmd_data);
    void ChangeCtrlModeCmdProcess(const cJSON* json_fun, const uint8_t target_ctrl_mode);
    void MotorCtrlCmdProcess(const cJSON* json_fun, const cJSON* ctrl_value);
    void ChangeCtrlModeCmdSendback(const cJSON* json_fun, const uint8_t target_ctrl_mode);
    void DeviceCtrlCmdSendback(const cJSON* json_fun, const char* ctrl_type);

    // 系统时间校准
    // void SystemTimeSyncCmdProcess(const cJSON* json_fun, const cJSON* cmd_data);
    // void SystemTimeSyncCmdSendback(const cJSON* json_fun, const char* system_time_sync);

    // 机器人设备相关信息
    std::string id_device;
    std::string device_type;
    std::string devel_mode;
    std::string TrackArmMode;

    // cjson互斥锁
    std::mutex json_mutex_;

    void joinSpinnerThread() 
    {
        if (spinner_thread_.joinable()) 
            spinner_thread_.join();
    }

private:
    // WebSocket通信
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr WSsend_msgs_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr WSreceive_msgs_sub;

    // 机器人控制
    rclcpp::Publisher<robot_msgs::msg::MotorCtrlNormal>::SharedPtr MotorCtrlNormalCmd_pub;
    rclcpp::Client<robot_msgs::srv::ChangeCtrlModeCmd>::SharedPtr ChangeCtrlModeCmd_client;
    rclcpp::Client<robot_msgs::srv::CtrlModeQuery>::SharedPtr CtrlModeQuery_client;

    // 设备时间校准
    rclcpp::Client<robot_msgs::srv::SystemTimeSyncCmd>::SharedPtr SystemTimeSyncCmd_client;

    // Spin逻辑
    std::thread spinner_thread_;
};

#endif