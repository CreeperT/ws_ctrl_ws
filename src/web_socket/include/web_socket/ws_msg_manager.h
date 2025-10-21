#ifndef _WS_MSGS_MANAGER_H_
#define _WS_MSGS_MANAGER_H_

#include "rclcpp/rclcpp.hpp"
#include <cjson/cJSON.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <string>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include "robot_msgs/msg/motor_ctrl_normal.hpp"    // 控制机器人移动msg
#include "robot_msgs/msg/motor_ctrlto_pos.hpp"     // 控制机器人点到点导航msg
#include "robot_msgs/srv/ctrl_mode_query.hpp"      // 查询机器人模式srv
#include "robot_msgs/srv/change_ctrl_mode_cmd.hpp" // 切换机器人模式srv
#include "robot_msgs/srv/system_time_sync_cmd.hpp" // 设备时间校准srv
#include "robot_msgs/msg/heartbeat_bag.hpp"        // 机器人心跳包msg
#include "robot_msgs/srv/robot_info_query.hpp"     // 机器人信息srv

class WSmsgs_Manager : public rclcpp::Node
{
public:
    WSmsgs_Manager(const char *node_name);

    // 初始化函数
    bool DeviceParamInit();       // 设备参数初始化
    void NodePublisherInit();     // 节点话题发布初始化
    void NodeSubscriberInit();    // 节点话题订阅初始化
    void NodeServiceClientInit(); // 节点服务客户端初始化
    void NodeSpinnerStartup();    // 节点启动

    // WebSocket通信
    void WSmsgsReceiveCallback(const std_msgs::msg::String::SharedPtr msg);                 // websocket消息接收订阅话题回调函数
    void WSreceiveJsonParse(const std_msgs::msg::String::SharedPtr msg);                    // websocket消息json包解析
    void WSsendJsonBack(const cJSON *json_fun, const cJSON *rt_info, const cJSON *rt_data); // websocket指令json包返回数据发送
    void WSsendJsonCmd(const cJSON *json_fun, const cJSON *cmd_data);                       // websocket发送机器人对服务器的指令
    void WSsendJsonUpload(const cJSON *json_fun, const cJSON *up_data);                     // websocket发送机器人上报服务器数据
    void WSsendJsonHeartbeatBag(const cJSON *json_fun, const cJSON *heartbeat_bag);         // websocket发送机器人心跳包

    // 错误处理
    void WrongParamProcess(const char *err_msg, const cJSON *json_fun);  // 1002
    void DeviceInternalCommunicationErrorProcess(const cJSON *json_fun); // 1003
    void RequestResourceNotExistProcess(const cJSON *json_fun);          // 1005
    void WrongIdDeviceProcess(const cJSON *json_fun);                    // 1007
    void DeviceOperationFailedProcess(const cJSON *json_fun);            // 2201
    void DeviceNotSupportProcess(const cJSON *json_fun);                 // 2202
    void DeviceModeErrorProcess(const cJSON *json_fun);                  // 2203
    void DeleteTaskFromDeviceFailProcess(const cJSON *json_fun);         // 2601
    void SendTask2DeviceFailProcess(const cJSON *json_fun);              // 2602
    void TaskNotExistProcess(const cJSON *json_fun);                     // 2603
    void TaskInstanceNotExistProcess(const cJSON *json_fun);             // 2604
    void TaskNameExistProcess(const cJSON *json_fun);                    // 2605
    void TaskPlanNotExistProcess(const cJSON *json_fun);                 // 2606

    // 机器人控制
    void DeviceCtrlCmdJsonParse(const cJSON *json_fun, const char *sub_function, const cJSON *cmd_data); // 机器人控制相关json解析
    void ChangeCtrlModeCmdProcess(const cJSON *json_fun, const uint8_t target_ctrl_mode);                // 切换控制模式指令处理
    void MotorCtrlCmdProcess(const cJSON *json_fun, const cJSON *ctrl_value);                            // 移动平台运动控制指令处理
    void ChangeCtrlModeCmdSendback(const cJSON *json_fun, const uint8_t target_ctrl_mode);               // 切换控制模式若正常，返回json包
    void DeviceCtrlCmdSendback(const cJSON *json_fun, const char *ctrl_type);                            // 设备控制若正常，返回json包

    // 机器人信息查询
    void RobotInfoQueryProcess(const cJSON *json_fun);
    void RobotInfoQuerySendback(const cJSON *json_fun, const robot_msgs::srv::RobotInfoQuery::Response::SharedPtr response); // 机器人信息查询若正常，返回json包

    // 机器人心跳包
    void HeartbeatBagCallback(const robot_msgs::msg::HeartbeatBag::SharedPtr msg); // 心跳包订阅回调

    // 系统时间校准
    void SystemTimeSyncCmdProcess(const cJSON *json_fun, const cJSON *cmd_data);
    void SystemTimeSyncCmdSendback(const cJSON *json_fun, const char *system_time_sync);

    inline std::string getCurrentTimeStr() // 获得当前时间戳
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

private:
    // WebSocket通信
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr WSsend_msgs_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr WSreceive_msgs_sub;

    // 机器人控制
    rclcpp::Publisher<robot_msgs::msg::MotorCtrlNormal>::SharedPtr MotorCtrlNormalCmd_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorCtrltoPos>::SharedPtr MotorCtrltoPosCmd_pub;
    rclcpp::Client<robot_msgs::srv::ChangeCtrlModeCmd>::SharedPtr ChangeCtrlModeCmd_client;
    rclcpp::Client<robot_msgs::srv::CtrlModeQuery>::SharedPtr CtrlModeQuery_client;

    // 机器人信息查询
    rclcpp::Client<robot_msgs::srv::RobotInfoQuery>::SharedPtr RobotInfoQuery_client;

    // 机器人心跳包
    rclcpp::Subscription<robot_msgs::msg::HeartbeatBag>::SharedPtr HeartbeatBag_sub;

    // 设备时间校准
    rclcpp::Client<robot_msgs::srv::SystemTimeSyncCmd>::SharedPtr SystemTimeSyncCmd_client;

    // Spin逻辑
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread spinner_thread_;

    // 机器人设备相关信息
    std::string id_device;
    std::string device_type;
    std::string devel_mode;
    std::string TrackArmMode;

    // cjson互斥锁
    // std::mutex json_mutex_;
};

#endif