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

#include "robot_msgs/srv/inspect_task_info_list_query.hpp"    // 巡检任务列表信息查询srv
#include "robot_msgs/srv/inspect_task_info_detail_query.hpp"  // 单个巡检任务详细信息查询srv
#include "robot_msgs/srv/inspect_task_plan_list_query.hpp"    // 巡检任务计划列表查询srv
#include "robot_msgs/srv/inspect_task_plan_add_cmd.hpp"       // 添加巡检任务计划srv
#include "robot_msgs/srv/inspect_task_plan_delete_cmd.hpp"    // 删除巡检任务计划srv
#include "robot_msgs/srv/inspect_task_ctrl_cmd.hpp"           // 巡检控制srv
#include "robot_msgs/srv/inspect_task_instance_query.hpp"     // 巡检任务实例查询srv
#include "robot_msgs/msg/inspect_task_execution_start.hpp"    // 巡检任务开始执行msg
#include "robot_msgs/msg/inspect_task_execution_complete.hpp" // 巡检任务执行完成msg
#include "robot_msgs/srv/inspect_task_info_configure_cmd.hpp" // 巡检任务配置srv
#include "robot_msgs/srv/inspect_task_info_delete_cmd.hpp"    // 删除巡检任务srv
#include "robot_msgs/srv/task_start_or_finish_response.hpp"   // 任务添加或删除srv

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

    // 巡检任务管理
    void InspectTaskCmdJsonPares(const cJSON *json_fun, const cJSON *cmd_data); // 巡检任务管理相关json解析

    void InspectTaskInfoCmdProcess(const cJSON *json_fun, const cJSON *cmd_data);       // 巡检任务管理任务信息类指令处理
    void InspectTaskPlanCmdProcess(const cJSON *json_fun, const cJSON *cmd_data);       // 巡检任务管理任务计划类指令处理
    void InspectTaskCtrlCmdProcess(const cJSON *json_fun, const cJSON *cmd_data);       // 巡检任务管理任务控制类指令处理
    void ProcessStartTask(cJSON *json_fun, const cJSON *cmd_data, cJSON *value_ctrl);   // 处理启动任务的子函数
    void ProcessControlTask(cJSON *json_fun, const cJSON *cmd_data, cJSON *value_ctrl); // 处理控制任务的子函数
    void InspectTaskInstanceCmdProcess(const cJSON *json_fun, const cJSON *cmd_data);   // 巡检任务管理任务实例类指令处理

    void InspectTaskInfoListQueryProcess(const cJSON *json_fun, const cJSON *cmd_data);    // 查询巡检任务列表指令处理
    void InspectTaskInfoDetailQueryProcess(const cJSON *json_fun, const cJSON *cmd_data);  // 单个巡检任务详细信息查询指令处理
    void InspectTaskInfoConfigureCmdProcess(const cJSON *json_fun, const cJSON *cmd_data); // 配置单个巡检任务指令处理
    void InspectTaskInfoDeleteCmdProcess(const cJSON *json_fun, const cJSON *cmd_data);    // 删除单个巡检任务指令处理

    void InspectTaskPlanListQueryProcess(const cJSON *json_fun, const cJSON *cmd_data); // 查询任务计划列表指令处理
    void InspectTaskPlanAddCmdProcess(const cJSON *json_fun, const cJSON *cmd_data);    // 添加巡检任务计划指令处理
    void InspectTaskPlanDeleteCmdProcess(const cJSON *json_fun, const cJSON *cmd_data); // 删除巡检任务计划指令处理

    void InspectTaskStartOrFinishServerRtProcess(const cJSON *json_fun, const cJSON *rt_data); // 巡检任务服务端回复命令处理

    void InspectTaskInfoListQuerySendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskInfoListQuery::Response::SharedPtr response); // 巡检任务列表查询正常JSON包返回
    void InspectTaskInfoDetailQuerySendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskInfoDetailQuery::Response::SharedPtr response,
                                            const robot_msgs::srv::InspectTaskInfoDetailQuery::Request::SharedPtr request); // 单个巡检任务详细信息查询正常JSON返回
    void InspectTaskInfoConfigureCmdSendback(const cJSON *json_fun, const char *id_task_info);                              // 配置单个巡检任务正常JSON返回
    void InspectTaskInfoDeleteCmdSendback(const cJSON *json_fun);                                                           // 删除单个巡检任务正常JSON返回

    void InspectTaskPlanListQuerySendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskPlanListQuery::Response::SharedPtr response,
                                          const robot_msgs::srv::InspectTaskPlanListQuery::Request::SharedPtr request); // 查询巡检任务计划列表正常JSON返回
    void InspectTaskPlanAddCmdSendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskPlanAddCmd::Response::SharedPtr response,
                                       const robot_msgs::srv::InspectTaskPlanAddCmd::Request::SharedPtr request); // 添加巡检任务计划正常JSON返回
    void InspectTaskPlanDeleteCmdSendback(const cJSON *json_fun);                                                 // 删除巡检任务计划正常JSON返回

    void InspectTaskCtrlCmdSendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr response); // 巡检任务控制正常JSON返回

    void InspectTaskInstanceCmdSendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskInstanceQuery::Response::SharedPtr response); // 查询巡检任务实例信息正常返回

    void InspectTaskExecutionStartCallback(const robot_msgs::msg::InspectTaskExecutionStart::SharedPtr msg);       // 巡检任务计划开始执行信息上报话题回调函数
    void InspectTaskExecutionCompleteCallback(const robot_msgs::msg::InspectTaskExecutionComplete::SharedPtr msg); // 巡检任务执行完成信息上报话题回调函数

    // 机器人信息查询
    void RobotInfoQueryProcess(const cJSON *json_fun);                                                                       // 机器人信息查询处理
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

    // 巡检任务管理
    rclcpp::Client<robot_msgs::srv::InspectTaskInfoListQuery>::SharedPtr InspectTaskInfoListQuery_client;            // 巡检任务列表查询客户端
    rclcpp::Client<robot_msgs::srv::InspectTaskInfoDetailQuery>::SharedPtr InspectTaskInfoDetailQuery_client;        // 单个巡检任务详细信息查询客户端
    rclcpp::Client<robot_msgs::srv::InspectTaskInfoConfigureCmd>::SharedPtr InspectTaskInfoConfigureCmd_client;      // 配置单个巡检任务客户端
    rclcpp::Client<robot_msgs::srv::InspectTaskInfoDeleteCmd>::SharedPtr InspectTaskInfoDeleteCmd_client;            // 删除单个巡检任务客户端
    rclcpp::Client<robot_msgs::srv::InspectTaskPlanListQuery>::SharedPtr InspectTaskPlanListQuery_client;            // 查询巡检任务计划列表客户端
    rclcpp::Client<robot_msgs::srv::InspectTaskPlanAddCmd>::SharedPtr InspectTaskPlanAddCmd_client;                  // 添加巡检任务计划客户端
    rclcpp::Client<robot_msgs::srv::InspectTaskPlanDeleteCmd>::SharedPtr InspectTaskPlanDeleteCmd_client;            // 删除巡检任务计划客户端
    rclcpp::Client<robot_msgs::srv::InspectTaskCtrlCmd>::SharedPtr InspectTaskCtrlCmd_client;                        // 巡检任务控制客户端
    rclcpp::Client<robot_msgs::srv::InspectTaskInstanceQuery>::SharedPtr InspectTaskInstanceQuery_client;            // 巡检任务实例查询客户端
    rclcpp::Client<robot_msgs::srv::TaskStartOrFinishResponse>::SharedPtr TaskStartOrFinishResponse_client;          // 任务开始或结束响应客户端
    rclcpp::Subscription<robot_msgs::msg::InspectTaskExecutionStart>::SharedPtr InspectTaskExecutionStart_sub;       // 巡检任务计划开始执行信息上报话题订阅
    rclcpp::Subscription<robot_msgs::msg::InspectTaskExecutionComplete>::SharedPtr InspectTaskExecutionComplete_sub; // 巡检任务执行完成信息上报话题订阅

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