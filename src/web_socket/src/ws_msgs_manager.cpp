#include "ws_msg_manager.h"
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

/***********************************************初始化相关***********************************************/

WSmsgs_Manager::WSmsgs_Manager(const char *node_name) : Node(node_name)
{
    if (DeviceParamInit())
    {
        NodePublisherInit();
        NodeSubscriberInit();
        NodeServiceClientInit();
    }
}

bool WSmsgs_Manager::DeviceParamInit()
{
    const char *homeDir = getenv("HOME");
    std::string jsonDir = homeDir;
    jsonDir = jsonDir + "/ws_ctrl_ws/src/DeviceParam.json";

    std::ifstream DeviceParamFile(jsonDir.c_str());
    if (!DeviceParamFile)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Can't open device param file!");
        return false;
    }

    std::string params((std::istreambuf_iterator<char>(DeviceParamFile)), std::istreambuf_iterator<char>());
    DeviceParamFile.close();

    cJSON *value_device_param = cJSON_Parse(params.c_str());
    if (value_device_param != NULL)
    {
        cJSON *value_device_type = cJSON_GetObjectItem(value_device_param, "device_type");
        cJSON *value_id_device = cJSON_GetObjectItem(value_device_param, "id_device");
        cJSON *value_devel_mode = cJSON_GetObjectItem(value_device_param, "Devel_Mode");
        cJSON *value_TrackArmMode = cJSON_GetObjectItem(value_device_param, "TrackArmMode");
        if (value_device_type != NULL && value_id_device != NULL && value_devel_mode != NULL && value_TrackArmMode != NULL)
        {
            device_type = value_device_type->valuestring;
            id_device = value_id_device->valuestring;
            devel_mode = value_devel_mode->valuestring;
            TrackArmMode = value_TrackArmMode->valuestring;
            cJSON_Delete(value_device_param);
            return true;
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Device param error!");
            cJSON_Delete(value_device_param);
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Device param error!");
        return false;
    }
}

void WSmsgs_Manager::NodePublisherInit()
{
    // WebSocket发送消息话题发布
    WSsend_msgs_pub = this->create_publisher<std_msgs::msg::String>("/WSmsgs_send_topic", 10);

    // 移动平台运动控制话题发布
    MotorCtrlNormalCmd_pub = this->create_publisher<robot_msgs::msg::MotorCtrlNormal>("/MotorCtrlNormal_topic", 10);
    MotorCtrltoPosCmd_pub = this->create_publisher<robot_msgs::msg::MotorCtrltoPos>("/MotorCtrltoPos_topic", 10);
}

void WSmsgs_Manager::NodeSubscriberInit()
{
    // WebSocket接收与发送消息话题
    WSreceive_msgs_sub = this->create_subscription<std_msgs::msg::String>(
        "WSmsgs_receive_topic", 10, std::bind(&WSmsgs_Manager::WSmsgsReceiveCallback, this, _1));

    // 心跳包话题订阅
    HeartbeatBag_sub = this->create_subscription<robot_msgs::msg::HeartbeatBag>(
        "HeartbeatBag_topic", 10, std::bind(&WSmsgs_Manager::HeartbeatBagCallback, this, _1));

    // 巡检任务计划开始执行信息上报话题订阅
    InspectTaskExecutionStart_sub = this->create_subscription<robot_msgs::msg::InspectTaskExecutionStart>(
        "InspectTaskExecutionStart_topic", 10, std::bind(&WSmsgs_Manager::InspectTaskExecutionStartCallback, this, _1));

    // 巡检任务计划执行完成信息上报话题订阅
    InspectTaskExecutionComplete_sub = this->create_subscription<robot_msgs::msg::InspectTaskExecutionComplete>(
        "InspectTaskExecutionComplete_topic", 10, std::bind(&WSmsgs_Manager::InspectTaskExecutionCompleteCallback, this, _1));
}

void WSmsgs_Manager::NodeServiceClientInit()
{
    // 切换控制模式客户端
    ChangeCtrlModeCmd_client = this->create_client<robot_msgs::srv::ChangeCtrlModeCmd>("ChangeCtrlModeCmd_service");

    // 控制模式查询客户端
    CtrlModeQuery_client = this->create_client<robot_msgs::srv::CtrlModeQuery>("CtrlModeQuery_service");

    // 机器人信息查询客户端
    RobotInfoQuery_client = this->create_client<robot_msgs::srv::RobotInfoQuery>("RobotInfoQuery_service");

    // 巡检任务列表查询客户端
    InspectTaskInfoListQuery_client = this->create_client<robot_msgs::srv::InspectTaskInfoListQuery>("InspectTaskInfoListQuery_service");

    // 单个巡检任务查询客户端
    InspectTaskInfoDetailQuery_client = this->create_client<robot_msgs::srv::InspectTaskInfoDetailQuery>("InspectTaskInfoDetailQuery_service");

    // 配置单个巡检任务客户端
    InspectTaskInfoConfigureCmd_client = this->create_client<robot_msgs::srv::InspectTaskInfoConfigureCmd>("InspectTaskInfoConfigureCmd_service");

    // 删除单个巡检任务客户端
    InspectTaskInfoDeleteCmd_client = this->create_client<robot_msgs::srv::InspectTaskInfoDeleteCmd>("InspectTaskInfoDeleteCmd_service");

    // 查询巡检任务计划列表客户端
    InspectTaskPlanListQuery_client = this->create_client<robot_msgs::srv::InspectTaskPlanListQuery>("InspectTaskPlanListQuery_service");

    // 添加巡检任务计划客户端
    InspectTaskPlanAddCmd_client = this->create_client<robot_msgs::srv::InspectTaskPlanAddCmd>("InspectTaskPlanAddCmd_service");

    // 删除巡检任务计划客户端
    InspectTaskPlanDeleteCmd_client = this->create_client<robot_msgs::srv::InspectTaskPlanDeleteCmd>("InspectTaskPlanDeleteCmd_service");

    // 巡检任务控制客户端
    InspectTaskCtrlCmd_client = this->create_client<robot_msgs::srv::InspectTaskCtrlCmd>("InspectTaskCtrlCmd_service");

    // 巡检任务实例查询客户端
    InspectTaskInstanceQuery_client = this->create_client<robot_msgs::srv::InspectTaskInstanceQuery>("InspectTaskInstanceQuery_service");

    // 巡检任务开始或结束响应客户端
    TaskStartOrFinishResponse_client = this->create_client<robot_msgs::srv::TaskStartOrFinishResponse>("TaskStartOrFinishResponse_service");

    // 设备时间校准客户端
    SystemTimeSyncCmd_client = this->create_client<robot_msgs::srv::SystemTimeSyncCmd>("SystemTimeSyncCmd_service");
}

void WSmsgs_Manager::NodeSpinnerStartup()
{
    executor_.add_node(shared_from_this());
    spinner_thread_ = std::thread([this]()
                                  { executor_.spin(); });
    spinner_thread_.join();
    // spinner_thread_ = std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
}

/***********************************************WS通信相关***********************************************/

void WSmsgs_Manager::WSmsgsReceiveCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (devel_mode == "debug")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] " << msg->data);
    }
    WSreceiveJsonParse(msg);
}

void WSmsgs_Manager::WSreceiveJsonParse(const std_msgs::msg::String::SharedPtr msg)
{
    cJSON *parsed_json = cJSON_Parse(msg->data.c_str());
    if (parsed_json != nullptr)
    {
        cJSON *value_json_fun = cJSON_GetObjectItem(parsed_json, "json_fun");
        if (value_json_fun != nullptr)
        {
            cJSON *value_type = cJSON_GetObjectItem(value_json_fun, "type");
            if (value_type != nullptr)
            {
                if (strcmp(value_type->valuestring, "server_cmd") == 0) // 服务端发送命令
                {
                    cJSON *value_cmd_data = cJSON_GetObjectItem(parsed_json, "cmd_data"); // 控制命令
                    if (value_cmd_data != NULL)
                    {
                        cJSON *value_function = cJSON_GetObjectItem(value_json_fun, "function");   // 功能
                        cJSON *value_id_device = cJSON_GetObjectItem(value_cmd_data, "id_device"); // 设备id
                        if (strcmp(value_id_device->valuestring, id_device.c_str()) == 0)
                        {
                            if (value_function != NULL)
                            {
                                if (strcmp(value_function->valuestring, "device") == 0)
                                {
                                    cJSON *value_sub_funtion = cJSON_GetObjectItem(value_json_fun, "sub_function"); // 子功能
                                    if (value_sub_funtion != NULL)
                                    {
                                        // 机器人控制相关
                                        if (strcmp(value_sub_funtion->valuestring, "control") == 0 || strcmp(value_sub_funtion->valuestring, "control_mode") == 0)
                                        {
                                            DeviceCtrlCmdJsonParse(value_json_fun, value_sub_funtion->valuestring, value_cmd_data);
                                            cJSON_Delete(parsed_json);
                                            return;
                                        }
                                        // 机器人信息查询
                                        else if (strcmp(value_sub_funtion->valuestring, "robot_info") == 0)
                                        {
                                            RobotInfoQueryProcess(value_json_fun);
                                            cJSON_Delete(parsed_json);
                                            return;
                                        }
                                        // 数据采集，待补充
                                        // 设备时间校准
                                        else if (strcmp(value_sub_funtion->valuestring, "system_time_synchronization") == 0)
                                        {
                                            SystemTimeSyncCmdProcess(value_json_fun, value_cmd_data);
                                            cJSON_Delete(parsed_json);
                                            return;
                                        }
                                        else
                                        {
                                            WrongParamProcess("Param Error: \"sub_function\" dose not have a correct param.", value_json_fun);
                                            cJSON_Delete(parsed_json);
                                            return;
                                        }
                                    }
                                    else
                                    {
                                        WrongParamProcess("Param Error: \"sub_function\" is NULL.", value_json_fun);
                                        cJSON_Delete(parsed_json);
                                        return;
                                    }
                                }
                                else if (strcmp(value_function->valuestring, "task") == 0)
                                {
                                    InspectTaskCmdJsonPares(value_json_fun, value_cmd_data);
                                    cJSON_Delete(parsed_json);
                                    return;
                                }
                                else
                                {
                                    WrongParamProcess("Param Error: \"function\" dose not have a correct param.", value_json_fun);
                                    cJSON_Delete(parsed_json);
                                    return;
                                }
                            }
                            else
                            {
                                WrongParamProcess("Param Error: \"function\" is NULL.", value_json_fun);
                                cJSON_Delete(parsed_json);
                                return;
                            }
                        }
                        else
                        {
                            WrongIdDeviceProcess(value_json_fun);
                            cJSON_Delete(parsed_json);
                            return;
                        }
                    }
                    else
                    {
                        WrongParamProcess("Param Error: \"cmd_data\" is NULL.", value_json_fun);
                        cJSON_Delete(parsed_json);
                        return;
                    }
                }
                else if (strcmp(value_type->valuestring, "server_rt") == 0) // 服务端返回消息
                {
                    cJSON *value_rt_data = cJSON_GetObjectItem(parsed_json, "rt_data");

                    if (value_rt_data != nullptr)
                    {
                        cJSON *value_function = cJSON_GetObjectItem(value_json_fun, "function");
                        cJSON *value_sub_function = cJSON_GetObjectItem(value_json_fun, "sub_function");
                        cJSON *value_id_device = cJSON_GetObjectItem(value_rt_data, "id_device");

                        if (value_id_device != nullptr)
                        {
                            if (strcmp(value_id_device->valuestring, id_device.c_str()) != 0)
                            {
                                WrongIdDeviceProcess(value_json_fun);
                                cJSON_Delete(value_json_fun);
                                return;
                            }
                            else
                            {
                                if (value_function != NULL && value_sub_function != NULL)
                                {
                                    if (strcmp(value_function->valuestring, "device") == 0)
                                    {
                                        if (strcmp(value_sub_function->valuestring, "monitor_data") == 0)
                                        {
                                            /* 数据采集，待补充 */
                                            cJSON_Delete(parsed_json);
                                            return;
                                        }
                                        else
                                        {
                                            WrongParamProcess("Param Error: unexpected sub_function value.", value_json_fun);
                                            cJSON_Delete(parsed_json);
                                            return;
                                        }
                                    }
                                    else if (strcmp(value_function->valuestring, "task") == 0)
                                    {
                                        if (strcmp(value_sub_function->valuestring, "task_execution") == 0)
                                        {
                                            InspectTaskStartOrFinishServerRtProcess(value_json_fun, value_rt_data);
                                            cJSON_Delete(parsed_json);
                                            return;
                                        }
                                        else
                                        {
                                            WrongParamProcess("Param Error: unexpected sub_function value.", value_json_fun);
                                            cJSON_Delete(parsed_json);
                                            return;
                                        }
                                    }
                                    else
                                    {
                                        WrongParamProcess("Param Error: unexpected function value.", value_json_fun);
                                        cJSON_Delete(parsed_json);
                                        return;
                                    }
                                }
                                else
                                {
                                    WrongParamProcess("Param Error: function or sub_function is NULL!", value_json_fun);
                                    cJSON_Delete(parsed_json);
                                    return;
                                }
                            }
                        }
                        else
                        {
                            WrongParamProcess("Param Error: id_device is NULL!", value_json_fun);
                            cJSON_Delete(parsed_json);
                            return;
                        }
                    }
                    else
                    {
                        WrongParamProcess("Param Error: rt_data is NULL!", value_json_fun);
                        cJSON_Delete(parsed_json);
                        return;
                    }
                }
                else
                {
                    WrongParamProcess("Param Error: \"type\" dose not have a correct param.", value_json_fun);
                    cJSON_Delete(parsed_json);
                    return;
                }
            }
            else
            {
                WrongParamProcess("Param Error: \"type\" is NULL.", value_json_fun);
                cJSON_Delete(parsed_json);
                return;
            }
        }
        else
        {
            cJSON_Delete(parsed_json);
            return;
        }
    }
    else
        return;
}

void WSmsgs_Manager::WSsendJsonBack(const cJSON *json_fun, const cJSON *rt_info, const cJSON *rt_data)
{
    cJSON *WSjson_SendBack = cJSON_CreateObject();
    cJSON_AddItemReferenceToObject(WSjson_SendBack, "json_fun", (cJSON *)json_fun);
    cJSON_AddItemReferenceToObject(WSjson_SendBack, "rt_info", (cJSON *)rt_info);
    cJSON_AddItemReferenceToObject(WSjson_SendBack, "rt_data", (cJSON *)rt_data);

    auto WSjson_SendBack_str = std_msgs::msg::String();
    char *temp_str = cJSON_Print(WSjson_SendBack);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] WSsendJsonBack(): cJSON_Print() error!");
    }
    else
    {
        WSjson_SendBack_str.data = temp_str;
        WSsend_msgs_pub->publish(WSjson_SendBack_str);
        free(temp_str);
        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] WebSocket sent msgs: " << WSjson_SendBack_str.data);
    }

    cJSON_Delete(WSjson_SendBack);
    return;
}

void WSmsgs_Manager::WSsendJsonCmd(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *WSjson_SendCmd = cJSON_CreateObject();
    cJSON_AddItemReferenceToObject(WSjson_SendCmd, "json_fun", (cJSON *)json_fun);
    cJSON_AddItemReferenceToObject(WSjson_SendCmd, "cmd_data", (cJSON *)cmd_data);

    auto WSjson_Sendcmd_str = std_msgs::msg::String();
    char *temp_str = cJSON_Print(WSjson_SendCmd);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] WSsendJsoncmd(): cJSON_Print() error!");
    }
    else
    {
        WSjson_Sendcmd_str.data = temp_str;
        WSsend_msgs_pub->publish(WSjson_Sendcmd_str);
        free(temp_str);
        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] WebSocket sent msgs: " << WSjson_Sendcmd_str.data);
    }

    cJSON_Delete(WSjson_SendCmd);
    return;
}

void WSmsgs_Manager::WSsendJsonUpload(const cJSON *json_fun, const cJSON *up_data)
{
    cJSON *WSjson_SendUpload = cJSON_CreateObject();
    cJSON_AddItemReferenceToObject(WSjson_SendUpload, "json_fun", (cJSON *)json_fun);
    cJSON_AddItemReferenceToObject(WSjson_SendUpload, "up_data", (cJSON *)up_data);

    std_msgs::msg::String WSjson_SendUpload_str;
    char *temp_str = cJSON_Print(WSjson_SendUpload);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] WSsendJsonUpload(): cJSON_Print() error!");
    }
    else
    {
        WSjson_SendUpload_str.data = temp_str;
        WSsend_msgs_pub->publish(WSjson_SendUpload_str);
        free(temp_str);
        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] WebSocket sent msgs: " << WSjson_SendUpload_str.data);
    }

    cJSON_Delete(WSjson_SendUpload);
    return;
}

void WSmsgs_Manager::WSsendJsonHeartbeatBag(const cJSON *json_fun, const cJSON *heartbeat_bag)
{
    cJSON *WSjson_HeartbeatBag = cJSON_CreateObject();
    cJSON_AddItemReferenceToObject(WSjson_HeartbeatBag, "json_fun", (cJSON *)json_fun);
    cJSON_AddItemReferenceToObject(WSjson_HeartbeatBag, "heartbeat_bag", (cJSON *)heartbeat_bag);

    auto WSjson_HeartbeatBag_str = std_msgs::msg::String();
    char *temp_str = cJSON_Print(WSjson_HeartbeatBag);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] WSsendJsonHeartbeatBag(): cJSON_Print() error!");
    }
    else
    {
        WSjson_HeartbeatBag_str.data = temp_str;
        WSsend_msgs_pub->publish(WSjson_HeartbeatBag_str);
        free(temp_str);
        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] WebSocket sent msgs: " << WSjson_HeartbeatBag_str.data);
    }

    cJSON_Delete(WSjson_HeartbeatBag);
    return;
}

/***********************************************错误处理相关***********************************************/

void WSmsgs_Manager::WrongParamProcess(const char *err_msg, const cJSON *json_fun) // 错误id：1002 参数错误
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }
    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1002);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", err_msg);

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::DeviceInternalCommunicationErrorProcess(const cJSON *json_fun) // 错误id：1003 设备内部通信错误
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }
    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1003);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Device internal communication error");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::RequestResourceNotExistProcess(const cJSON *json_fun) // 错误id：1005 请求的资源不存在
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }
    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1005);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "The request resource not exist");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::WrongIdDeviceProcess(const cJSON *json_fun) // 错误id：1007 请求设备id错误
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }
    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1007);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Device id error");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::DeviceOperationFailedProcess(const cJSON *json_fun) // 错误id：2201 设备操作失败
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }
    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 2201);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Device operation failed");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::DeviceNotSupportProcess(const cJSON *json_fun) // 错误id：2202 设备不支持该操作
{
    cJSON *json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == nullptr)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 2202);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Device is not supported");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::DeviceModeErrorProcess(const cJSON *json_fun) // 错误id：2203 设备控制模式错误
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }
    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 2203);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Device mode error");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::DeleteTaskFromDeviceFailProcess(const cJSON *json_fun) // 错误id：2601 从设备删除任务失败
{
    cJSON *json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == nullptr)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 2601);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Delete task from device failed");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::SendTask2DeviceFailProcess(const cJSON *json_fun) // 错误id：2602 任务下发失败
{
    cJSON *json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 2602);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Send task to device failed");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::TaskNotExistProcess(const cJSON *json_fun) // 错误id：2603 任务不存在
{
    cJSON *json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 2603);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Task not exist");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::TaskInstanceNotExistProcess(const cJSON *json_fun) // 错误id：2604 任务实例不存在
{
    cJSON *json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 2604);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Task instance not exist");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::TaskNameExistProcess(const cJSON *json_fun) // 错误id：2605 任务名称已存在
{
    cJSON *json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 2605);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Task name already exist");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::TaskPlanNotExistProcess(const cJSON *json_fun) // 错误id：2606 任务计划不存在
{
    cJSON *json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 2606);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Task plan not exist");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

/***********************************************机器人控制相关***********************************************/

void WSmsgs_Manager::DeviceCtrlCmdJsonParse(const cJSON *json_fun, const char *sub_function, const cJSON *cmd_data)
{
    // 切换控制模式
    if (strcmp(sub_function, "control_mode") == 0)
    {
        cJSON *value_ctrl_mode = cJSON_GetObjectItem(cmd_data, "mode");
        if (value_ctrl_mode != NULL && cJSON_IsNumber(value_ctrl_mode))
        {
            ChangeCtrlModeCmdProcess(json_fun, value_ctrl_mode->valueint);
            return;
        }
        else
        {
            WrongParamProcess("Param Error: \"mode\" is wrong", json_fun);
            return;
        }
    }

    // 后台手动遥控
    else if (strcmp(sub_function, "control") == 0)
    {
        robot_msgs::srv::CtrlModeQuery::Request::SharedPtr request;
        while (!CtrlModeQuery_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "CtrlModeQuery Service not available, waiting again...");
        }

        cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
        cJSON *cmd_data_copy = cJSON_Duplicate(cmd_data, 1);

        if (json_fun_copy == NULL || cmd_data_copy == NULL)
        {
            RCLCPP_ERROR(this->get_logger(), "copy is null");
            return;
        }

        auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::CtrlModeQuery>::SharedFuture>();

        auto res_callback = [this, json_fun_copy, cmd_data_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::CtrlModeQuery>::SharedFuture future)
        {
            *shared_future_ptr = future;
            auto response = future.get();
            if (response->runtime_ctrl_mode == 1)
            {
                cJSON *value_ctrl_type = cJSON_GetObjectItem(cmd_data_copy, "ctrl_type");
                cJSON *value_ctrl_value = cJSON_GetObjectItem(cmd_data_copy, "ctrl_value");

                if (value_ctrl_type != NULL && value_ctrl_value != NULL)
                {
                    // 移动平台运动控制
                    if (strcmp(value_ctrl_type->valuestring, "e_ctrl_motor") == 0)
                    {
                        MotorCtrlCmdProcess(json_fun_copy, value_ctrl_value);
                        return;
                    }

                    /*else if其他设备控制，待补充*/
                    else
                    {
                        WrongParamProcess("Param Error: \"ctrl_type\" does not have a correct param.", json_fun_copy);
                        return;
                    }
                }
                else
                {
                    WrongParamProcess("Param Error: \"ctrl_type\" or \"ctrl_value\" is NULL.", json_fun_copy);
                    return;
                }
            }
            else
            {
                DeviceModeErrorProcess(json_fun_copy);
                return;
            }
        };

        auto future = CtrlModeQuery_client->async_send_request(request, res_callback);
        *shared_future_ptr = future.future;
        auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                             [this, json_fun_copy, shared_future_ptr]()
                                             {
                                                 if (shared_future_ptr->valid() &&
                                                     shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                 {
                                                     DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                     return;
                                                 }
                                             });
    }

    /*else if其他指令，待补充*/

    else
        return;
}

void WSmsgs_Manager::ChangeCtrlModeCmdProcess(const cJSON *json_fun, const uint8_t target_ctrl_mode)
{
    if (target_ctrl_mode == 0 || target_ctrl_mode == 1)
    {
        robot_msgs::srv::ChangeCtrlModeCmd::Request::SharedPtr request;
        request->ctrl_mode = target_ctrl_mode;
        while (!ChangeCtrlModeCmd_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "ChangeCtrlModeCmd Service not available, waiting again...");
        }

        cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
        if (json_fun_copy == NULL)
        {
            RCLCPP_ERROR(this->get_logger(), "copy is null");
            return;
        }

        auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ChangeCtrlModeCmd>::SharedFuture>();

        auto res_callback = [this, json_fun_copy, target_ctrl_mode, shared_future_ptr](rclcpp::Client<robot_msgs::srv::ChangeCtrlModeCmd>::SharedFuture future)
        {
            *shared_future_ptr = future;
            auto response = future.get();
            if (response->execute_success)
            {
                ChangeCtrlModeCmdSendback(json_fun_copy, target_ctrl_mode);
                RCLCPP_INFO(this->get_logger(), "res->succuss is true");
                return;
            }
            else
            {
                DeviceOperationFailedProcess(json_fun_copy);
                return;
            }
        };

        auto future = ChangeCtrlModeCmd_client->async_send_request(request, res_callback);
        *shared_future_ptr = future.future;
        auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                             [this, json_fun_copy, shared_future_ptr]()
                                             {
                                                 if (shared_future_ptr->valid() &&
                                                     shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                 {
                                                     DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                     return;
                                                 }
                                             });
    }
    else
        return;
}

void WSmsgs_Manager::ChangeCtrlModeCmdSendback(const cJSON *json_fun, const uint8_t target_ctrl_mode)
{

    if (json_fun == NULL)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }

    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == NULL)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }

    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON_AddNumberToObject(rt_data_SendBack, "mode", target_ctrl_mode);

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::MotorCtrlCmdProcess(const cJSON *json_fun, const cJSON *ctrl_value)
{
    cJSON *value_action = cJSON_GetObjectItem(ctrl_value, "action");
    cJSON *value_speed = cJSON_GetObjectItem(ctrl_value, "speed");

    if (value_action != nullptr)
    {
        float speed = -10; // -10表示服务器发送指令中speed为空，使用机器人上设定的默认值
        if (value_speed != nullptr)
        {
            speed = value_speed->valuedouble * 0.01; // speed单位m/s，value_speed单位cm/s
        }
        else
        {
            speed = 0.5; // 默认速度0.5m/s
        }

        if (strcmp(value_action->valuestring, "e_move_forward") == 0)
        {
            robot_msgs::msg::MotorCtrlNormal MotorCtrlNormalMsg;
            MotorCtrlNormalMsg.command = "move_forward";
            MotorCtrlNormalMsg.run_speed = speed;
            MotorCtrlNormalCmd_pub->publish(MotorCtrlNormalMsg);
            DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
            return;
        }
        else if (strcmp(value_action->valuestring, "e_move_back") == 0)
        {
            robot_msgs::msg::MotorCtrlNormal MotorCtrlNormalMsg;
            MotorCtrlNormalMsg.command = "move_back";
            MotorCtrlNormalMsg.run_speed = speed;
            MotorCtrlNormalCmd_pub->publish(MotorCtrlNormalMsg);
            DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
            return;
        }
        else if (strcmp(value_action->valuestring, "e_turn_left") == 0)
        {
            robot_msgs::msg::MotorCtrlNormal MotorCtrlNormalMsg;
            MotorCtrlNormalMsg.command = "turn_left";
            MotorCtrlNormalMsg.run_speed = speed;
            MotorCtrlNormalCmd_pub->publish(MotorCtrlNormalMsg);
            DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
            return;
        }
        else if (strcmp(value_action->valuestring, "e_turn_right") == 0)
        {
            robot_msgs::msg::MotorCtrlNormal MotorCtrlNormalMsg;
            MotorCtrlNormalMsg.command = "turn_right";
            MotorCtrlNormalMsg.run_speed = speed;
            MotorCtrlNormalCmd_pub->publish(MotorCtrlNormalMsg);
            DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
            return;
        }
        else if (strcmp(value_action->valuestring, "e_move_stop") == 0)
        {
            robot_msgs::msg::MotorCtrlNormal MotorCtrlNormalMsg;
            MotorCtrlNormalMsg.command = "move_stop";
            MotorCtrlNormalMsg.run_speed = 0;
            MotorCtrlNormalCmd_pub->publish(MotorCtrlNormalMsg);
            DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
            return;
        }
        else if (strcmp(value_action->valuestring, "e_runto_position") == 0)
        {
            cJSON *value_position = cJSON_GetObjectItem(ctrl_value, "position");
            if (value_position != nullptr)
            {
                cJSON *value_id_track = cJSON_GetObjectItem(value_position, "id_track");
                cJSON *value_track_pos = cJSON_GetObjectItem(value_position, "track_pos");

                if (value_id_track != nullptr && value_track_pos != nullptr)
                {
                    robot_msgs::msg::MotorCtrltoPos MotorCtrltoPosMsg;
                    MotorCtrltoPosMsg.command = "runto_position";
                    MotorCtrltoPosMsg.track_id = value_id_track->valuestring;
                    MotorCtrltoPosMsg.track_pos = value_track_pos->valueint;
                    MotorCtrltoPosMsg.run_speed = speed;
                    MotorCtrltoPosCmd_pub->publish(MotorCtrltoPosMsg);
                    DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
                    return;
                }
            }
            else
            {
                WrongParamProcess("Param Error: \"position\" does not have a correct param.", json_fun);
            }
        }
        else if (strcmp(value_action->valuestring, "e_go_home") == 0)
        {
            robot_msgs::msg::MotorCtrltoPos MotorCtrltoPosMsg;
            MotorCtrltoPosMsg.command = "go_home";
            MotorCtrltoPosMsg.track_id = "home";
            MotorCtrltoPosMsg.track_pos = 0;
            MotorCtrltoPosMsg.run_speed = speed;
            MotorCtrltoPosCmd_pub->publish(MotorCtrltoPosMsg);
            DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
        }
        else
        {
            WrongParamProcess("Param Error: \"action\" is NULL.", json_fun);
            return;
        }
    }
    else
    {
        WrongParamProcess("Param Error: \"ctrl_value\" is NULL.", json_fun);
        return;
    }
}

void WSmsgs_Manager::DeviceCtrlCmdSendback(const cJSON *json_fun, const char *ctrl_type)
{
    if (json_fun == NULL)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }
    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == NULL)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "ctrl_type", ctrl_type);

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

/***********************************************巡检任务查询***********************************************/
void WSmsgs_Manager::InspectTaskCmdJsonPares(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_sub_function = cJSON_GetObjectItem(json_fun, "sub_function");

    if (value_sub_function != nullptr)
    {
        if (strcmp(value_sub_function->valuestring, "task_info") == 0)
        {
            InspectTaskInfoCmdProcess(json_fun, cmd_data);
            return;
        }
        else if (strcmp(value_sub_function->valuestring, "task_plan") == 0)
        {
            InspectTaskPlanCmdProcess(json_fun, cmd_data);
            return;
        }
        else if (strcmp(value_sub_function->valuestring, "control") == 0)
        {
            InspectTaskCtrlCmdProcess(json_fun, cmd_data);
            return;
        }
        else if (strcmp(value_sub_function->valuestring, "instance") == 0)
        {
            InspectTaskInstanceCmdProcess(json_fun, cmd_data);
            return;
        }
        else
        {
            WrongParamProcess("Param Error: unexpected task_sub_function value.", json_fun);
            return;
        }
    }
    else
    {
        WrongParamProcess("Param Error: task_sub_function is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskInfoCmdProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_sub_sub_function = cJSON_GetObjectItem(json_fun, "sub_sub_function");

    if (value_sub_sub_function != nullptr)
    {
        if (strcmp(value_sub_sub_function->valuestring, "list") == 0)
        {
            InspectTaskInfoListQueryProcess(json_fun, cmd_data);
            return;
        }
        else if (strcmp(value_sub_sub_function->valuestring, "query") == 0)
        {
            InspectTaskInfoDetailQueryProcess(json_fun, cmd_data);
            return;
        }
        else if (strcmp(value_sub_sub_function->valuestring, "configuration") == 0)
        {
            InspectTaskInfoConfigureCmdProcess(json_fun, cmd_data);
            return;
        }
        else if (strcmp(value_sub_sub_function->valuestring, "deletion") == 0)
        {
            InspectTaskInfoDeleteCmdProcess(json_fun, cmd_data);
            return;
        }
        else
        {
            WrongParamProcess("Param Error: unexpected task_sub_sub_function value.", json_fun);
            return;
        }
    }
    else
    {
        WrongParamProcess("Param Error: task_sub_sub_function is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskInfoListQueryProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_task_type = cJSON_GetObjectItem(cmd_data, "task_type");
    if (value_task_type != nullptr)
    {
        if (cJSON_IsArray(value_task_type))
        {
            size_t array_size = cJSON_GetArraySize(value_task_type);
            if (array_size > 0)
            {
                while (!InspectTaskInfoListQuery_client->wait_for_service(std::chrono::seconds(1)))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "InspectTaskInfoListQuery Service not available, waiting again...");
                }
                robot_msgs::srv::InspectTaskInfoListQuery::Request::SharedPtr request;

                for (size_t i = 0; i < array_size; i++)
                {
                    cJSON *array_item = cJSON_GetArrayItem(value_task_type, i);
                    if (array_item != nullptr)
                    {
                        if (strcmp(array_item->valuestring, "e_inspect_all") == 0 ||
                            strcmp(array_item->valuestring, "e_inspect_routine") == 0 ||
                            strcmp(array_item->valuestring, "e_inspect_special") == 0 ||
                            strcmp(array_item->valuestring, "e_inspect_directional") == 0)
                        {
                            std_msgs::msg::String data_temp;
                            data_temp.data = array_item->valuestring;
                            request->task_type.push_back(data_temp.data);
                        }
                        else
                        {
                            WrongParamProcess("Param Error: task_type's array_item error.", json_fun);
                            return;
                        }
                    }
                    else
                    {
                        DeviceOperationFailedProcess(json_fun);
                        return;
                    }
                }

                cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
                if (json_fun_copy == nullptr)
                {
                    RCLCPP_ERROR(this->get_logger(), "copy is null");
                    return;
                }

                auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::InspectTaskInfoListQuery>::SharedFuture>();
                auto res_callback = [this, json_fun_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::InspectTaskInfoListQuery>::SharedFuture future)
                {
                    *shared_future_ptr = future;
                    auto response = future.get();
                    InspectTaskInfoListQuerySendback(json_fun_copy, response);
                    RCLCPP_INFO(this->get_logger(), "InspectTaskInfoListQuery successful");
                    return;
                };
                auto future = InspectTaskInfoListQuery_client->async_send_request(request, res_callback);
                *shared_future_ptr = future.future;

                auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                     [this, json_fun_copy, shared_future_ptr]()
                                                     {
                                                         if (shared_future_ptr->valid() &&
                                                             shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                         {
                                                             DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                             return;
                                                         }
                                                     });
            }
            else
            {
                WrongParamProcess("Param Error: task_type's array_size <= 0.", json_fun);
                return;
            }
        }
        else
        {
            WrongParamProcess("Param Error: task_type is not an array.", json_fun);
            return;
        }
    }
    else
    {
        WrongParamProcess("Param Error: task_type is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskInfoListQuerySendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskInfoListQuery::Response::SharedPtr response)
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }

    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }

    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON *task_info_list_array = cJSON_CreateArray();
    size_t array_size = response->inspect_task_info_simple_list.size();
    std::vector<cJSON *> task_info_array(array_size);
    for (size_t i = 0; i < array_size; i++)
    {
        task_info_array[i] = cJSON_CreateObject();
        cJSON_AddStringToObject(task_info_array[i], "id_task_info", response->inspect_task_info_simple_list[i].id_task_info.c_str());
        cJSON_AddStringToObject(task_info_array[i], "task_name", response->inspect_task_info_simple_list[i].task_name.c_str());
        cJSON_AddStringToObject(task_info_array[i], "task_type", response->inspect_task_info_simple_list[i].task_type.c_str());
        cJSON_AddStringToObject(task_info_array[i], "create_time", response->inspect_task_info_simple_list[i].create_time.c_str());
    }
    cJSON_AddItemReferenceToObject(rt_data_SendBack, "list", task_info_list_array);

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    cJSON_Delete(task_info_list_array);
    for (size_t i = 0; i < array_size; i++)
    {
        cJSON_Delete(task_info_array[i]);
    }
    return;
}

void WSmsgs_Manager::InspectTaskInfoDetailQueryProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_id_task_info = cJSON_GetObjectItem(cmd_data, "id_task_info");
    if (value_id_task_info != nullptr)
    {
        while (!InspectTaskInfoDetailQuery_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "InspectTaskInfoDetailQuery Service not available, waiting again...");
        }

        robot_msgs::srv::InspectTaskInfoDetailQuery::Request::SharedPtr request;
        request->id_task_info = value_id_task_info->valuestring;

        cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
        if (json_fun_copy == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "copy is null");
            return;
        }

        auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::InspectTaskInfoDetailQuery>::SharedFuture>();
        auto res_callback = [this, json_fun_copy, request, shared_future_ptr](rclcpp::Client<robot_msgs::srv::InspectTaskInfoDetailQuery>::SharedFuture future)
        {
            *shared_future_ptr = future;
            auto response = future.get();
            if (response->task_not_exist)
            {
                TaskNotExistProcess(json_fun_copy);
                return;
            }
            else
            {
                InspectTaskInfoDetailQuerySendback(json_fun_copy, response, request);
                RCLCPP_INFO(this->get_logger(), "InspectTaskInfoDetailQuery successful");
                return;
            }
        };
        auto future = InspectTaskInfoDetailQuery_client->async_send_request(request, res_callback);
        *shared_future_ptr = future.future;

        auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                             [this, json_fun_copy, shared_future_ptr]()
                                             {
                                                 if (shared_future_ptr->valid() &&
                                                     shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                 {
                                                     DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                     return;
                                                 }
                                             });
    }
    else
    {
        WrongParamProcess("Param Error: id_task_info is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskInfoDetailQuerySendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskInfoDetailQuery::Response::SharedPtr response,
                                                        const robot_msgs::srv::InspectTaskInfoDetailQuery::Request::SharedPtr request)
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }

    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }

    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "id_task_info", request->id_task_info.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "task_name", response->task_name.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "task_type", response->task_type.c_str());
    cJSON *monitor_points_array = cJSON_CreateArray();
    size_t array_size = response->monitor_points.size();
    for (size_t i = 0; i < array_size; i++)
    {
        cJSON_AddItemToArray(monitor_points_array, cJSON_CreateString(response->monitor_points[i].c_str()));
    }
    cJSON_AddItemReferenceToObject(rt_data_SendBack, "monitor_points", monitor_points_array);
    cJSON_AddStringToObject(rt_data_SendBack, "create_time", response->create_time.c_str());

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    cJSON_Delete(monitor_points_array);
    return;
}

void WSmsgs_Manager::InspectTaskInfoConfigureCmdProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_id_task_info = cJSON_GetObjectItem(cmd_data, "id_task_info");
    cJSON *value_task_name = cJSON_GetObjectItem(cmd_data, "task_name");
    cJSON *value_task_type = cJSON_GetObjectItem(cmd_data, "task_type");
    cJSON *value_monitor_points = cJSON_GetObjectItem(cmd_data, "monitor_points");
    cJSON *value_create_time = cJSON_GetObjectItem(cmd_data, "create_time");
    if (value_task_name != nullptr &&
        value_task_type != nullptr &&
        value_monitor_points != nullptr &&
        value_create_time != nullptr)
    {
        if (cJSON_IsArray(value_monitor_points))
        {
            while (!InspectTaskInfoConfigureCmd_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "InspectTaskInfoConfigureCmd Service not available, waiting again...");
            }

            robot_msgs::srv::InspectTaskInfoConfigureCmd::Request::SharedPtr request;
            if (value_id_task_info == nullptr)
            {
                request->action = "create";
                request->id_task_info = "";
            }
            else
            {
                request->action = "configure";
                request->id_task_info = value_id_task_info->valuestring;
            }
            request->task_name = value_task_name->valuestring;
            request->task_type = value_task_type->valuestring;
            request->create_time = value_create_time->valuestring;
            size_t array_size = cJSON_GetArraySize(value_monitor_points);
            for (size_t i = 0; i < array_size; i++)
            {
                cJSON *array_item = cJSON_GetArrayItem(value_monitor_points, i);
                std_msgs::msg::String data_temp;
                data_temp.data = array_item->valuestring;
                request->monitor_points.push_back(data_temp.data);
            }

            cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
            if (json_fun_copy == nullptr)
            {
                RCLCPP_ERROR(this->get_logger(), "copy is null");
                return;
            }

            auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::InspectTaskInfoConfigureCmd>::SharedFuture>();
            auto res_callback = [this, json_fun_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::InspectTaskInfoConfigureCmd>::SharedFuture future)
            {
                *shared_future_ptr = future;
                auto response = future.get();
                if (response->task_not_exist)
                {
                    TaskNotExistProcess(json_fun_copy);
                    return;
                }
                else
                {
                    if (response->name_used)
                    {
                        TaskNameExistProcess(json_fun_copy);
                        return;
                    }
                    else
                    {
                        InspectTaskInfoConfigureCmdSendback(json_fun_copy, response->id_task_info.c_str());
                        RCLCPP_INFO(this->get_logger(), "InspectTaskInfoConfigureCmd successful");
                        return;
                    }
                }
            };

            auto future = InspectTaskInfoConfigureCmd_client->async_send_request(request, res_callback);
            *shared_future_ptr = future.future;

            auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                 [this, json_fun_copy, shared_future_ptr]()
                                                 {
                                                     if (shared_future_ptr->valid() &&
                                                         shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                     {
                                                         DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                         return;
                                                     }
                                                 });
        }
        else
        {
            WrongParamProcess("Param Error: monitor_points is not an array.", json_fun);
            return;
        }
    }
    else
    {
        WrongParamProcess("Param Error: task_name or task_type or monitor_points or create_time is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskInfoConfigureCmdSendback(const cJSON *json_fun, const char *id_task_info)
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }

    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "id_task_info", id_task_info);

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::InspectTaskInfoDeleteCmdProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_id_task_info = cJSON_GetObjectItem(cmd_data, "id_task_info");
    if (value_id_task_info != nullptr)
    {
        while (!InspectTaskInfoDeleteCmd_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "InspectTaskInfoDeleteCmd Service not available, waiting again...");
        }

        robot_msgs::srv::InspectTaskInfoDeleteCmd::Request::SharedPtr request;
        request->id_task_info = value_id_task_info->valuestring;

        cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
        if (json_fun_copy == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "copy is null");
            return;
        }

        auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::InspectTaskInfoDeleteCmd>::SharedFuture>();
        auto res_callback = [this, json_fun_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::InspectTaskInfoDeleteCmd>::SharedFuture future)
        {
            *shared_future_ptr = future;
            auto response = future.get();
            if (response->task_not_exist)
            {
                TaskNotExistProcess(json_fun_copy);
                return;
            }
            else
            {
                if (response->deletion_success)
                {
                    InspectTaskInfoDeleteCmdSendback(json_fun_copy);
                    RCLCPP_INFO(this->get_logger(), "InspectTaskInfoDeleteCmd successful");
                    return;
                }
                else
                {
                    DeleteTaskFromDeviceFailProcess(json_fun_copy);
                    return;
                }
            }
        };
        auto future = InspectTaskInfoDeleteCmd_client->async_send_request(request, res_callback);
        *shared_future_ptr = future.future;

        auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                             [this, json_fun_copy, shared_future_ptr]()
                                             {
                                                 if (shared_future_ptr->valid() &&
                                                     shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                 {
                                                     DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                     return;
                                                 }
                                             });
    }
    else
    {
        WrongParamProcess("Param Error: id_task_info is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskInfoDeleteCmdSendback(const cJSON *json_fun)
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }

    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::InspectTaskPlanCmdProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_sub_sub_function = cJSON_GetObjectItem(json_fun, "sub_sub_function");

    if (value_sub_sub_function != nullptr)
    {
        if (strcmp(value_sub_sub_function->valuestring, "list") == 0)
        {
            InspectTaskPlanListQueryProcess(json_fun, cmd_data);
            return;
        }
        else if (strcmp(value_sub_sub_function->valuestring, "addition") == 0)
        {
            InspectTaskPlanAddCmdProcess(json_fun, cmd_data);
            return;
        }
        else if (strcmp(value_sub_sub_function->valuestring, "deletion") == 0)
        {
            InspectTaskPlanDeleteCmdProcess(json_fun, cmd_data);
            return;
        }
        else
        {
            WrongParamProcess("Param Error: unexpected sub_sub_function value.", json_fun);
            return;
        }
    }
    else
    {
        WrongParamProcess("Param Error: sub_sub_function is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskPlanListQueryProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_id_task_info = cJSON_GetObjectItem(cmd_data, "id_task_info");
    if (value_id_task_info != nullptr)
    {
        while (!InspectTaskPlanListQuery_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "InspectTaskPlanListQuery Service not available, waiting again...");
        }

        robot_msgs::srv::InspectTaskPlanListQuery::Request::SharedPtr request;
        request->id_task_info = value_id_task_info->valuestring;

        cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
        if (json_fun_copy == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "copy is null");
            return;
        }

        auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::InspectTaskPlanListQuery>::SharedFuture>();
        auto res_callback = [this, json_fun_copy, request, shared_future_ptr](rclcpp::Client<robot_msgs::srv::InspectTaskPlanListQuery>::SharedFuture future)
        {
            *shared_future_ptr = future;
            auto response = future.get();
            if (response->task_not_exist)
            {
                TaskNotExistProcess(json_fun_copy);
                return;
            }
            else
            {
                InspectTaskPlanListQuerySendback(json_fun_copy, response, request);
                RCLCPP_INFO(this->get_logger(), "InspectTaskPlanListQuery successful");
                return;
            }
        };
        auto future = InspectTaskPlanListQuery_client->async_send_request(request, res_callback);
        *shared_future_ptr = future.future;

        auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                             [this, json_fun_copy, shared_future_ptr]()
                                             {
                                                 if (shared_future_ptr->valid() &&
                                                     shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                 {
                                                     DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                     return;
                                                 }
                                             });
    }
    else
    {
        WrongParamProcess("Param Error: id_task_info is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskPlanListQuerySendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskPlanListQuery::Response::SharedPtr response,
                                                      const robot_msgs::srv::InspectTaskPlanListQuery::Request::SharedPtr request)
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }

    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "id_task_info", request->id_task_info.c_str());
    cJSON *task_plan_list_array = cJSON_CreateArray();
    size_t plan_array_size = response->inspect_task_plan_list.size();
    std::vector<cJSON *> task_plan_array(plan_array_size);
    std::vector<cJSON *> express_array(plan_array_size);
    for (size_t i = 0; i < plan_array_size; i++)
    {
        task_plan_array[i] = cJSON_CreateObject();
        cJSON_AddStringToObject(task_plan_array[i], "id_task_plan", response->inspect_task_plan_list[i].id_task_plan.c_str());
        cJSON_AddStringToObject(task_plan_array[i], "plan_type", response->inspect_task_plan_list[i].plan_type.c_str());
        express_array[i] = cJSON_CreateArray();
        size_t points_array_size = response->inspect_task_plan_list[i].express.size();
        for (size_t j = 0; j < points_array_size; j++)
        {
            cJSON_AddItemToArray(express_array[i], cJSON_CreateString(response->inspect_task_plan_list[i].express[j].c_str()));
        }
        cJSON_AddItemReferenceToObject(task_plan_array[i], "express", express_array[i]);
        cJSON_AddStringToObject(task_plan_array[i], "create_time", response->inspect_task_plan_list[i].create_time.c_str());
        cJSON_AddItemReferenceToArray(task_plan_list_array, task_plan_array[i]);
    }
    cJSON_AddItemReferenceToObject(rt_data_SendBack, "list", task_plan_list_array);

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    cJSON_Delete(task_plan_list_array);
    for (size_t i = 0; i < plan_array_size; i++)
    {
        cJSON_Delete(task_plan_array[i]);
        cJSON_Delete(express_array[i]);
    }
    return;
}

void WSmsgs_Manager::InspectTaskPlanAddCmdProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_id_task_info = cJSON_GetObjectItem(cmd_data, "id_task_info");
    cJSON *value_plan_type = cJSON_GetObjectItem(cmd_data, "plan_type");
    cJSON *value_express = cJSON_GetObjectItem(cmd_data, "express");
    cJSON *value_create_time = cJSON_GetObjectItem(cmd_data, "create_time");
    if (value_id_task_info != nullptr &&
        value_plan_type != nullptr &&
        value_express != nullptr &&
        value_create_time != nullptr)
    {
        if (cJSON_IsArray(value_express))
        {
            while (!InspectTaskPlanAddCmd_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "InspectTaskPlanAddCmd Service not available, waiting again...");
            }

            robot_msgs::srv::InspectTaskPlanAddCmd::Request::SharedPtr request;
            request->id_task_info = value_id_task_info->valuestring;
            request->plan_type = value_plan_type->valuestring;
            request->create_time = value_create_time->valuestring;
            size_t array_size = cJSON_GetArraySize(value_express);
            for (size_t i = 0; i < array_size; i++)
            {
                cJSON *array_item = cJSON_GetArrayItem(value_express, i);
                std_msgs::msg::String data_temp;
                data_temp.data = array_item->valuestring;
                request->express.push_back(data_temp.data);
            }

            cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
            if (json_fun_copy == nullptr)
            {
                RCLCPP_ERROR(this->get_logger(), "copy is null");
                return;
            }

            auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::InspectTaskPlanAddCmd>::SharedFuture>();
            auto res_callback = [this, json_fun_copy, request, shared_future_ptr](rclcpp::Client<robot_msgs::srv::InspectTaskPlanAddCmd>::SharedFuture future)
            {
                *shared_future_ptr = future;
                auto response = future.get();
                if (response->task_not_exist)
                {
                    TaskNotExistProcess(json_fun_copy);
                    return;
                }
                else
                {
                    if (response->create_success)
                    {
                        InspectTaskPlanAddCmdSendback(json_fun_copy, response, request);
                        RCLCPP_INFO(this->get_logger(), "InspectTaskPlanAddCmd successful");
                        return;
                    }
                    else
                    {
                        SendTask2DeviceFailProcess(json_fun_copy);
                        return;
                    }
                }
            };

            auto future = InspectTaskPlanAddCmd_client->async_send_request(request, res_callback);
            *shared_future_ptr = future.future;

            auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                 [this, json_fun_copy, shared_future_ptr]()
                                                 {
                                                     if (shared_future_ptr->valid() &&
                                                         shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                     {
                                                         DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                         return;
                                                     }
                                                 });
        }
        else
        {
            WrongParamProcess("Param Error: express is not an array.", json_fun);
            return;
        }
    }
    else
    {
        WrongParamProcess("Param Error: id_task_info or plan_type or express or create_time is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskPlanAddCmdSendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskPlanAddCmd::Response::SharedPtr response,
                                                   const robot_msgs::srv::InspectTaskPlanAddCmd::Request::SharedPtr request)
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }

    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "id_task_info", request->id_task_info.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "id_task_plan", response->id_task_plan.c_str());

    WSsendJsonBack(json_fun, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::InspectTaskPlanDeleteCmdProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_id_task_plan = cJSON_GetObjectItem(cmd_data, "id_task_plan");
    if (value_id_task_plan != nullptr)
    {
        while (!InspectTaskPlanDeleteCmd_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "InspectTaskPlanDeleteCmd Service not available, waiting again...");
        }

        robot_msgs::srv::InspectTaskPlanDeleteCmd::Request::SharedPtr request;
        request->id_task_plan = value_id_task_plan->valuestring;

        cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
        if (json_fun_copy == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "copy is null");
            return;
        }

        auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::InspectTaskPlanDeleteCmd>::SharedFuture>();
        auto res_callback = [this, json_fun_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::InspectTaskPlanDeleteCmd>::SharedFuture future)
        {
            *shared_future_ptr = future;
            auto response = future.get();
            if (response->task_plan_not_exist)
            {
                TaskNotExistProcess(json_fun_copy);
                return;
            }
            else
            {
                if (response->deletion_success)
                {
                    InspectTaskPlanDeleteCmdSendback(json_fun_copy);
                    RCLCPP_INFO(this->get_logger(), "InspectTaskPlanDeleteCmd successful");
                    return;
                }
                else
                {
                    DeleteTaskFromDeviceFailProcess(json_fun_copy);
                    return;
                }
            }
        };
        auto future = InspectTaskPlanDeleteCmd_client->async_send_request(request, res_callback);
        *shared_future_ptr = future.future;

        auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                             [this, json_fun_copy, shared_future_ptr]()
                                             {
                                                 if (shared_future_ptr->valid() &&
                                                     shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                 {
                                                     DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                     return;
                                                 }
                                             });
    }
    else
    {
        WrongParamProcess("Param Error: id_task_plan is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskPlanDeleteCmdSendback(const cJSON *json_fun)
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }

    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::InspectTaskCtrlCmdProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    while (!CtrlModeQuery_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "CtrlModeQuery Service not available, waiting again...");
    }

    robot_msgs::srv::CtrlModeQuery::Request::SharedPtr request;

    cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
    if (json_fun_copy == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "copy is null");
        return;
    }

    cJSON *cmd_data_copy = cJSON_Duplicate(cmd_data, 1);
    if (cmd_data_copy == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "copy is null");
        return;
    }

    auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::CtrlModeQuery>::SharedFuture>();
    auto res_callback = [this, json_fun_copy, cmd_data_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::CtrlModeQuery>::SharedFuture future)
    {
        *shared_future_ptr = future;
        auto response = future.get();
        if (response->runtime_ctrl_mode)
        {
            cJSON *value_ctrl = cJSON_GetObjectItem(cmd_data_copy, "ctrl");
            if (value_ctrl != NULL)
            {
                if (strcmp(value_ctrl->valuestring, "e_start") == 0)
                {
                    ProcessStartTask(json_fun_copy, cmd_data_copy, value_ctrl);
                }
                else if (strcmp(value_ctrl->valuestring, "e_pause") == 0 ||
                         strcmp(value_ctrl->valuestring, "e_restart") == 0 ||
                         strcmp(value_ctrl->valuestring, "e_stop") == 0)
                {
                    ProcessControlTask(json_fun_copy, cmd_data_copy, value_ctrl);
                }
                else
                {
                    WrongParamProcess("Param Error: unexpected value_ctrl value.", json_fun_copy);
                }
            }
            else
            {
                WrongParamProcess("Param Error: value_ctrl is NULL!", json_fun_copy);
            }
        }
        else
        {
            DeviceModeErrorProcess(json_fun_copy);
        }
    };

    auto future = CtrlModeQuery_client->async_send_request(request, res_callback);
    *shared_future_ptr = future.future;

    auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                         [this, json_fun_copy, shared_future_ptr]()
                                         {
                                             if (shared_future_ptr->valid() &&
                                                 shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                             {
                                                 DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                 return;
                                             }
                                         });
}

void WSmsgs_Manager::ProcessStartTask(cJSON *json_fun, const cJSON *cmd_data, cJSON *value_ctrl)
{
    cJSON *value_id_task_info = cJSON_GetObjectItem(cmd_data, "id_task_info");
    if (value_id_task_info != NULL)
    {
        while (!InspectTaskCtrlCmd_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the InspectTaskCtrlCmd service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "InspectTaskCtrlCmd Service not available, waiting again...");
        }

        robot_msgs::srv::InspectTaskCtrlCmd::Request::SharedPtr request;
        request->id_task_info = value_id_task_info->valuestring;
        request->ctrl = value_ctrl->valuestring;

        cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
        if (json_fun_copy == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "copy is null");
            return;
        }

        auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::InspectTaskCtrlCmd>::SharedFuture>();
        auto res_callback = [this, json_fun_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::InspectTaskCtrlCmd>::SharedFuture future)
        {
            *shared_future_ptr = future;
            auto response = future.get();
            if (response->task_not_exist)
            {
                TaskNotExistProcess(json_fun_copy);
            }
            else
            {
                if (response->execute_success)
                {
                    InspectTaskCtrlCmdSendback(json_fun_copy, response);
                    RCLCPP_INFO(this->get_logger(), "InspectTaskCtrlCmd start successful");
                }
                else
                {
                    SendTask2DeviceFailProcess(json_fun_copy);
                }
            }
        };

        auto future = InspectTaskCtrlCmd_client->async_send_request(request, res_callback);
        *shared_future_ptr = future.future;

        auto task_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                  [this, json_fun_copy, shared_future_ptr]()
                                                  {
                                                      if (shared_future_ptr->valid() &&
                                                          shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                      {
                                                          DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                          return;
                                                      }
                                                  });
    }
    else
    {
        WrongParamProcess("Param Error: value_id_task_info is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::ProcessControlTask(cJSON *json_fun, const cJSON *cmd_data, cJSON *value_ctrl)
{
    cJSON *value_task_ticket = cJSON_GetObjectItem(cmd_data, "task_ticket");
    if (value_task_ticket != nullptr)
    {
        while (!InspectTaskCtrlCmd_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the InspectTaskCtrlCmd service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "InspectTaskCtrlCmd Service not available, waiting again...");
        }

        auto request = std::make_shared<robot_msgs::srv::InspectTaskCtrlCmd::Request>();
        request->task_ticket = value_task_ticket->valuestring;
        request->ctrl = value_ctrl->valuestring;

        cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
        if (json_fun_copy == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "copy is null");
            return;
        }

        auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::InspectTaskCtrlCmd>::SharedFuture>();

        auto res_callback = [this, json_fun_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::InspectTaskCtrlCmd>::SharedFuture future)
        {
            *shared_future_ptr = future;
            auto response = future.get();

            if (response->task_instance_not_exist)
            {
                TaskInstanceNotExistProcess(json_fun_copy);
            }
            else
            {
                if (response->execute_success)
                {
                    InspectTaskCtrlCmdSendback(json_fun_copy, response);
                    RCLCPP_INFO(this->get_logger(), "InspectTaskCtrlCmd control successful");
                }
                else
                {
                    SendTask2DeviceFailProcess(json_fun_copy);
                }
            }
        };

        auto future = InspectTaskCtrlCmd_client->async_send_request(request, res_callback);
        *shared_future_ptr = future.future;

        // 创建任务控制超时定时器
        auto task_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                  [this, json_fun_copy, shared_future_ptr]()
                                                  {
                                                      if (shared_future_ptr->valid() &&
                                                          shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                      {
                                                          DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                          return;
                                                      }
                                                  });
    }
    else
    {
        WrongParamProcess("Param Error: value_task_ticket is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskCtrlCmdSendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr response)
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }

    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "id_task_info", response->id_task_info.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "task_ticket", response->task_ticket.c_str());

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::InspectTaskInstanceCmdProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_page_size = cJSON_GetObjectItem(cmd_data, "page_size");
    cJSON *value_start_time = cJSON_GetObjectItem(cmd_data, "start_time");
    cJSON *value_end_time = cJSON_GetObjectItem(cmd_data, "end_time");
    cJSON *value_status = cJSON_GetObjectItem(cmd_data, "status");
    if (value_page_size != nullptr &&
        value_start_time != nullptr &&
        value_end_time != nullptr &&
        value_status != nullptr)
    {
        if (cJSON_IsArray(value_status))
        {
            while (!InspectTaskInstanceQuery_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "InspectTaskInstanceQuery Service not available, waiting again...");
            }

            robot_msgs::srv::InspectTaskInstanceQuery::Request::SharedPtr request;
            request->page_size = value_page_size->valueint;
            request->start_time = value_start_time->valuestring;
            request->end_time = value_end_time->valuestring;
            size_t array_size = cJSON_GetArraySize(value_status);
            for (size_t i = 0; i < array_size; i++)
            {
                cJSON *array_item = cJSON_GetArrayItem(value_status, i);
                if (strcmp(array_item->valuestring, "e_waiting") == 0 ||
                    strcmp(array_item->valuestring, "e_started") == 0 ||
                    strcmp(array_item->valuestring, "e_stopped") == 0 ||
                    strcmp(array_item->valuestring, "e_finished") == 0 ||
                    strcmp(array_item->valuestring, "e_outtime") == 0 ||
                    strcmp(array_item->valuestring, "e_paused_manual") == 0)
                {
                    std_msgs::msg::String data_temp;
                    data_temp.data = array_item->valuestring;
                    request->status.push_back(data_temp.data);
                }
                else
                {
                    WrongParamProcess("Param Error: unexpected status value.", json_fun);
                }
            }

            cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
            if (json_fun_copy == nullptr)
            {
                RCLCPP_ERROR(this->get_logger(), "copy is null");
                return;
            }

            auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::InspectTaskInstanceQuery>::SharedFuture>();
            auto res_callback = [this, json_fun_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::InspectTaskInstanceQuery>::SharedFuture future)
            {
                *shared_future_ptr = future;
                auto response = future.get();
                InspectTaskInstanceCmdSendback(json_fun_copy, response);
                RCLCPP_INFO(this->get_logger(), "InspectTaskPlanAddCmd successful");
                return;
            };

            auto future = InspectTaskInstanceQuery_client->async_send_request(request, res_callback);
            *shared_future_ptr = future.future;

            auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                 [this, json_fun_copy, shared_future_ptr]()
                                                 {
                                                     if (shared_future_ptr->valid() &&
                                                         shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                     {
                                                         DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                         return;
                                                     }
                                                 });
        }
        else
        {
            WrongParamProcess("Param Error: status is not an array.", json_fun);
            return;
        }
    }
    else
    {
        WrongParamProcess("Param Error: page_size or start_time or end_time or status is NULL!", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskInstanceCmdSendback(const cJSON *json_fun, const robot_msgs::srv::InspectTaskInstanceQuery::Response::SharedPtr response)
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }

    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON *task_instance_list_array = cJSON_CreateArray();
    size_t array_size = response->task_instance_list.size();
    std::vector<cJSON *> task_instance_array(array_size);
    for (size_t i = 0; i < array_size; i++)
    {
        task_instance_array[i] = cJSON_CreateObject();
        cJSON_AddStringToObject(task_instance_array[i], "id_task_info", response->task_instance_list[i].id_task_info.c_str());
        cJSON_AddStringToObject(task_instance_array[i], "task_name", response->task_instance_list[i].task_name.c_str());
        cJSON_AddStringToObject(task_instance_array[i], "task_ticket", response->task_instance_list[i].task_ticket.c_str());
        cJSON_AddStringToObject(task_instance_array[i], "status", response->task_instance_list[i].status.c_str());
        cJSON_AddStringToObject(task_instance_array[i], "start_time", response->task_instance_list[i].start_time.c_str());
        cJSON_AddStringToObject(task_instance_array[i], "end_time", response->task_instance_list[i].end_time.c_str());
        cJSON_AddNumberToObject(task_instance_array[i], "execute_time", response->task_instance_list[i].execute_time);
        cJSON_AddItemReferenceToArray(task_instance_list_array, task_instance_array[i]);
    }
    cJSON_AddNumberToObject(rt_data_SendBack, "total", array_size);
    cJSON_AddItemReferenceToObject(rt_data_SendBack, "list", task_instance_list_array);

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    for (size_t i = 0; i < array_size; i++)
    {
        cJSON_Delete(task_instance_array[i]);
    }
    return;
}

void WSmsgs_Manager::InspectTaskStartOrFinishServerRtProcess(const cJSON *json_fun, const cJSON *rt_data)
{
    cJSON *value_sub_sub_function = cJSON_GetObjectItem(json_fun, "sub_sub_function");
    cJSON *value_task_ticket = cJSON_GetObjectItem(rt_data, "task_ticket");

    if (value_sub_sub_function == nullptr || value_task_ticket == nullptr)
    {
        WrongParamProcess("Param Error: sub_sub_function or task_ticket is NULL.", json_fun);
        return;
    }

    while (!TaskStartOrFinishResponse_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "TaskStartOrFinishResponse Service not available, waiting again...");
    }

    robot_msgs::srv::TaskStartOrFinishResponse::Request::SharedPtr request;
    request->start_or_finish = value_sub_sub_function->valuestring;
    request->task_ticket = value_task_ticket->valuestring;

    if (request->start_or_finish == "start" ||
        request->start_or_finish == "complete")
    {
        cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
        if (json_fun_copy == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "copy is null");
            return;
        }
        auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::TaskStartOrFinishResponse>::SharedFuture>();
        auto res_callback = [this, json_fun_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::TaskStartOrFinishResponse>::SharedFuture future)
        {
            *shared_future_ptr = future;
            auto response = future.get();
            if(!response->task_ticket_exist)
            {
                RequestResourceNotExistProcess(json_fun_copy);
                return;
            }    
        };

        auto future = TaskStartOrFinishResponse_client->async_send_request(request, res_callback);
        *shared_future_ptr = future.future;

        auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                [this, json_fun_copy, shared_future_ptr]()
                                                {
                                                    if (shared_future_ptr->valid() &&
                                                        shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                    {
                                                        DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                        return;
                                                    }
                                                });
    }
    else
    {
        WrongParamProcess("Param Error: unexpected start_or_finish value.", json_fun);
        return;
    }
}

void WSmsgs_Manager::InspectTaskExecutionStartCallback(const robot_msgs::msg::InspectTaskExecutionStart::SharedPtr msg)
{
    cJSON* json_fun_SendUpload = cJSON_CreateObject();
    cJSON_AddStringToObject(json_fun_SendUpload, "type", "client_upload");
    cJSON_AddStringToObject(json_fun_SendUpload, "function", "task");
    cJSON_AddStringToObject(json_fun_SendUpload, "sub_function", "task_execution");
    cJSON_AddStringToObject(json_fun_SendUpload, "sub_sub_function", "start");

    cJSON* up_data_SendUpload = cJSON_CreateObject();
    cJSON_AddStringToObject(up_data_SendUpload, "id_device", id_device.c_str());
    cJSON_AddStringToObject(up_data_SendUpload, "id_task_info", msg->id_task_info.c_str());
    cJSON_AddStringToObject(up_data_SendUpload, "id_task_plan", msg->id_task_plan.c_str());
    cJSON_AddStringToObject(up_data_SendUpload, "task_ticket", msg->task_ticket.c_str());
    cJSON_AddStringToObject(up_data_SendUpload, "start_time", msg->start_time.c_str());

    WSsendJsonUpload(json_fun_SendUpload, up_data_SendUpload);

    cJSON_Delete(json_fun_SendUpload);
    cJSON_Delete(up_data_SendUpload);
    return;
}

void WSmsgs_Manager::InspectTaskExecutionCompleteCallback(const robot_msgs::msg::InspectTaskExecutionComplete::SharedPtr msg)
{
    cJSON* json_fun_SendUpload = cJSON_CreateObject();
    cJSON_AddStringToObject(json_fun_SendUpload, "type", "client_upload");
    cJSON_AddStringToObject(json_fun_SendUpload, "function", "task");
    cJSON_AddStringToObject(json_fun_SendUpload, "sub_function", "task_execution");
    cJSON_AddStringToObject(json_fun_SendUpload, "sub_sub_function", "complete");

    cJSON* up_data_SendUpload = cJSON_CreateObject();
    cJSON_AddStringToObject(up_data_SendUpload, "id_device", id_device.c_str());
    cJSON_AddStringToObject(up_data_SendUpload, "id_task_info", msg->id_task_info.c_str());
    cJSON_AddStringToObject(up_data_SendUpload, "task_ticket", msg->task_ticket.c_str());
    cJSON_AddNumberToObject(up_data_SendUpload, "point_count", msg->point_count);
    cJSON_AddStringToObject(up_data_SendUpload, "start_time", msg->start_time.c_str());
    cJSON_AddStringToObject(up_data_SendUpload, "end_time", msg->end_time.c_str());
    cJSON_AddNumberToObject(up_data_SendUpload, "execute_time", msg->execute_time);

    WSsendJsonUpload(json_fun_SendUpload, up_data_SendUpload);

    cJSON_Delete(json_fun_SendUpload);
    cJSON_Delete(up_data_SendUpload);
    return;
}

/***********************************************机器人信息查询***********************************************/
void WSmsgs_Manager::RobotInfoQueryProcess(const cJSON *json_fun)
{
    robot_msgs::srv::RobotInfoQuery::Request::SharedPtr request;
    while (!RobotInfoQuery_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "RobotInfoQuery Service not available, waiting again...");
    }

    cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
    if (json_fun_copy == NULL)
    {
        RCLCPP_ERROR(this->get_logger(), "copy is null");
        return;
    }

    auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::RobotInfoQuery>::SharedFuture>();

    auto res_callback = [this, json_fun_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::RobotInfoQuery>::SharedFuture future)
    {
        *shared_future_ptr = future;
        auto response = future.get();
        RobotInfoQuerySendback(json_fun_copy, response);
        RCLCPP_INFO(this->get_logger(), "res->succuss is true");
        return;
    };

    auto future = RobotInfoQuery_client->async_send_request(request, res_callback);
    *shared_future_ptr = future.future;
    auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                         [this, json_fun_copy, shared_future_ptr]()
                                         {
                                             if (shared_future_ptr->valid() &&
                                                 shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                             {
                                                 DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                 return;
                                             }
                                         });
}

void WSmsgs_Manager::RobotInfoQuerySendback(const cJSON *json_fun, const robot_msgs::srv::RobotInfoQuery::Response::SharedPtr response)
{
    if (json_fun == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "json_fun is NULL in [%s]. Cannot duplicate.", __FUNCTION__);
        return;
    }

    cJSON *json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    if (json_fun_SendBack == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "cJSON_Duplicate failed in [%s].", __FUNCTION__);
        return;
    }

    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "e_device_type", "e_robot_track");

    cJSON_AddNumberToObject(rt_data_SendBack, "e_runtime_speed", response->runtime_speed);
    cJSON_AddNumberToObject(rt_data_SendBack, "e_runtime_runtime", response->runtime_runtime);
    cJSON_AddNumberToObject(rt_data_SendBack, "e_runtime_mileage", response->runtime_mileage);

    if (device_type == "e_robot_track")
    {
        cJSON_AddStringToObject(rt_data_SendBack, "e_runtime_rfid", response->runtime_rfid.c_str());
    }

    cJSON_AddNumberToObject(rt_data_SendBack, "e_battery_percent", response->runtime_battery_percent);

    cJSON *runtime_position = cJSON_CreateObject();
    cJSON_AddStringToObject(runtime_position, "id_track", response->runtime_track_id.c_str());
    cJSON_AddNumberToObject(runtime_position, "track_pos", response->runtime_track_pos);
    cJSON_AddItemReferenceToObject(rt_data_SendBack, "e_runtime_position", runtime_position);

    cJSON *arm_end_pos = cJSON_CreateObject();
    cJSON_AddNumberToObject(arm_end_pos, "e_arm_end_pos_x", response->runtime_arm_end_pos_x);
    cJSON_AddNumberToObject(arm_end_pos, "e_arm_end_pos_y", response->runtime_arm_end_pos_y);
    cJSON_AddNumberToObject(arm_end_pos, "e_arm_end_pos_z", response->runtime_arm_end_pos_z);
    cJSON_AddNumberToObject(arm_end_pos, "e_arm_end_pos_rx", response->runtime_arm_end_pos_rx);
    cJSON_AddNumberToObject(arm_end_pos, "e_arm_end_pos_ry", response->runtime_arm_end_pos_ry);
    cJSON_AddNumberToObject(arm_end_pos, "e_arm_end_pos_rz", response->runtime_arm_end_pos_rz);
    cJSON_AddItemReferenceToObject(rt_data_SendBack, "e_arm_end_pos", arm_end_pos);

    cJSON *arm_joint_pos = cJSON_CreateObject();
    cJSON_AddNumberToObject(arm_joint_pos, "e_arm_joint_j1_pos", response->runtime_arm_joint_j1_pos);
    cJSON_AddNumberToObject(arm_joint_pos, "e_arm_joint_j2_pos", response->runtime_arm_joint_j2_pos);
    cJSON_AddNumberToObject(arm_joint_pos, "e_arm_joint_j3_pos", response->runtime_arm_joint_j3_pos);
    cJSON_AddNumberToObject(arm_joint_pos, "e_arm_joint_j4_pos", response->runtime_arm_joint_j4_pos);

    if (device_type == "e_robot_crawler")
    {
        cJSON_AddNumberToObject(arm_joint_pos, "e_arm_joint_j5_pos", response->runtime_arm_joint_j5_pos);
        cJSON_AddNumberToObject(arm_joint_pos, "e_arm_joint_j6_pos", response->runtime_arm_joint_j6_pos);
    }
    cJSON_AddItemReferenceToObject(rt_data_SendBack, "e_arm_joint_pos", arm_joint_pos);

    if (device_type == "e_robot_crawler")
    {
        cJSON_AddNumberToObject(rt_data_SendBack, "e_lift_pos", response->runtime_lift_pos);
    }

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    // 清理JSON对象
    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    cJSON_Delete(runtime_position);
    cJSON_Delete(arm_end_pos);
    cJSON_Delete(arm_joint_pos);
}

/***********************************************机器人心跳包***********************************************/
void WSmsgs_Manager::HeartbeatBagCallback(const robot_msgs::msg::HeartbeatBag::SharedPtr msg)
{
    cJSON *json_fun_HeartbeatBag = cJSON_CreateObject();
    cJSON_AddStringToObject(json_fun_HeartbeatBag, "type", "robot_heartbeat_bag");

    cJSON *heartbeat_bag = cJSON_CreateObject();
    cJSON_AddStringToObject(heartbeat_bag, "id_device", id_device.c_str());
    cJSON_AddNumberToObject(heartbeat_bag, "e_battery_percent", msg->runtime_battery_percent);
    cJSON *runtime_position = cJSON_CreateObject();
    cJSON_AddStringToObject(runtime_position, "id_track", msg->runtime_track_id.c_str());
    cJSON_AddNumberToObject(runtime_position, "track_pos", msg->runtime_track_pos);
    cJSON_AddItemReferenceToObject(heartbeat_bag, "e_runtime_position", runtime_position);
    cJSON_AddNumberToObject(heartbeat_bag, "e_ctrl_mode", msg->runtime_ctrl_mode);
    cJSON_AddNumberToObject(heartbeat_bag, "e_runtime_status", msg->runtime_status);
    if (!msg->runtime_task_ticket.empty())
    {
        cJSON_AddStringToObject(heartbeat_bag, "task_ticket", msg->runtime_task_ticket.c_str());
    }
    cJSON_AddNumberToObject(heartbeat_bag, "e_charge_status", msg->runtime_charge_status);
    cJSON_AddNumberToObject(heartbeat_bag, "e_timestamp", msg->runtime_timestamp);
    cJSON_AddNumberToObject(heartbeat_bag, "e_error_code", msg->runtime_error_code);

    WSsendJsonHeartbeatBag(json_fun_HeartbeatBag, heartbeat_bag);

    cJSON_Delete(json_fun_HeartbeatBag);
    cJSON_Delete(heartbeat_bag);
    cJSON_Delete(runtime_position);
    return;
}

/***********************************************系统时间校准***********************************************/
void WSmsgs_Manager::SystemTimeSyncCmdProcess(const cJSON *json_fun, const cJSON *cmd_data)
{
    cJSON *value_system_time = cJSON_GetObjectItem(cmd_data, "system_time");
    if (value_system_time != nullptr)
    {
        auto request = std::make_shared<robot_msgs::srv::SystemTimeSyncCmd::Request>();
        request->system_time = value_system_time->valuestring;
        while (!SystemTimeSyncCmd_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "SystemTimeSyncCmd Service not available, waiting again...");
        }
        cJSON *json_fun_copy = cJSON_Duplicate(json_fun, 1);
        if (json_fun_copy == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "copy is null");
            return;
        }
        auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::SystemTimeSyncCmd>::SharedFuture>();
        auto res_callback = [this, json_fun_copy, shared_future_ptr](rclcpp::Client<robot_msgs::srv::SystemTimeSyncCmd>::SharedFuture future)
        {
            *shared_future_ptr = future;
            if (json_fun_copy != nullptr)
            {
                auto response = future.get();
                if (response->sync_time_success)
                {
                    SystemTimeSyncCmdSendback(json_fun_copy, response->system_time_sync.c_str());
                    return;
                }
                else
                {
                    DeviceOperationFailedProcess(json_fun_copy);
                    return;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "json_fun is null");
                return;
            }
        };
        auto future = SystemTimeSyncCmd_client->async_send_request(request, res_callback);
        *shared_future_ptr = future.future;
        auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                             [this, json_fun_copy, shared_future_ptr]()
                                             {
                                                 if (shared_future_ptr->valid() &&
                                                     shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                 {
                                                     DeviceInternalCommunicationErrorProcess(json_fun_copy);
                                                     return;
                                                 }
                                             });
    }
    else
    {
        WrongParamProcess("Param Error: \"system_time\" is null", json_fun);
        return;
    }
}

void WSmsgs_Manager::SystemTimeSyncCmdSendback(const cJSON *json_fun, const char *system_time_sync)
{
    cJSON *json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON *rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON *rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "system_time", system_time_sync);

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}
