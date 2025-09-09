#include "ws_msg_manage.h"

using namespace std::placeholders;

/***********************************************初始化相关***********************************************/

WSmsgs_Manager::WSmsgs_Manager(const char* node_name) : Node(node_name)
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
    const char* homeDir = getenv("HOME");
    std::string jsonDir = homeDir;
    jsonDir = jsonDir + "/ws_ctrl_ws/src/DeviceParam.json";

    std::ifstream DeviceParamFile(jsonDir.c_str());
    if (!DeviceParamFile)
    {
        time_t now = time(NULL);
        struct tm localt;
        localtime_r(&now, &localt);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << time_buf << "] Can't open device param file!");
        return false;
    }

    std::string params((std::istreambuf_iterator<char>(DeviceParamFile)), std::istreambuf_iterator<char>());
    DeviceParamFile.close();

    cJSON* value_device_param = cJSON_Parse(params.c_str());
    if (value_device_param != NULL)
    {
        cJSON* value_device_type = cJSON_GetObjectItem(value_device_param, "device_type");
        cJSON* value_id_device = cJSON_GetObjectItem(value_device_param, "id_device");
        cJSON* value_devel_mode = cJSON_GetObjectItem(value_device_param, "Devel_Mode");
        cJSON* value_TrackArmMode = cJSON_GetObjectItem(value_device_param, "TrackArmMode");
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
            time_t now = time(NULL);
            struct tm localt;
            localtime_r(&now, &localt);
            char time_buf[64];
            strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);

            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << time_buf << "] Device param error!");
            cJSON_Delete(value_device_param);
            return false;
        }
    }
    else
    {
        time_t now = time(NULL);
        struct tm localt;
        localtime_r(&now, &localt);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << time_buf << "] Device param error!");
        return false;
    }
}

void WSmsgs_Manager::NodePublisherInit()
{
    // WebSocket发送消息话题发布
    WSsend_msgs_pub = this->create_publisher<std_msgs::msg::String>("/WSmsgs_send_topic", 10);

    // 移动平台运动控制话题发布
    MotorCtrlNormalCmd_pub = this->create_publisher<robot_msgs::msg::MotorCtrlNormal>("/MotorCtrlNormal_topic", 10);
}

void WSmsgs_Manager::NodeSubscriberInit()
{
    // WebSocket接收与发送消息话题
    WSreceive_msgs_sub = this->create_subscription<std_msgs::msg::String>(
        "WSmsgs_receive_topic", 10, std::bind(&WSmsgs_Manager::WSmsgsReceiveCallback, this, _1)
    );
}

void WSmsgs_Manager::NodeServiceClientInit()
{
    // 切换控制模式客户端
    ChangeCtrlModeCmd_client = this->create_client<robot_msgs::srv::ChangeCtrlModeCmd>("ChangeCtrlModeCmd_service");

    // 控制模式查询客户端
    CtrlModeQuery_client = this->create_client<robot_msgs::srv::CtrlModeQuery>("CtrlModeQuery_service");

    // 设备时间校准客户端
    // SystemTimeSyncCmd_client = this->create_client<robot_msgs::srv::SystemTimeSyncCmd>("SystemTimeSyncCmd_service");
}

void WSmsgs_Manager::NodeSpinnerStartup()
{
    // executor_.add_node(shared_from_this());
    spinner_thread_ = std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
}

/***********************************************WS通信相关***********************************************/

void WSmsgs_Manager::WSmsgsReceiveCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (devel_mode == "debug")
    {
        time_t now = time(NULL);
        struct tm localt;
        localtime_r(&now, &localt);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);

        RCLCPP_INFO_STREAM(this->get_logger(), "[" << time_buf << "] " << msg->data);
    }
    WSreceiveJsonParse(msg);
}

void WSmsgs_Manager::WSreceiveJsonParse(const std_msgs::msg::String::SharedPtr msg)
{
    cJSON* parsed_json = cJSON_Parse(msg->data.c_str());
    if(parsed_json != NULL)
    {
        cJSON* value_json_fun = cJSON_GetObjectItem(parsed_json, "json_fun");
        if(value_json_fun != NULL)
        {
            cJSON* value_type = cJSON_GetObjectItem(value_json_fun, "type");
            if(value_type != NULL)
            {
                if(strcmp(value_type->valuestring, "server_cmd") == 0) // 服务端发送命令
                {
                    cJSON* value_cmd_data = cJSON_GetObjectItem(parsed_json, "cmd_data"); // 控制命令
                    if(value_cmd_data != NULL)
                    {
                        cJSON* value_function = cJSON_GetObjectItem(value_json_fun, "function"); // 功能
                        cJSON* value_id_device = cJSON_GetObjectItem(value_cmd_data, "id_device"); // 设备id
                        if(strcmp(value_id_device->valuestring, id_device.c_str()) == 0)
                        {
                            if(value_function != NULL)
                            {
                                if(strcmp(value_function->valuestring, "device") == 0)
                                {
                                    cJSON* value_sub_funtion = cJSON_GetObjectItem(value_json_fun, "sub_function"); // 子功能
                                    if(value_sub_funtion != NULL)
                                    {
                                        // 机器人控制相关，待补充
                                        if(strcmp(value_sub_funtion->valuestring, "ctrl") == 0 || strcmp(value_sub_funtion->valuestring, "ctrl_mode") == 0)
                                        {
                                            DeviceCtrlCmdJsonParse(value_json_fun, value_sub_funtion->valuestring, value_cmd_data);
                                            cJSON_Delete(parsed_json);
                                            return;
                                        }
                                        // else if内容，待补充
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
                                else if(strcmp(value_function->valuestring, "task") == 0)
                                {   
                                    /*任务相关函数，待补充*/
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
                else if(strcmp(value_type->valuestring, "client_rt") == 0) // 客户端返回消息
                {
                    /*客户端返回消息逻辑，待补充*/
                    cJSON_Delete(parsed_json);
                    return;
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
    else return;
}

void WSmsgs_Manager::WSsendJsonBack(const cJSON* json_fun, const cJSON* rt_info, const cJSON* rt_data)
{
    cJSON* WSjson_SendBack = cJSON_CreateObject();
    cJSON_AddItemReferenceToObject(WSjson_SendBack, "json_fun", (cJSON*)json_fun);
    cJSON_AddItemReferenceToObject(WSjson_SendBack, "rt_info", (cJSON*)rt_info);
    cJSON_AddItemReferenceToObject(WSjson_SendBack, "rt_data", (cJSON*)rt_data);

    auto WSjson_SendBack_str = std_msgs::msg::String();
    char* temp_str = cJSON_Print(WSjson_SendBack);

    if (!temp_str)
    {
        time_t now = time(NULL);
        struct tm localt;
        localtime_r(&now, &localt);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << time_buf << "] WSsendJsonBack(): cJSON_Print() error!");
    }
    else
    {
        WSjson_SendBack_str.data = temp_str;
        WSsend_msgs_pub->publish(WSjson_SendBack_str);
        free(temp_str);
    }

    cJSON_Delete(WSjson_SendBack);
    return;
}

void WSmsgs_Manager::WSsendJsonCmd(const cJSON* json_fun, const cJSON* cmd_data)
{
    cJSON* WSjson_SendCmd = cJSON_CreateObject();
    cJSON_AddItemReferenceToObject(WSjson_SendCmd, "json_fun", (cJSON*)json_fun);
    cJSON_AddItemReferenceToObject(WSjson_SendCmd, "cmd_data", (cJSON*)cmd_data);

    auto WSjson_Sendcmd_str = std_msgs::msg::String();
    char* temp_str = cJSON_Print(WSjson_SendCmd);

    if (!temp_str)
    {
        time_t now = time(NULL);
        struct tm localt;
        localtime_r(&now, &localt);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << time_buf << "] WSsendJsoncmd(): cJSON_Print() error!");
    }
    else
    {
        WSjson_Sendcmd_str.data = temp_str;
        WSsend_msgs_pub->publish(WSjson_Sendcmd_str);
        free(temp_str);
    }

    cJSON_Delete(WSjson_SendCmd);
    return;
}

/***********************************************错误处理相关***********************************************/

void WSmsgs_Manager::WrongParamProcess(const char* err_msg, const cJSON* json_fun) // 错误id：1002
{
    cJSON* json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON* rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1002);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", err_msg);

    cJSON* rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::DeviceInternalCommunicationErrorProcess(const cJSON* json_fun) // 错误id：1003
{
    cJSON* json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON* rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1003);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Device internal communication error");

    cJSON* rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::RequestResourceNotExistProcess(const cJSON* json_fun) // 错误id：1005
{
    cJSON* json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON* rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1005);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "The request resource not exist");

    cJSON* rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::WrongIdDeviceProcess(const cJSON* json_fun) // 错误id：1007
{
    cJSON* json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON* rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1007);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Device id error");

    cJSON* rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::DeviceOperationFailedProcess(const cJSON* json_fun) // 错误id：2201
{
    cJSON* json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON* rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 2201);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Device operation failed");

    cJSON* rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::DeviceModeErrorProcess(const cJSON* json_fun) // 错误id：2203
{
    cJSON* json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON* rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 2203);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "Device mode error");

    cJSON* rt_data_SendBack = cJSON_CreateObject();

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

/***********************************************机器人控制相关***********************************************/

void WSmsgs_Manager::DeviceCtrlCmdJsonParse(const cJSON* json_fun, const char* sub_function, const cJSON* cmd_data)
{
    // 切换控制模式
    if(strcmp(sub_function, "ctrl_mode") == 0)
    {
        cJSON* value_ctrl_mode = cJSON_GetObjectItem(cmd_data, "mode");
        RCLCPP_INFO(this->get_logger(), "json_raw:%s", cJSON_Print(value_ctrl_mode));
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
    else if(strcmp(sub_function, "ctrl") == 0)
    {
        auto request = std::make_shared<robot_msgs::srv::CtrlModeQuery::Request>();
        while (!CtrlModeQuery_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "CtrlModeQuery Service not available, waiting again...");
        }
        
        auto future = CtrlModeQuery_client->async_send_request(request,
            [this, json_fun, cmd_data](rclcpp::Client<robot_msgs::srv::CtrlModeQuery>::SharedFuture future){
                auto response = future.get();
                if (response->runtime_ctrl_mode == 1)
                {
                    
                    cJSON* value_ctrl_type = cJSON_GetObjectItem(cmd_data, "ctrl_type");
                    cJSON* value_ctrl_value = cJSON_GetObjectItem(cmd_data, "ctrl_value");

                    if (value_ctrl_type != NULL && value_ctrl_value != NULL)
                    {
                        // 移动平台运动控制
                        if (strcmp(value_ctrl_type->valuestring, "e_ctrl_motor") == 0)
                        {
                            MotorCtrlCmdProcess(json_fun, value_ctrl_value);
                            return;
                        }

                        /*else if其他设备控制，待补充*/
                        else
                        {
                            WrongParamProcess("Param Error: \"ctrl_type\" does not have a correct param.", json_fun);
                            return;
                        }
                    }
                    else
                    {
                        WrongParamProcess("Param Error: \"ctrl_type\" or \"ctrl_value\" is NULL.", json_fun);
                        return;
                    }
                }
                else
                {
                    DeviceModeErrorProcess(json_fun);
                    return;
                }
            });
        auto wait_result = future.wait_for(std::chrono::seconds(2));
        if(wait_result != std::future_status::ready)
        {
            DeviceInternalCommunicationErrorProcess(json_fun);
            return;
        }
    }

    /*else if其他指令，待补充*/

    else return;
}

void WSmsgs_Manager::ChangeCtrlModeCmdProcess(const cJSON* json_fun, const uint8_t target_ctrl_mode)
{
    if (target_ctrl_mode == 0 || target_ctrl_mode == 1)
    {
        auto request = std::make_shared<robot_msgs::srv::ChangeCtrlModeCmd::Request>();
        request->ctrl_mode = target_ctrl_mode;

        while (!ChangeCtrlModeCmd_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "ChangeCtrlModeCmd Service not available, waiting again...");
        }
        auto future = ChangeCtrlModeCmd_client->async_send_request(request,
            [this, json_fun, target_ctrl_mode](rclcpp::Client<robot_msgs::srv::ChangeCtrlModeCmd>::SharedFuture future){
                auto response = future.get();
                if (response->execute_success)
                {
                    ChangeCtrlModeCmdSendback(json_fun, target_ctrl_mode);
                }
                else
                {
                    DeviceOperationFailedProcess(json_fun);
                }
            });
        auto wait_result = future.wait_for(std::chrono::seconds(2));
        if(wait_result != std::future_status::ready)
        {
            DeviceInternalCommunicationErrorProcess(json_fun);
            return;
        }
    }
    else return;
}

void WSmsgs_Manager::ChangeCtrlModeCmdSendback(const cJSON* json_fun, const uint8_t target_ctrl_mode)
{
    cJSON* json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON* rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON* rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON_AddNumberToObject(rt_data_SendBack, "mode", target_ctrl_mode);

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}

void WSmsgs_Manager::MotorCtrlCmdProcess(const cJSON* json_fun, const cJSON* ctrl_value)
{
    cJSON* value_action = cJSON_GetObjectItem(ctrl_value, "action");
    cJSON* value_speed = cJSON_GetObjectItem(ctrl_value, "speed");

    if(value_action != NULL)
    {
        float speed = -10; // -10表示服务器发送指令中speed为空，使用机器人上设定的默认值
        if(value_speed != NULL)
        {
            speed = value_speed->valuedouble;
        }
        else
        {
            WrongParamProcess("Param Error: \"speed\" is NULL.", json_fun);
            return;
        }

        if(value_action != NULL)
        {
            if(strcmp(value_action->string, "e_move_forward") == 0)
            {
                robot_msgs::msg::MotorCtrlNormal MotorCtrlNormalMsg;
                MotorCtrlNormalMsg.command = "move_forward";
                MotorCtrlNormalMsg.run_speed = speed;
                MotorCtrlNormalCmd_pub->publish(MotorCtrlNormalMsg);
                DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
                return;
                // auto MotorCtrlNormalMsg = std::make_unique<robot_msgs::msg::MotorCtrlNormal>();
                // MotorCtrlNormalMsg->command = "move_forward";
                // MotorCtrlNormalMsg->run_speed = speed;
                // MotorCtrlNormalCmd_pub->publish(std::move(MotorCtrlNormalMsg));
                // DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
                // return;
            }
            else if(strcmp(value_action->string, "e_move_back") == 0)
            {
                robot_msgs::msg::MotorCtrlNormal MotorCtrlNormalMsg;
                MotorCtrlNormalMsg.command = "move_back";
                MotorCtrlNormalMsg.run_speed = speed;
                MotorCtrlNormalCmd_pub->publish(MotorCtrlNormalMsg);
                DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
                return;
                // auto MotorCtrlNormalMsg = std::make_unique<robot_msgs::msg::MotorCtrlNormal>();
                // MotorCtrlNormalMsg->command = "move_back";
                // MotorCtrlNormalMsg->run_speed = speed;
                // MotorCtrlNormalCmd_pub->publish(std::move(MotorCtrlNormalMsg));
                // DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
                // return;
            }
            else if(strcmp(value_action->string, "e_turn_left") == 0)
            {
                robot_msgs::msg::MotorCtrlNormal MotorCtrlNormalMsg;
                MotorCtrlNormalMsg.command = "turn_left";
                MotorCtrlNormalMsg.run_speed = speed;
                MotorCtrlNormalCmd_pub->publish(MotorCtrlNormalMsg);
                DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
                return;
                // auto MotorCtrlNormalMsg = std::make_unique<robot_msgs::msg::MotorCtrlNormal>();
                // MotorCtrlNormalMsg->command = "turn_left";
                // MotorCtrlNormalMsg->run_speed = speed;
                // MotorCtrlNormalCmd_pub->publish(std::move(MotorCtrlNormalMsg));
                // DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
                // return;
            }
            else if(strcmp(value_action->string, "e_turn_right") == 0)
            {
                robot_msgs::msg::MotorCtrlNormal MotorCtrlNormalMsg;
                MotorCtrlNormalMsg.command = "turn_right";
                MotorCtrlNormalMsg.run_speed = speed;
                MotorCtrlNormalCmd_pub->publish(MotorCtrlNormalMsg);
                DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
                return;
                // auto MotorCtrlNormalMsg = std::make_unique<robot_msgs::msg::MotorCtrlNormal>();
                // MotorCtrlNormalMsg->command = "turn_right";
                // MotorCtrlNormalMsg->run_speed = speed;
                // MotorCtrlNormalCmd_pub->publish(std::move(MotorCtrlNormalMsg));
                // DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
                // return;
            }
            else if(strcmp(value_action->string, "e_move_stop") == 0)
            {
                robot_msgs::msg::MotorCtrlNormal MotorCtrlNormalMsg;
                MotorCtrlNormalMsg.command = "move_stop";
                MotorCtrlNormalMsg.run_speed = speed;
                MotorCtrlNormalCmd_pub->publish(MotorCtrlNormalMsg);
                DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
                return;
                // auto MotorCtrlNormalMsg = std::make_unique<robot_msgs::msg::MotorCtrlNormal>();
                // MotorCtrlNormalMsg->command = "move_forward";
                // MotorCtrlNormalMsg->run_speed = speed;
                // MotorCtrlNormalCmd_pub->publish(std::move(MotorCtrlNormalMsg));
                // DeviceCtrlCmdSendback(json_fun, "e_ctrl_motor");
                // return;
            }
            else
            {
                WrongParamProcess("Param Error: \"action\" dose not have a correct param.", json_fun);
                return;
            }
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

void WSmsgs_Manager::DeviceCtrlCmdSendback(const cJSON* json_fun, const char* ctrl_type)
{
    cJSON* json_fun_SendBack = cJSON_CreateObject();
    json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    while (json_fun_SendBack == NULL)
    {
        json_fun_SendBack = cJSON_Duplicate(json_fun, 1);
    }
    cJSON_ReplaceItemInObject(json_fun_SendBack, "type", cJSON_CreateString("client_rt"));

    cJSON* rt_info_SendBack = cJSON_CreateObject();
    cJSON_AddNumberToObject(rt_info_SendBack, "code", 1000);
    cJSON_AddStringToObject(rt_info_SendBack, "msg", "ok");

    cJSON* rt_data_SendBack = cJSON_CreateObject();
    cJSON_AddStringToObject(rt_data_SendBack, "id_device", id_device.c_str());
    cJSON_AddStringToObject(rt_data_SendBack, "ctrl_type", ctrl_type);

    WSsendJsonBack(json_fun_SendBack, rt_info_SendBack, rt_data_SendBack);

    cJSON_Delete(json_fun_SendBack);
    cJSON_Delete(rt_info_SendBack);
    cJSON_Delete(rt_data_SendBack);
    return;
}
