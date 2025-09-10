#include "ws_msg_manage.h"
using namespace std::placeholders;

/***********************************************初始化相关***********************************************/

WSmsgs_Manager_Server::WSmsgs_Manager_Server(const char* node_name) : rclcpp::Node(node_name)
{
    if(DeviceParamInit())
    {
        NodePublisherInit();
        NodeSubscriberInit();
        NodeServiceServerInit();
        NodeServiceClientInit();
    }
}

bool WSmsgs_Manager_Server::DeviceParamInit()
{
    // 参数初值
    ctrl_mode = 1;

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
    if(value_device_param != NULL)
    {
        cJSON* value_device_type = cJSON_GetObjectItem(value_device_param, "device_type");
        cJSON* value_devel_mode = cJSON_GetObjectItem(value_device_param, "Devel_Mode");
        cJSON* value_battery_percent_low_threshold = cJSON_GetObjectItem(value_device_param, "battery_percent_low_threshold");
        cJSON* value_battery_percent_high_threshold = cJSON_GetObjectItem(value_device_param, "battery_percent_high_threshold");
        cJSON* value_battery_percent_full_threshold = cJSON_GetObjectItem(value_device_param, "battery_percent_full_threshold");
        cJSON* value_ManualMode2TaskModeDuration = cJSON_GetObjectItem(value_device_param, "ManualMode2TaskModeDuration");
        if (value_device_type != NULL && value_devel_mode != NULL && value_battery_percent_low_threshold != NULL && value_battery_percent_high_threshold != NULL && value_battery_percent_full_threshold != NULL && value_ManualMode2TaskModeDuration != NULL)
        {
            device_type = value_device_type->valuestring;
            devel_mode = value_devel_mode->valuestring;
            battery_percent_low_threshold = value_battery_percent_low_threshold->valueint;
            battery_percent_high_threshold = value_battery_percent_high_threshold->valueint;
            battery_percent_full_threshold = value_battery_percent_full_threshold->valueint;
            ManualMode2TaskModeDuration = value_ManualMode2TaskModeDuration->valueint;
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

        RCLCPP_ERROR(this->get_logger(), "[%s] Device param error!", time_buf);
        return false;
    }
}

void WSmsgs_Manager_Server::NodePublisherInit()
{

}

void WSmsgs_Manager_Server::NodeSubscriberInit()
{
    // MotorCtrlNormalCmd_sub = this->create_subscription<robot_msgs::msg::MotorCtrlNormal>(
    //     "/MotorCtrlNormal_topic", 10, std::bind(&WSmsgs_Manager_Server::MotorCtrlNormalCmdCallback, this, _1)
    // );
}

void WSmsgs_Manager_Server::NodeServiceServerInit()
{
    ChangeCtrlModeCmd_server = this->create_service<robot_msgs::srv::ChangeCtrlModeCmd>(
        "ChangeCtrlModeCmd_service", std::bind(&WSmsgs_Manager_Server::ChangeCtrlModeCmdHandle, this, _1, _2)
    );

    CtrlModeQuery_server = this->create_service<robot_msgs::srv::CtrlModeQuery>(
        "CtrlModeQuery_service", std::bind(&WSmsgs_Manager_Server::CtrlModeQueryHandle, this, _1, _2)
    );
}

void WSmsgs_Manager_Server::NodeServiceClientInit()
{
    spinner_thread_ = std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
}

void WSmsgs_Manager_Server::NodeSpinnerStartup()
{

}

/***********************************************中断函数相关***********************************************/

bool WSmsgs_Manager_Server::ChangeCtrlModeCmdHandle(const robot_msgs::srv::ChangeCtrlModeCmd::Request::SharedPtr req,
                                                    robot_msgs::srv::ChangeCtrlModeCmd::Response::SharedPtr res)
{
    bool success = true;
    res->execute_success = success;
    RCLCPP_INFO(this->get_logger(), "Request handled. Mode: %d, Success: %s", req->ctrl_mode, success ? "true" : "false");
    return true;
}


bool WSmsgs_Manager_Server::CtrlModeQueryHandle(const robot_msgs::srv::CtrlModeQuery::Request::SharedPtr req,
                                                robot_msgs::srv::CtrlModeQuery::Response::SharedPtr res)
{
    (void) req;
    res->runtime_ctrl_mode = ctrl_mode;
    return true;
}

/***********************************************回调函数相关***********************************************/

void WSmsgs_Manager_Server::MotorCtrlNormalCmdCallback(const robot_msgs::msg::MotorCtrlNormal::SharedPtr msg)
{
    (void) msg;
}