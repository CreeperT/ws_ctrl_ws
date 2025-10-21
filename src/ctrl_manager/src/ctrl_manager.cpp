#include "ctrl_manager.h"
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

/***********************************************初始化相关***********************************************/

Ctrl_Manager::Ctrl_Manager(const char* node_name) : rclcpp::Node(node_name)
{
    if(DeviceParamInit())
    {
        NodePublisherInit();
        NodeSubscriberInit();
        NodeServiceServerInit();
        NodeServiceClientInit();

        std::thread RobotHeartbeatBagPub_thread([this](){
            HeartbeatBag_timer = this->create_wall_timer(std::chrono::seconds(10), std::bind(&Ctrl_Manager::RobotHeartbeatBagPub, this));
        });
        RobotHeartbeatBagPub_thread.detach();
    }
}

bool Ctrl_Manager::DeviceParamInit()
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

void Ctrl_Manager::NodePublisherInit()
{
    HeartbeatBag_pub = this->create_publisher<robot_msgs::msg::HeartbeatBag>("/HeartbeatBag_topic", 10);
}

void Ctrl_Manager::NodeSubscriberInit()
{
    // MotorCtrlNormalCmd_sub = this->create_subscription<robot_msgs::msg::MotorCtrlNormal>(
    //     "/MotorCtrlNormal_topic", 10, std::bind(&Ctrl_Manager::MotorCtrlNormalCmdCallback, this, _1)
    // );
}

void Ctrl_Manager::NodeServiceServerInit()
{
    ChangeCtrlModeCmd_server = this->create_service<robot_msgs::srv::ChangeCtrlModeCmd>(
        "ChangeCtrlModeCmd_service", std::bind(&Ctrl_Manager::ChangeCtrlModeCmdHandle, this, _1, _2)
    );

    CtrlModeQuery_server = this->create_service<robot_msgs::srv::CtrlModeQuery>(
        "CtrlModeQuery_service", std::bind(&Ctrl_Manager::CtrlModeQueryHandle, this, _1, _2)
    );
}

void Ctrl_Manager::NodeServiceClientInit()
{
    
}

void Ctrl_Manager::NodeSpinnerStartup()
{
    spinner_thread_ = std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
}

/***********************************************中断处理函数相关***********************************************/

bool Ctrl_Manager::ChangeCtrlModeCmdHandle(const robot_msgs::srv::ChangeCtrlModeCmd::Request::SharedPtr req,
                                                    robot_msgs::srv::ChangeCtrlModeCmd::Response::SharedPtr res)
{
    bool success = true;
    res->execute_success = success;
    RCLCPP_INFO(this->get_logger(), "Request handled. Mode: %d, Success: %s", req->ctrl_mode, success ? "true" : "false");
    return true;
}


bool Ctrl_Manager::CtrlModeQueryHandle(const robot_msgs::srv::CtrlModeQuery::Request::SharedPtr req,
                                                robot_msgs::srv::CtrlModeQuery::Response::SharedPtr res)
{
    (void) req;
    res->runtime_ctrl_mode = ctrl_mode;
    return true;
}

/***********************************************回调函数相关***********************************************/

// void Ctrl_Manager::MotorCtrlNormalCmdCallback(const robot_msgs::msg::MotorCtrlNormal::SharedPtr msg)
// {
//     /*四足机器人的手动控制实现已放在b2_manual_ctrl这个包里*/
// }

void Ctrl_Manager::RobotHeartbeatBagPub()
{
    {
        time_t now = time(NULL);
        struct tm localt;
        localtime_r(&now, &localt);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);
        RCLCPP_INFO(this->get_logger(), "[%s] RobotHeartbeatBagPub Thread Start!", time_buf);
    }
    
    // 仅供调试用
    auto heartbeat_msg = std::make_shared<robot_msgs::msg::HeartbeatBag>();

    battery_percent = 80;
    runtime_pos_id_track = "模拟轨道";
    runtime_pos_track_pos = 1234;
    ctrl_mode = 1;
    runtime_status = 1;
    charge_status = 0;
    error_code = 0000;

    heartbeat_msg->runtime_battery_percent = battery_percent;
    heartbeat_msg->runtime_track_id = runtime_pos_id_track;
    heartbeat_msg->runtime_track_pos = runtime_pos_track_pos;
    heartbeat_msg->runtime_ctrl_mode = ctrl_mode;
    heartbeat_msg->runtime_task_ticket = "模拟票据";
    heartbeat_msg->runtime_status = runtime_status;
    heartbeat_msg->runtime_charge_status = charge_status;
    heartbeat_msg->runtime_timestamp = std::time(nullptr);
    heartbeat_msg->runtime_error_code = error_code;

    HeartbeatBag_pub->publish(*heartbeat_msg);
}
