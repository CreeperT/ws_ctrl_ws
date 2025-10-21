#include "info_manager.h"
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

/***********************************************初始化相关***********************************************/
Info_Manager::Info_Manager(const char *node_name) : Node(node_name)
{
    if (DeviceParamInit())
    {
        NodePublisherInit();
        NodeSubscriberInit();
        NodeServiceServerInit();
        NodeServiceClientInit();
    }
}

Info_Manager::~Info_Manager()
{
}

bool Info_Manager::DeviceParamInit()
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

void Info_Manager::NodePublisherInit()
{
}

void Info_Manager::NodeSubscriberInit()
{
}

void Info_Manager::NodeServiceServerInit()
{
    RobotInfoQuery_server = this->create_service<robot_msgs::srv::RobotInfoQuery>(
        "RobotInfoQuery_server", std::bind(&Info_Manager::RobotInfoQueryHandle, this, _1, _2));
}

void Info_Manager::NodeServiceClientInit()
{
}

void Info_Manager::NodeSpinnerStartup()
{
    executor_.add_node(shared_from_this());
    spinner_thread_ = std::thread([this]()
                                  { executor_.spin(); });
    spinner_thread_.join();
    // spinner_thread_ = std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
}

/***********************************************机器人信息查询***********************************************/
bool Info_Manager::RobotInfoQueryHandle(robot_msgs::srv::RobotInfoQuery::Request::SharedPtr req,
                                        robot_msgs::srv::RobotInfoQuery::Response::SharedPtr res)
{
}
