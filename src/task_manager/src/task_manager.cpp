#include "task_manager.h"
using std::placeholders::_1;
using std::placeholders::_2;

Task_Manager::Task_Manager(const char* node_name) : Node(node_name)
{
    if (DeviceParamInit())
    {
        NodePublisherInit();
        NodeServiceServerInit();
    }
}

Task_Manager::~Task_Manager()
{
    if (pTaskFileManager)
    {
        pTaskFileManager.reset();
    }

    if (pTaskExecute)
    {
        pTaskExecute.reset();
    }
}

/********************************************************初始化*************************************************************/
bool Task_Manager::DeviceParamInit()
{
    const char* homeDir = getenv("HOME");
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

    cJSON* value_device_param = cJSON_Parse(params.c_str());
    if (value_device_param != NULL)
    {
        cJSON* value_device_type = cJSON_GetObjectItem(value_device_param, "device_type");
        cJSON* value_id_device = cJSON_GetObjectItem(value_device_param, "id_device");
        cJSON* value_devel_mode = cJSON_GetObjectItem(value_device_param, "Devel_Mode");

        cJSON* value_ImgDataCollectWaitTime = cJSON_GetObjectItem(value_device_param, "ImgDataCollectWaitTime");
        cJSON* value_TaskStartOrFinishResponseDuration = cJSON_GetObjectItem(value_device_param, "TaskStartOrFinishResponseDuration");
        cJSON* value_WiperWorkDuration = cJSON_GetObjectItem(value_device_param, "WiperWorkDurationSeconds");

        cJSON* value_TrackArmMode = cJSON_GetObjectItem(value_device_param, "TrackArmMode");

        if (value_device_type != NULL && value_id_device != NULL && value_devel_mode != NULL && value_ImgDataCollectWaitTime != NULL && value_TaskStartOrFinishResponseDuration != NULL && value_TrackArmMode != NULL && value_WiperWorkDuration != NULL)
        {
            device_type = value_device_type->valuestring;
            id_device = value_id_device->valuestring;
            devel_mode = value_devel_mode->valuestring;

            ImgDataCollectWaitTime = value_ImgDataCollectWaitTime->valueint;
            TaskStartOrFinishResponseDuration = value_TaskStartOrFinishResponseDuration->valueint;
            WiperWorkDuration = value_WiperWorkDuration->valueint;

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

void Task_Manager::NodePublisherInit()
{

}

void Task_Manager::NodeServiceServerInit()
{
    InspectTaskInfoListQuery_server = this->create_service<robot_msgs::srv::InspectTaskInfoListQuery>(
        "InspectTaskInfoListQuery_service", std::bind(&Task_Manager::InspectTaskInfoListQueryHandle, this, _1, _2));
    InspectTaskInfoDetailQuery_server = this->create_service<robot_msgs::srv::InspectTaskInfoDetailQuery>(
        "InspectTaskInfoDetailQuery_service", std::bind(&Task_Manager::InspectTaskInfoDetailQueryHandle, this, _1, _2));
    InspectTaskInfoConfigureCmd_server = this->create_service<robot_msgs::srv::InspectTaskInfoConfigureCmd>(
        "InspectTaskInfoConfigureCmd_service", std::bind(&Task_Manager::InspectTaskInfoConfigureCmdHandle, this, _1, _2));
    InspectTaskInfoDeleteCmd_server = this->create_service<robot_msgs::srv::InspectTaskInfoDeleteCmd>(
        "InspectTaskInfoDeleteCmd_service", std::bind(&Task_Manager::InspectTaskInfoDeleteCmdHandle, this, _1, _2));

    InspectTaskPlanListQuery_server = this->create_service<robot_msgs::srv::InspectTaskPlanListQuery>(
        "InspectTaskPlanListQuery_service", std::bind(&Task_Manager::InspectTaskPlanListQueryHandle, this, _1, _2));
    InspectTaskPlanAddCmd_server = this->create_service<robot_msgs::srv::InspectTaskPlanAddCmd>(
        "InspectTaskPlanAddCmd_service", std::bind(&Task_Manager::InspectTaskPlanAddCmdHandle, this, _1, _2));
    InspectTaskPlanDeleteCmd_server = this->create_service<robot_msgs::srv::InspectTaskPlanDeleteCmd>(
        "InspectTaskPlanDeleteCmd_service", std::bind(&Task_Manager::InspectTaskPlanDeleteCmdHandle, this, _1, _2));

    InspectTaskCtrlCmd_server = this->create_service<robot_msgs::srv::InspectTaskCtrlCmd>(
        "InspectTaskCtrlCmd_service", std::bind(&Task_Manager::InspectTaskCtrlCmdHandle, this, _1, _2));

    InspectTaskInstanceQuery_server = this->create_service<robot_msgs::srv::InspectTaskInstanceQuery>(
        "InspectTaskInstanceQuery_service", std::bind(&Task_Manager::InspectTaskInstanceQueryHandle, this, _1, _2));
}

/********************************************************服务中断函数*************************************************************/

bool Task_Manager::InspectTaskInfoListQueryHandle(robot_msgs::srv::InspectTaskInfoListQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoListQuery::Response::SharedPtr res)
{
    return pTaskFileManager->TaskInfoListQuery(req, res);
}

bool Task_Manager::InspectTaskInfoDetailQueryHandle(robot_msgs::srv::InspectTaskInfoDetailQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoDetailQuery::Response::SharedPtr res)
{
    return pTaskFileManager->TaskInfoDetailQuery(req, res);
}

bool Task_Manager::InspectTaskInfoConfigureCmdHandle(robot_msgs::srv::InspectTaskInfoConfigureCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoConfigureCmd::Response::SharedPtr res)
{
    if (req->action == "configure")
    {
        return pTaskFileManager->TaskInfoConfigure(req, res);
    }
    else if (req->action == "create")
    {
        return pTaskFileManager->TaskInfoCreate(req, res);
    }
    else
    {
        return false;
    }
}

bool Task_Manager::InspectTaskInfoDeleteCmdHandle(robot_msgs::srv::InspectTaskInfoDeleteCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoDeleteCmd::Response::SharedPtr res)
{
    return pTaskFileManager->TaskInfoDelete(req, res);
}

bool Task_Manager::InspectTaskPlanListQueryHandle(robot_msgs::srv::InspectTaskPlanListQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanListQuery::Response::SharedPtr res)
{
    return pTaskFileManager->TaskPlanListQuery(req, res);
}

bool Task_Manager::InspectTaskPlanAddCmdHandle(robot_msgs::srv::InspectTaskPlanAddCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanAddCmd::Response::SharedPtr res)
{
    return pTaskFileManager->TaskPlanAdd(req, res);
}

bool Task_Manager::InspectTaskPlanDeleteCmdHandle(robot_msgs::srv::InspectTaskPlanDeleteCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanDeleteCmd::Response::SharedPtr res)
{
    return pTaskFileManager->TaskPlanDelete(req, res);
}

bool Task_Manager::InspectTaskCtrlCmdHandle(robot_msgs::srv::InspectTaskCtrlCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr res)
{
    if (req->ctrl == "e_start")
    {
        return pTaskExecute->TaskCtrlStart(req, res);
    }
    else if (req->ctrl == "e_pause")
    {
        return pTaskExecute->TaskCtrlPause(req, res);
    }
    else if (req->ctrl == "e_restart")
    {
        return pTaskExecute->TaskCtrlRestart(req, res);
    }
    else if (req->ctrl == "e_stop")
    {
        return pTaskExecute->TaskCtrlStop(req, res);
    }
    else
    {
        return false;
    }
}

bool Task_Manager::InspectTaskInstanceQueryHandle(robot_msgs::srv::InspectTaskInstanceQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInstanceQuery::Response::SharedPtr res)
{
    return pTaskFileManager->TaskInstanceQuery(req, res);
}
