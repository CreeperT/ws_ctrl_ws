#include "task_execute.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

Task_Execute::Task_Execute(const std::string id_device_, const std::string device_type_, const std::string devel_mode_, const std::string TrackArmMode_,
                           const uint16_t ImgDataCollectWaitTime_, const uint16_t TaskStartOrFinishResponseDuration_, const uint16_t WiperWorkDuration_) : Node("task_execute")
{
    id_device = id_device_;
    device_type = device_type_;
    devel_mode = devel_mode_;
    TrackArmMode = TrackArmMode_;
    ImgDataCollectWaitTime = ImgDataCollectWaitTime_;
    TaskStartOrFinishResponseDuration = TaskStartOrFinishResponseDuration_;
    WiperWorkDuration = WiperWorkDuration_;

    getTaskFilesDir();
    NodePublisherInit();
    NodeServiceClientInit();
    NodeServiceServerInit();

    TaskExecuteInit();

    std::thread CronPlanTaskListUpdateThread(&Task_Execute::CronPlanTaskListUpdate, this);
    CronPlanTaskListUpdateThread.detach();

    std::thread CronPlanTaskTriggerThread(&Task_Execute::CronPlanTaskTrigger, this);
    CronPlanTaskTriggerThread.detach();

    std::thread TaskStartOrFinishResponseCheckThread(&Task_Execute::TaskStartOrFinishResponseCheck, this);
    TaskStartOrFinishResponseCheckThread.detach();
}

/********************************************************初始化*************************************************************/

void Task_Execute::getTaskFilesDir()
{
    const char *homeDir = getenv("HOME");
    TaskFilesDir = homeDir;
    TaskFilesDir = TaskFilesDir + "ws_ctrl_ws/src/task_manager/task_files/";
}

void Task_Execute::NodePublisherInit()
{
    InspectTaskExecutionStart_pub = this->create_publisher<robot_msgs::msg::InspectTaskExecutionStart>("/InspectTaskExecutionStart_topic", 10);

    InspectTaskExecutionComplete_pub = this->create_publisher<robot_msgs::msg::InspectTaskExecutionComplete>("/InspectTaskExecutionComplete_topic", 10);

    TaskFinishAndExecuteBatteryCharge_pub = this->create_publisher<std_msgs::msg::Bool>("/TaskFinishAndExecuteBatteryCharge_topic", 10);

    if (device_type == "e_robot_track" ||
        device_type == "e_robot_crawler" ||
        device_type == "e_robot_quadrupedal")
    {
        ArmCtrlStopCmd_pub = this->create_publisher<std_msgs::msg::Bool>("/ArmStopCtrl_topic", 10);
    }

    if (device_type == "e_robot_track")
    {
        TrackMotorStopCtrl_pub = this->create_publisher<std_msgs::msg::Int32>("/robot_stop_or_start", 10);
    }

    if (device_type == "e_robot_crawler")
    {
        MotorStopCtrl_pub = this->create_publisher<std_msgs::msg::Bool>("/CrawlerMotorStop_topic", 10);
        LiftCtrlStopCmd_pub = this->create_publisher<std_msgs::msg::Bool>("/LiftStopCtrl_topic", 10);
    }

    if (device_type == "e_robot_quadrupedal")
    {
        MotorStopCtrl_pub = this->create_publisher<std_msgs::msg::Bool>("/QuadrupedalMotorStopCtrl_topic", 10);
    }
}

void Task_Execute::NodeServiceClientInit()
{
    // 创建服务客户端
    MotorCtrltoPosSrv_client = this->create_client<robot_msgs::srv::MotorCtrltoPosSrv>("MotorCtrltoPosSrv_service");
    
    // 等待服务可用
    while (!MotorCtrltoPosSrv_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for MotorCtrltoPosSrv_service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for MotorCtrltoPosSrv_service...");
    }

    if (device_type == "e_robot_crawler" || device_type == "e_robot_track" || device_type == "e_robot_quadrupedal")
    {
        ArmCtrlSrv_client = this->create_client<robot_msgs::srv::ArmCtrlSrv>("ArmCtrlSrv_service");
        
        while (!ArmCtrlSrv_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for ArmCtrlSrv_service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for ArmCtrlSrv_service...");
        }
    }

    if (device_type == "e_robot_crawler")
    {
        LiftCtrlSrv_client = this->create_client<robot_msgs::srv::LiftCtrlSrv>("LiftCtrlSrv_service");
        
        while (!LiftCtrlSrv_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for LiftCtrlSrv_service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for LiftCtrlSrv_service...");
        }
    }

    // 扩展设备控制服务
    ExtendDevCtrlSrv_client = this->create_client<robot_msgs::srv::ExtendDevCtrlSrv>("ExtendDevCtrlSrv_service");
    
    while (!ExtendDevCtrlSrv_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for ExtendDevCtrlSrv_service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for ExtendDevCtrlSrv_service...");
    }

    // 任务数据收集服务
    TaskDataCollectSrv_client = this->create_client<robot_msgs::srv::TaskDataCollectSrv>("TaskDataCollectSrv_service");
    
    while (!TaskDataCollectSrv_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for TaskDataCollectSrv_service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for TaskDataCollectSrv_service...");
    }

    // 控制模式查询服务
    CtrlModeQuery_client = this->create_client<robot_msgs::srv::CtrlModeQuery>("CtrlModeQuery_service");
    
    while (!CtrlModeQuery_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for CtrlModeQuery_service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for CtrlModeQuery_service...");
    }

    RCLCPP_INFO(this->get_logger(), "All services clients initialized successfully!");
}

void Task_Execute::NodeServiceServerInit()
{
    // 创建服务服务器
    TaskStartOrFinishResponse_server = this->create_service<robot_msgs::srv::TaskStartOrFinishResponse>(
        "TaskStartOrFinishResponse_service",
        std::bind(&Task_Execute::TaskStartOrFinishResponseHandle, this, _1, _2));

    TaskExecuteStatusQuery_server = this->create_service<robot_msgs::srv::TaskExecuteStatusQuery>(
        "TaskExecuteStatusQuery_service", 
        std::bind(&Task_Execute::TaskExecuteStatusQueryHandle, this, _1, _2));

    BatteryChargePauseTask_server = this->create_service<robot_msgs::srv::BatteryChargePauseTask>(
        "BatteryChargePauseTask_service",
        std::bind(&Task_Execute::BatteryChargePauseTaskHandle, this, _1, _2));

    BatteryChargeStartTask_server = this->create_service<robot_msgs::srv::BatteryChargeStartTask>(
        "BatteryChargeStartTask_service",
        std::bind(&Task_Execute::BatteryChargeStartTaskHandle, this, _1, _2));

    ModeChangePauseTask_server = this->create_service<robot_msgs::srv::ModeChangePauseTask>(
        "ModeChangePauseTask_service",
        std::bind(&Task_Execute::ModeChangePauseTaskHandle, this, _1, _2));

    ModeChangeStartTask_server = this->create_service<robot_msgs::srv::ModeChangeStartTask>(
        "ModeChangeStartTask_service",
        std::bind(&Task_Execute::ModeChangeStartTaskHandle, this, _1, _2));

    if (device_type == "e_robot_track" || device_type == "e_robot_crawler")
    {
        AvoidObstaclesPauseTask_server = this->create_service<robot_msgs::srv::AvoidObstaclesPauseTask>(
            "AvoidObstaclesPauseTask_service",
            std::bind(&Task_Execute::AvoidObstaclesPauseTaskHandle, this, _1, _2));

        AvoidObstaclesStartTask_server = this->create_service<robot_msgs::srv::AvoidObstaclesStartTask>(
            "AvoidObstaclesStartTask_service",
            std::bind(&Task_Execute::AvoidObstaclesStartTaskHandle, this, _1, _2));
    }
}

std::string Task_Execute::generateRandomUUID()
{
    std::random_device rd;
    std::mt19937 gen(rd());  // 使用Mersenne Twister随机数引擎
    std::uniform_int_distribution<int> dis(0, 15);  // 生成0到15的随机数 (十六进制范围)

    std::stringstream uuid;
    uuid << std::uppercase;  // 使用大写字母

    // 生成8-4-4-4-12格式的UUID
    int lengths[] = {8, 4, 4, 4, 12};  // 每段的长度
    for (int i = 0; i < 5; ++i)
    {
        if (i > 0)
        {
            uuid << "-";  // 插入分隔符
        }
        for (int j = 0; j < lengths[i]; ++j)
        {
            uuid << std::hex << dis(gen);  // 生成十六进制字符
        }
    }

    return uuid.str();
}

/********************************************************任务控制*************************************************************/
bool Task_Execute::TaskCtrlStart(robot_msgs::srv::InspectTaskCtrlCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr res)
{
    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_execute_flag == "execute" && task_execute_infos[i].task_execute_type == "manual_task")
        {
            res->execute_success = false;
            return true;
        }
    }

    std::string TaskInfoIndexFileDir = TaskFilesDir + "task_info_files/TaskInfoIndex.json";
    std::ifstream TaskInfoIndexFileRead(TaskInfoIndexFileDir.c_str());
    if (!TaskInfoIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Can't open TaskInfoIndex.json (" << req->id_task_info << ")!");
        return false;
    }

    std::string TaskInfoIndexContent((std::istreambuf_iterator<char>(TaskInfoIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInfoIndexFileRead.close();

    cJSON* TaskInfoIndexJson = cJSON_Parse(TaskInfoIndexContent.c_str());
    if (TaskInfoIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Failed to parse TaskInfoIndexJson (" << req->id_task_info << ")!");
        return false;
    }

    cJSON* task_info_index_array = cJSON_GetObjectItem(TaskInfoIndexJson, "TaskInfoIndex");
    if (task_info_index_array == NULL || !cJSON_IsArray(task_info_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Failed to get task_info_index_array (" << req->id_task_info << ")!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    size_t TaskInfoIndexArraySize = cJSON_GetArraySize(task_info_index_array);

    if (TaskInfoIndexArraySize == 0)
    {
        res->task_not_exist = true;
        cJSON_Delete(TaskInfoIndexJson);
        return true;
    }

    std::string id_task_info_query = req->id_task_info;
    std::string id_task_info_erase = id_device + "-task-info-";
    std::regex pattern(id_task_info_erase);
    id_task_info_query = std::regex_replace(id_task_info_query, pattern, "");

    bool task_exist = false;
    bool loop_flag = true;
    std::string task_type_query;
    std::string task_name_query;
    for (size_t i = 0; i < TaskInfoIndexArraySize && loop_flag; i++)
    {
        cJSON* task_info_index_temp = cJSON_GetArrayItem(task_info_index_array, i);
        if (task_info_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Failed to get task_info_index_temp (" << req->id_task_info << ")!");
            cJSON_Delete(TaskInfoIndexJson);
            return false;
        }

        cJSON* id_task_info_temp = cJSON_GetObjectItem(task_info_index_temp, "id_task_info");
        cJSON* task_type_temp = cJSON_GetObjectItem(task_info_index_temp, "task_type");
        cJSON* task_name_temp = cJSON_GetObjectItem(task_info_index_temp, "task_name");
        if (id_task_info_temp == NULL || task_type_temp == NULL || task_name_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Failed to get id_task_info_temp or task_type_temp or task_name_temp (" << req->id_task_info << ")!");
            cJSON_Delete(TaskInfoIndexJson);
            return false;
        }

        if (id_task_info_query == id_task_info_temp->valuestring)
        {
            task_exist = true;
            task_type_query = task_type_temp->valuestring;
            task_name_query = task_name_temp->valuestring;
            loop_flag = false;
            cJSON_Delete(TaskInfoIndexJson);
        }
    }

    if (!task_exist)
    {
        res->task_not_exist = true;
        cJSON_Delete(TaskInfoIndexJson);
        return true;
    }

    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Can't open TaskInstanceIndex.json in local1 (" << req->id_task_info << ")!");
        return false;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Failed to parse TaskInstanceIndexJson (" << req->id_task_info << ")!");
        return false;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Failed to get task_instance_index_array (" << req->id_task_info << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return false;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);

    std::string task_ticket = generateRandomUUID();

    std::string start_time;
    {
        auto now = std::chrono::system_clock::now();
        std::time_t time_now = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm = *std::localtime(&time_now);
        std::ostringstream oss;
        oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
        start_time = oss.str();
    }

    if (TaskInstanceIndexArraySize == 0)
    {
        cJSON* task_instance_index_new = cJSON_CreateObject();
        cJSON_AddStringToObject(task_instance_index_new, "id_task_info", id_task_info_query.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_name", task_name_query.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_type", task_type_query.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_ticket", task_ticket.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_execute_type", "manual_task");
        cJSON_AddStringToObject(task_instance_index_new, "status", "e_started");
        cJSON_AddStringToObject(task_instance_index_new, "start_time", start_time.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "end_time", "");
        cJSON_AddNumberToObject(task_instance_index_new, "execute_time", 0);
        cJSON_AddNumberToObject(task_instance_index_new, "point_count", 0);

        cJSON_AddItemToArray(task_instance_index_array, task_instance_index_new);

        char* temp_str = cJSON_Print(TaskInstanceIndexJson);
        cJSON_Delete(TaskInstanceIndexJson);

        if (!temp_str)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): cJSON_Print(TaskInstanceIndexJson) error (" << req->id_task_info << ")!");
            return false;
        }

        std::string TaskInstanceIndexContentNew = temp_str;
        free(temp_str);

        std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
        if (!TaskInstanceIndexFileWrite)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Can't open TaskInstanceIndex.json in local2 (" << req->id_task_info << ")!");
            return false;
        }

        TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

        if (!TaskInstanceIndexFileWrite.good())
        {
            TaskInstanceIndexFileWrite.close();

            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Error: Failed to write to TaskInstanceIndex.json in local1 (" << req->id_task_info << ")!");
            return false;
        }

        TaskInstanceIndexFileWrite.close();
    }
    else
    {
        std::vector<std::string> TaskTicketArray(TaskInstanceIndexArraySize);
        for (size_t i = 0; i < TaskInstanceIndexArraySize; i++)
        {
            cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
            if (task_instance_index_temp == NULL)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Failed to get task_instance_index_temp (" << req->id_task_info << ")!");
                cJSON_Delete(TaskInstanceIndexJson);
                return false;
            }

            cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
            if (task_ticket_temp == NULL)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Failed to get task_ticket_temp (" << req->id_task_info << ")!");
                cJSON_Delete(TaskInstanceIndexJson);
                return false;
            }

            TaskTicketArray[i] = task_ticket_temp->valuestring;
        }

        bool task_ticket_flag = true;
        while (task_ticket_flag)
        {
            size_t cnt = 0;
            bool loop_flag = true;
            for (size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
            {
                cnt++;
                if (task_ticket == TaskTicketArray[i])
                {
                    task_ticket = generateRandomUUID();
                    loop_flag = false;
                }
            }

            if (cnt == TaskInstanceIndexArraySize)
            {
                task_ticket_flag = false;
            }
        }

        cJSON* task_instance_index_new = cJSON_CreateObject();
        cJSON_AddStringToObject(task_instance_index_new, "id_task_info", id_task_info_query.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_name", task_name_query.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_type", task_type_query.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_ticket", task_ticket.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_execute_type", "manual_task");
        cJSON_AddStringToObject(task_instance_index_new, "status", "e_started");
        cJSON_AddStringToObject(task_instance_index_new, "start_time", start_time.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "end_time", "");
        cJSON_AddNumberToObject(task_instance_index_new, "execute_time", 0);
        cJSON_AddNumberToObject(task_instance_index_new, "point_count", 0);

        cJSON_AddItemToArray(task_instance_index_array, task_instance_index_new);

        char* temp_str = cJSON_Print(TaskInstanceIndexJson);
        cJSON_Delete(TaskInstanceIndexJson);

        if (!temp_str)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): cJSON_Print(TaskInstanceIndexJson) error (" << req->id_task_info << ")!");
            return false;
        }

        std::string TaskInstanceIndexContentNew = temp_str;
        free(temp_str);

        std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
        if (!TaskInstanceIndexFileWrite)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Can't open TaskInstanceIndex.json in local3 (" << req->id_task_info << ")!");
            return false;
        }

        TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

        if (!TaskInstanceIndexFileWrite.good())
        {
            TaskInstanceIndexFileWrite.close();

            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStart(): Error: Failed to write to TaskInstanceIndex.json in local2 (" << req->id_task_info << ")!");
            return false;
        }

        TaskInstanceIndexFileWrite.close();
    }

    res->task_not_exist = false;
    res->execute_success = true;
    res->id_task_info = req->id_task_info;
    res->task_ticket = id_device + "-task-ticket-" + task_ticket;

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_execute_flag == "execute")
        {
            task_execute_infos[i].task_execute_flag = "pause_auto";
            task_execute_infos[i].pause_auto_execute_first_time = true;
            task_execute_infos[i].execute_stop_flag = true;

            std_msgs::msg::Bool ArmCtrlStopMsg;
            ArmCtrlStopMsg.data = true;
            ArmCtrlStopCmd_pub->publish(ArmCtrlStopMsg);

            if (device_type == "e_robot_track")
            {
                std_msgs::msg::Int32 TrackMotorStopCtrlMsg;
                TrackMotorStopCtrlMsg.data = 0;
                TrackMotorStopCtrl_pub->publish(TrackMotorStopCtrlMsg);
            }

            if (device_type == "e_robot_crawler")
            {
                std_msgs::msg::Bool StopCtrlMsg;
                StopCtrlMsg.data = true;
                MotorStopCtrl_pub->publish(StopCtrlMsg);
                LiftCtrlStopCmd_pub->publish(StopCtrlMsg);
            }

            if (device_type == "e_robot_quadrupedal")
            {
                std_msgs::msg::Bool StopCtrlMsg;
                StopCtrlMsg.data = true;
                MotorStopCtrl_pub->publish(StopCtrlMsg);
            }
        }
    }

    task_execute_info task_execute_info_;
    task_execute_info_.task_execute_type = "manual_task";
    task_execute_info_.task_ticket = task_ticket;
    task_execute_info_.id_task_info = id_task_info_query;
    task_execute_info_.task_execute_flag = "execute";
    task_execute_info_.task_type = task_type_query;
    task_execute_info_.task_name = task_name_query;
    task_execute_info_.point_count = 0;
    task_execute_info_.execute_monitor_point_lock = false;
    task_execute_info_.execute_robot_home_lock = false;
    task_execute_info_.execute_stop_flag = false;
    task_execute_infos.push_back(task_execute_info_);

    std::thread task_execute_thread(&Task_Execute::TaskExecute, this, task_ticket);
    task_execute_thread.detach();

    return true;
}

bool Task_Execute::TaskCtrlPause(robot_msgs::srv::InspectTaskCtrlCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr res)
{
    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlPause(): Can't open TaskInstanceIndex.json in local1 (" << req->task_ticket << ")!");
        return false;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlPause(): Failed to parse TaskInstanceIndexJson (" << req->task_ticket << ")!");
        return false;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlPause(): Failed to get task_instance_index_array (" << req->task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return false;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);

    if (TaskInstanceIndexArraySize == 0)
    {
        cJSON_Delete(TaskInstanceIndexJson);
        res->task_instance_not_exist = true;
        return true;
    }

    std::string task_ticket_query = req->task_ticket;
    std::string task_ticket_erase = id_device + "-task-ticket-";
    std::regex pattern(task_ticket_erase);
    task_ticket_query = std::regex_replace(task_ticket_query, pattern, "");

    bool task_ticket_exist = false;
    bool loop_flag = true;
    std::string task_status_query;
    std::string id_task_info_query;
    size_t task_ticket_query_idx;
    for (size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
    {
        cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
        if (task_instance_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlPause(): Failed to get task_instance_index_temp (" << req->task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
        cJSON* task_status_temp = cJSON_GetObjectItem(task_instance_index_temp, "status");
        cJSON* id_task_info_temp = cJSON_GetObjectItem(task_instance_index_temp, "id_task_info");
        if (task_ticket_temp == NULL || task_status_temp == NULL || id_task_info_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlPause(): Failed to get task_ticket_temp or task_status_temp or id_task_info_temp (" << req->task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        if (task_ticket_query == task_ticket_temp->valuestring)
        {
            task_ticket_exist = true;
            task_status_query = task_status_temp->valuestring;
            id_task_info_query = id_task_info_temp->valuestring;
            task_ticket_query_idx = i;
            loop_flag = false;
        }
    }

    if (!task_ticket_exist)
    {
        res->task_instance_not_exist = true;
        cJSON_Delete(TaskInstanceIndexJson);
        return true;
    }

    if (task_status_query == "e_waiting" || task_status_query == "e_stopped" || task_status_query == "e_finished" || task_status_query == "e_outtime" || task_status_query == "e_paused_manual" || task_status_query == "e_paused_auto")
    {
        res->task_instance_not_exist = false;
        res->execute_success = false;
        cJSON_Delete(TaskInstanceIndexJson);
        return true;
    }

    cJSON* task_instance_index_pause = cJSON_GetArrayItem(task_instance_index_array, task_ticket_query_idx);
    cJSON_ReplaceItemInObject(task_instance_index_pause, "status", cJSON_CreateString("e_paused_manual"));

    char* temp_str = cJSON_Print(TaskInstanceIndexJson);
    cJSON_Delete(TaskInstanceIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlPause(): cJSON_Print(TaskInstanceIndexJson) error (" << req->task_ticket << ")!");
        return false;
    }

    std::string TaskInstanceIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInstanceIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlPause(): Can't open TaskInstanceIndex.json in local2 (" << req->task_ticket << ")!");
        return false;
    }

    TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

    if (!TaskInstanceIndexFileWrite.good())
    {
        TaskInstanceIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlPause(): Error: Failed to write to TaskInstanceIndex.json (" << req->task_ticket << ")!");
        return false;
    }

    TaskInstanceIndexFileWrite.close();

    res->task_instance_not_exist = false;
    res->execute_success = true;
    res->id_task_info = id_device + "-task-info-" + id_task_info_query;
    res->task_ticket = req->task_ticket;

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_ticket_query)
        {
            task_execute_infos[i].task_execute_flag = "pause_manual";
            task_execute_infos[i].pause_manual_execute_fitst_time = true;
            task_execute_infos[i].execute_stop_flag = true;
        }
    }

    std_msgs::msg::Bool ArmCtrlStopMsg;
    ArmCtrlStopMsg.data = true;
    ArmCtrlStopCmd_pub->publish(ArmCtrlStopMsg);

    if (device_type == "e_robot_track")
    {
        std_msgs::msg::Int32 TrackMotorStopCtrlMsg;
        TrackMotorStopCtrlMsg.data = 0;
        TrackMotorStopCtrl_pub->publish(TrackMotorStopCtrlMsg);
    }

    if (device_type == "e_robot_crawler")
    {
        std_msgs::msg::Bool StopCtrlMsg;
        StopCtrlMsg.data = true;
        MotorStopCtrl_pub->publish(StopCtrlMsg);
        LiftCtrlStopCmd_pub->publish(StopCtrlMsg);
    }

    if (device_type == "e_robot_quadrupedal")
    {
        std_msgs::msg::Bool StopCtrlMsg;
        StopCtrlMsg.data = true;
        MotorStopCtrl_pub->publish(StopCtrlMsg);
    }

    return true;

}

bool Task_Execute::TaskCtrlRestart(robot_msgs::srv::InspectTaskCtrlCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr res)
{
    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlRestart(): Can't open TaskInstanceIndex.json in local1 (" << req->task_ticket << ")!");
        return false;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlRestart(): Failed to parse TaskInstanceIndexJson (" << req->task_ticket << ")!");
        return false;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlRestart(): Failed to get task_instance_index_array (" << req->task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return false;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);

    if (TaskInstanceIndexArraySize == 0)
    {
        cJSON_Delete(TaskInstanceIndexJson);
        res->task_instance_not_exist = true;
        return true;
    }

    std::string task_ticket_query = req->task_ticket;
    std::string task_ticket_erase = id_device + "-task-ticket-";
    std::regex pattern(task_ticket_erase);
    task_ticket_query = std::regex_replace(task_ticket_query, pattern, "");

    bool task_ticket_exist = false;
    bool loop_flag = true;
    std::string task_status_query;
    std::string id_task_info_query;
    std::string task_type_query;
    std::string task_name_query;
    uint64_t point_count_query;
    size_t task_ticket_query_idx;
    for (size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
    {
        cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
        if (task_instance_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlRestart(): Failed to get task_instance_index_temp (" << req->task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
        cJSON* task_status_temp = cJSON_GetObjectItem(task_instance_index_temp, "status");
        cJSON* id_task_info_temp = cJSON_GetObjectItem(task_instance_index_temp, "id_task_info");
        cJSON* task_type_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_type");
        cJSON* task_name_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_name");
        cJSON* point_count_temp = cJSON_GetObjectItem(task_instance_index_temp, "point_count");
        if (task_ticket_temp == NULL || task_status_temp == NULL || id_task_info_temp == NULL || task_type_temp == NULL || task_name_temp == NULL || point_count_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlRestart(): Failed to get task_ticket_temp or task_status_temp or id_task_info_temp or task_type_temp or task_name_temp or point_count_temp (" << req->task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        if (task_ticket_query == task_ticket_temp->valuestring)
        {
            task_ticket_exist = true;
            task_status_query = task_status_temp->valuestring;
            id_task_info_query = id_task_info_temp->valuestring;
            task_type_query = task_type_temp->valuestring;
            task_name_query = task_name_temp->valuestring;
            point_count_query = point_count_temp->valuedouble;
            task_ticket_query_idx = i;
            loop_flag = false;
        }
    }

    if (!task_ticket_exist)
    {
        res->task_instance_not_exist = true;
        cJSON_Delete(TaskInstanceIndexJson);
        return true;
    }

    if (task_status_query == "e_started" || task_status_query == "e_stopped" || task_status_query == "e_finished" || task_status_query == "e_outtime")
    {
        res->task_instance_not_exist = false;
        res->execute_success = false;
        cJSON_Delete(TaskInstanceIndexJson);
        return true;
    }

    cJSON* task_instance_index_restart = cJSON_GetArrayItem(task_instance_index_array, task_ticket_query_idx);
    cJSON_ReplaceItemInObject(task_instance_index_restart, "status", cJSON_CreateString("e_started"));

    char* temp_str = cJSON_Print(TaskInstanceIndexJson);
    cJSON_Delete(TaskInstanceIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlRestart(): cJSON_Print(TaskInstanceIndexJson) error (" << req->task_ticket << ")!");
        return false;
    }

    std::string TaskInstanceIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInstanceIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlRestart(): Can't open TaskInstanceIndex.json in local2 (" << req->task_ticket << ")!");
        return false;
    }

    TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

    if (!TaskInstanceIndexFileWrite.good())
    {
        TaskInstanceIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlRestart(): Error: Failed to write to TaskInstanceIndex.json (" << req->task_ticket << ")!");
        return false;
    }

    TaskInstanceIndexFileWrite.close();

    res->task_instance_not_exist = false;
    res->execute_success = true;
    res->id_task_info = id_device + "-task-info-" + id_task_info_query;
    res->task_ticket = req->task_ticket;

    bool task_execute_info_exist = false;

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_ticket_query)
        {
            task_execute_infos[i].task_execute_flag = "execute";
            task_execute_infos[i].execute_stop_flag = false;
            task_execute_info_exist = true;
        }
    }

    if (!task_execute_info_exist)
    {
        task_execute_info task_execute_info_;
        task_execute_info_.task_execute_type = "manual_task";
        task_execute_info_.task_ticket = task_ticket_query;
        task_execute_info_.id_task_info = id_task_info_query;
        task_execute_info_.task_execute_flag = "execute";
        task_execute_info_.task_type = task_type_query;
        task_execute_info_.task_name = task_name_query;
        task_execute_info_.point_count = point_count_query;
        task_execute_info_.execute_monitor_point_lock = false;
        task_execute_info_.execute_robot_home_lock = false;
        task_execute_info_.execute_stop_flag = false;
        task_execute_infos.push_back(task_execute_info_);

        std::thread task_execute_thread(&Task_Execute::TaskExecute, this, task_ticket_query);
        task_execute_thread.detach();
    }

    return true;
}

bool Task_Execute::TaskCtrlStop(robot_msgs::srv::InspectTaskCtrlCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskCtrlCmd::Response::SharedPtr res)
{
    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStop(): Can't open TaskInstanceIndex.json in local1 (" << req->task_ticket << ")!");
        return false;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStop(): Failed to parse TaskInstanceIndexJson (" << req->task_ticket << ")!");
        return false;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStop(): Failed to get task_instance_index_array (" << req->task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return false;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);

    if (TaskInstanceIndexArraySize == 0)
    {
        cJSON_Delete(TaskInstanceIndexJson);
        res->task_instance_not_exist = true;
        return true;
    }

    std::string task_ticket_query = req->task_ticket;
    std::string task_ticket_erase = id_device + "-task-ticket-";
    std::regex pattern(task_ticket_erase);
    task_ticket_query = std::regex_replace(task_ticket_query, pattern, "");

    bool task_ticket_exist = false;
    bool loop_flag = true;
    std::string task_status_query;
    std::string id_task_info_query;
    size_t task_ticket_query_idx;
    for (size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
    {
        cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
        if (task_instance_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStop(): Failed to get task_instance_index_temp (" << req->task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
        cJSON* task_status_temp = cJSON_GetObjectItem(task_instance_index_temp, "status");
        cJSON* id_task_info_temp = cJSON_GetObjectItem(task_instance_index_temp, "id_task_info");
        if (task_ticket_temp == NULL || task_status_temp == NULL || id_task_info_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStop(): Failed to get task_ticket_temp or task_status_temp or id_task_info_temp (" << req->task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        if (task_ticket_query == task_ticket_temp->valuestring)
        {
            task_ticket_exist = true;
            task_status_query = task_status_temp->valuestring;
            id_task_info_query = id_task_info_temp->valuestring;
            task_ticket_query_idx = i;
            loop_flag = false;
        }
    }

    if (!task_ticket_exist)
    {
        res->task_instance_not_exist = true;
        cJSON_Delete(TaskInstanceIndexJson);
        return true;
    }

    if (task_status_query == "e_stopped" || task_status_query == "e_finished" || task_status_query == "e_outtime")
    {
        res->task_instance_not_exist = false;
        res->execute_success = false;
        cJSON_Delete(TaskInstanceIndexJson);
        return true;
    }

    cJSON* task_instance_index_stop = cJSON_GetArrayItem(task_instance_index_array, task_ticket_query_idx);
    cJSON_ReplaceItemInObject(task_instance_index_stop, "status", cJSON_CreateString("e_stopped"));

    char* temp_str = cJSON_Print(TaskInstanceIndexJson);
    cJSON_Delete(TaskInstanceIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStop(): cJSON_Print(TaskInstanceIndexJson) error (" << req->task_ticket << ")!");
        return false;
    }

    std::string TaskInstanceIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInstanceIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStop(): Can't open TaskInstanceIndex.json in local2 (" << req->task_ticket << ")!");
        return false;
    }

    TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

    if (!TaskInstanceIndexFileWrite.good())
    {
        TaskInstanceIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskCtrlStop(): Error: Failed to write to TaskInstanceIndex.json (" << req->task_ticket << ")!");
        return false;
    }

    TaskInstanceIndexFileWrite.close();

    res->task_instance_not_exist = false;
    res->execute_success = true;
    res->id_task_info = id_device + "-task-info-" + id_task_info_query;
    res->task_ticket = req->task_ticket;

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_ticket_query)
        {
            task_execute_infos[i].task_execute_flag = "stop";
            task_execute_infos[i].execute_stop_flag = true;
        }
    }

    std_msgs::msg::Bool ArmCtrlStopMsg;
    ArmCtrlStopMsg.data = true;
    ArmCtrlStopCmd_pub->publish(ArmCtrlStopMsg);

    if (device_type == "e_robot_track")
    {
        std_msgs::msg::Int32 TrackMotorStopCtrlMsg;
        TrackMotorStopCtrlMsg.data = 0;
        TrackMotorStopCtrl_pub->publish(TrackMotorStopCtrlMsg);
    }

    if (device_type == "e_robot_crawler")
    {
        std_msgs::msg::Bool StopCtrlMsg;
        StopCtrlMsg.data = true;
        MotorStopCtrl_pub->publish(StopCtrlMsg);
        LiftCtrlStopCmd_pub->publish(StopCtrlMsg);
    }

    if (device_type == "e_robot_quadrupedal")
    {
        std_msgs::msg::Bool StopCtrlMsg;
        StopCtrlMsg.data = true;
        MotorStopCtrl_pub->publish(StopCtrlMsg);
    }

    return true;
}

/********************************************************任务执行*************************************************************/
void Task_Execute::TaskExecute(const std::string task_ticket)
{
    task_execute_info task_execute_info_local;

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_ticket == task_execute_infos[i].task_ticket)
        {
            task_execute_info_local = task_execute_infos[i];
        }
    }

    std::string TaskInfoFileDir = TaskFilesDir + "task_info_files/" + task_execute_info_local.task_type + "/" + task_execute_info_local.id_task_info + ".json";
    std::ifstream TaskInfoFileRead(TaskInfoFileDir.c_str());
    if (!TaskInfoFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecute(): Can't open task info json file (" << task_ticket << ")!");
        return;
    }

    std::string TaskInfoContent((std::istreambuf_iterator<char>(TaskInfoFileRead)), std::istreambuf_iterator<char>());
    TaskInfoFileRead.close();

    cJSON* TaskInfoJson = cJSON_Parse(TaskInfoContent.c_str());
    if (TaskInfoJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecute(): Failed to parse task info json (" << task_ticket << ")!");
        return;
    }

    cJSON* value_monitor_points = cJSON_GetObjectItem(TaskInfoJson, "monitor_points");
    if (value_monitor_points == NULL || !cJSON_IsArray(value_monitor_points))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecute(): Failed to get value_monitor_points from TaskInfoJson (" << task_ticket << ")!");
        cJSON_Delete(TaskInfoJson);
        return;
    }

    std::vector<std::string> task_monitor_points;
    for (size_t i = 0; i < (size_t)cJSON_GetArraySize(value_monitor_points); i++)
    {
        cJSON* monitor_point_temp = cJSON_GetArrayItem(value_monitor_points, i);
        if (monitor_point_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecute(): Failed to get monitor_point_temp from value_monitor_points (" << task_ticket << ")!");
            cJSON_Delete(TaskInfoJson);
            return;
        }

        task_monitor_points.push_back(monitor_point_temp->valuestring);
    }

    cJSON_Delete(TaskInfoJson);

    while (task_execute_info_local.task_execute_flag != "stop" && task_execute_info_local.task_execute_flag != "finish" && task_execute_info_local.task_execute_flag != "outtime")
    {
        if (task_execute_info_local.task_execute_flag == "execute")
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] task_ticket: \"" << task_ticket << "\" wait to execute!");

            bool wait_flag = true;

            for (size_t i = 0; i < 120 && wait_flag; i++)
            {
                for (size_t j = 0; j < task_execute_infos.size(); j++)
                {
                    if (task_ticket == task_execute_infos[j].task_ticket)
                    {
                        if (task_execute_infos[j].task_execute_flag != "execute")
                        {
                            wait_flag = false;

                            RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] task_ticket: \"" << task_ticket << "\" quit waiting due to task_execute_flag changing!");
                        }
                    }
                }

                if ((i + 1) % 10 == 0)
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecute(): Task wait " << i + 1 << " seconds to execute (" << task_ticket << ")!");
                }

                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            if (wait_flag)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] task_ticket: \"" << task_ticket << "\" executing!");

                if (!TaskStartExecuteCheckRobotHome())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecute(): Failed to set robot to home status (" << task_ticket << ")!");
                    continue;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecute(): Robot Homing Successfully (" << task_ticket << ")!");
                }
            }

            uint64_t point_count_local = 0;
            for (size_t i = task_execute_info_local.point_count; i < task_monitor_points.size() && task_execute_info_local.task_execute_flag == "execute"; i++)
            {
                for (size_t j = 0; j < task_execute_infos.size(); j++)
                {
                    if (task_ticket == task_execute_infos[j].task_ticket)
                    {
                        task_execute_info_local = task_execute_infos[j];
                    }
                }

                if (task_execute_info_local.task_execute_flag == "execute")
                {
                    if (point_count_local > 0)
                    {
                        if (i < task_monitor_points.size() - 1)
                        {
                            ExecuteMonitorPoint(task_execute_info_local, task_monitor_points[i], task_monitor_points[i - 1], task_monitor_points[i + 1], point_count_local);
                        }
                        else
                        {
                            ExecuteMonitorPoint(task_execute_info_local, task_monitor_points[i], task_monitor_points[i - 1], task_monitor_points[i], point_count_local);
                        }
                    }
                    else
                    {
                        if (i < task_monitor_points.size() - 1)
                        {
                            ExecuteMonitorPoint(task_execute_info_local, task_monitor_points[i], task_monitor_points[i], task_monitor_points[i + 1], point_count_local);
                        }
                        else
                        {
                            ExecuteMonitorPoint(task_execute_info_local, task_monitor_points[i], task_monitor_points[i], task_monitor_points[i], point_count_local);
                        }
                    }

                    if (i == (task_monitor_points.size() - 1))
                    {
                        for (size_t j = 0; j < task_execute_infos.size(); j++)
                        {
                            if (task_ticket == task_execute_infos[j].task_ticket)
                            {
                                task_execute_infos[j].task_execute_flag = "finish";
                            }
                        }
                    }

                    point_count_local++;
                }
            }
        }

        for (size_t i = 0; i < task_execute_infos.size(); i++)
        {
            if (task_ticket == task_execute_infos[i].task_ticket)
            {
                task_execute_info_local = task_execute_infos[i];
            }
        }

        // task pause_auto
        if (task_execute_info_local.task_execute_flag == "pause_auto")
        {
            if (task_execute_info_local.pause_auto_execute_first_time)
            {
                if (ExecuteRobotHomeStatus(task_execute_info_local))
                {
                    if (TaskPauseAuto(task_ticket))
                    {
                        for (size_t i = 0; i < task_execute_infos.size(); i++)
                        {
                            if (task_ticket == task_execute_infos[i].task_ticket)
                            {
                                task_execute_infos[i].pause_auto_execute_first_time = false;
                            }
                        }
                    }
                }
            }
        }

        // task pause_manual
        if (task_execute_info_local.task_execute_flag == "pause_manual")
        {
            if (task_execute_info_local.pause_manual_execute_fitst_time)
            {
                if (ExecuteRobotHomeStatus(task_execute_info_local))
                {
                    for (size_t i = 0; i < task_execute_infos.size(); i++)
                    {
                        if (task_ticket == task_execute_infos[i].task_ticket)
                        {
                            task_execute_infos[i].pause_manual_execute_fitst_time = false;
                        }
                    }
                }
            }
        }

        // task stop
        if (task_execute_info_local.task_execute_flag == "stop")
        {
            ExecuteRobotHomeStatus(task_execute_info_local);

            TaskStop(task_ticket);

            size_t task_execute_info_local_idx;

            for (size_t i = 0; i < task_execute_infos.size(); i++)
            {
                if (task_execute_info_local.task_ticket == task_execute_infos[i].task_ticket)
                {
                    task_execute_info_local_idx = i;
                }
            }

            task_execute_infos.erase(task_execute_infos.begin() + task_execute_info_local_idx);
        }

        // task finished
        if (task_execute_info_local.point_count == task_monitor_points.size() || task_execute_info_local.task_execute_flag == "finish")
        {
            ExecuteRobotHomeStatus(task_execute_info_local);

            TaskFinish(task_execute_info_local);

            size_t task_execute_info_local_idx;

            for (size_t i = 0; i < task_execute_infos.size(); i++)
            {
                if (task_execute_info_local.task_ticket == task_execute_infos[i].task_ticket)
                {
                    task_execute_info_local_idx = i;
                }
            }

            task_execute_infos.erase(task_execute_infos.begin() + task_execute_info_local_idx);

            task_execute_info_local.task_execute_flag = "finish";

            bool other_task_pause_or_wait = false;
            std::string task_ticket_start_auto;
            for (size_t i = 0; i < task_execute_infos.size(); i++)
            {
                if (task_execute_infos[i].task_execute_flag == "pause_auto" || task_execute_infos[i].task_execute_flag == "waiting" || task_execute_infos[i].task_execute_flag == "execute"/* waiting状态的任务刚跳转为execute状态 */)
                {
                    other_task_pause_or_wait = true;

                    if (task_execute_infos[i].task_execute_flag == "pause_auto")
                    {
                        task_execute_infos[i].task_execute_flag = "execute";
                        task_execute_infos[i].execute_stop_flag = false;
                        task_ticket_start_auto = task_execute_infos[i].task_ticket;
                        break;
                    }
                }
            }

            if (!task_ticket_start_auto.empty())
            {
                TaskStartAuto(task_ticket_start_auto);
            }

            if (!other_task_pause_or_wait)
            {
                std_msgs::msg::Bool TaskFinishAndExecuteBatteryChargeMsg;
                TaskFinishAndExecuteBatteryChargeMsg.data = true;
                TaskFinishAndExecuteBatteryCharge_pub->publish(TaskFinishAndExecuteBatteryChargeMsg);
            }
        }

        // task waiting
        if (task_execute_info_local.task_execute_flag == "waiting")
        {
            // 先检查是否outtime
            size_t this_plan_task_idx;
            size_t next_plan_task_idx = 0;
            for (size_t i = 0; i < task_execute_infos.size(); i++)
            {
                if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
                {
                    this_plan_task_idx = i;
                }
            }

            for (size_t i = this_plan_task_idx; i < task_execute_infos.size(); i++)
            {
                if (i > this_plan_task_idx && task_execute_infos[i].task_execute_type == "plan_task" && task_execute_infos[i].id_task_plan == task_execute_info_local.id_task_plan)
                {
                    next_plan_task_idx = i;
                }
            }

            // 同一任务计划下一次任务已被触发，将当前次任务设置为outtime
            if (next_plan_task_idx > this_plan_task_idx)
            {
                TaskOuttime(task_ticket);

                size_t task_execute_info_local_idx;

                for (size_t i = 0; i < task_execute_infos.size(); i++)
                {
                    if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
                    {
                        task_execute_info_local_idx = i;
                    }
                }

                task_execute_infos.erase(task_execute_infos.begin() + task_execute_info_local_idx);

                task_execute_info_local.task_execute_flag = "outtime";

            }
            else
            {
                bool other_task_execute_or_pause = false;
                for (size_t i = 0; i < task_execute_infos.size(); i++)
                {
                    if (task_execute_infos[i].task_execute_flag == "execute" || task_execute_infos[i].task_execute_flag == "pause_auto" || task_execute_infos[i].task_execute_flag == "pause_manual")
                    {
                        other_task_execute_or_pause = true;
                        break;
                    }
                }

                if (!other_task_execute_or_pause)
                {
                    robot_msgs::srv::CtrlModeQuery::Request::SharedPtr request;
                    auto res_callback = [this, task_execute_info_local](rclcpp::Client<robot_msgs::srv::CtrlModeQuery>::SharedFuture future)
                    {
                        auto response = future.get();
                        if(response->runtime_ctrl_mode == 0)
                        {
                            for (size_t i = 0; i < task_execute_infos.size(); i++)
                            {
                                if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
                                {
                                    task_execute_infos[i].task_execute_flag = "execute";
                                    PlanTaskStartAuto(task_execute_infos[i]);
                                    break;
                                }
                            }    
                        }   
                    };
                    
                    auto future = CtrlModeQuery_client->async_send_request(request, res_callback);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] task_ticket: \"" << task_ticket << "\" TaskExecute program exit!");
}

void Task_Execute::ExecuteMonitorPoint(const task_execute_info task_execute_info_local, const std::string monitor_point, const std::string monitor_point_last, const std::string monitor_point_next, uint64_t point_count_local)
{
    bool execute_robot_home_lock_local = true;
    while (execute_robot_home_lock_local)
    {
        bool execute_robot_home_locks = false;
        for (size_t i = 0; i < task_execute_infos.size(); i++)
        {
            execute_robot_home_locks = execute_robot_home_locks || task_execute_infos[i].execute_robot_home_lock;
        }

        execute_robot_home_lock_local = execute_robot_home_locks;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
        {
            task_execute_infos[i].execute_monitor_point_lock = true;
            task_execute_infos[i].arm_home = false;
        }
    }

    std::string MonitorPointFileDir = TaskFilesDir + "monitor_points_files/" + monitor_point + ".json";
    std::string MonitorPointLastFileDir = TaskFilesDir + "monitor_points_files/" + monitor_point_last + ".json";
    std::string MonitorPointNextFileDir = TaskFilesDir + "monitor_points_files/" + monitor_point_next + ".json";

    std::ifstream MonitorPointFileRead(MonitorPointFileDir.c_str());
    if (!MonitorPointFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Can't open monitor_point(" << monitor_point << ") json file (" << task_execute_info_local.task_ticket << ")!");
        return;
    }

    std::string MonitorPointContent((std::istreambuf_iterator<char>(MonitorPointFileRead)), std::istreambuf_iterator<char>());
    MonitorPointFileRead.close();

    std::ifstream MonitorPointLastFileRead(MonitorPointLastFileDir.c_str());
    if (!MonitorPointLastFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Can't open monitor_point_last(" << monitor_point_last << ") json file (" << task_execute_info_local.task_ticket << ")!");
        return;
    }
    std::string MonitorPointLastContent((std::istreambuf_iterator<char>(MonitorPointLastFileRead)), std::istreambuf_iterator<char>());
    MonitorPointLastFileRead.close();

    std::ifstream MonitorPointNextFileRead(MonitorPointNextFileDir.c_str());
    if (!MonitorPointNextFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Can't open monitor_point_next(" << monitor_point_next << ") json file (" << task_execute_info_local.task_ticket << ")!");
        return;
    }
    std::string MonitorPointNextContent((std::istreambuf_iterator<char>(MonitorPointNextFileRead)), std::istreambuf_iterator<char>());
    MonitorPointNextFileRead.close();

    cJSON* value_monitor_point = cJSON_Parse(MonitorPointContent.c_str());
    if (value_monitor_point == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to parse monitor point json (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
        return;
    }

    cJSON* value_monitor_point_pos = cJSON_GetObjectItem(value_monitor_point, "monitor_point_pos");
    cJSON* value_track_id = cJSON_GetObjectItem(value_monitor_point, "track_id");
    cJSON* value_track_pos = cJSON_GetObjectItem(value_monitor_point, "track_pos");
    cJSON* value_lift_pos;
    if (device_type == "e_robot_crawler")
    {
        value_lift_pos = cJSON_GetObjectItem(value_monitor_point, "lift_pos");
    }
    cJSON* value_arm_pos = cJSON_GetObjectItem(value_monitor_point, "arm_pos");

    if (value_monitor_point_pos == NULL || value_track_id == NULL || value_track_pos == NULL || value_arm_pos == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to get value_monitor_point_pos or value_track_id or value_track_pos or value_arm_pos from value_monitor_point (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
        cJSON_Delete(value_monitor_point);
        return;
    }

    if (device_type == "e_robot_crawler")
    {
        if (value_lift_pos == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to get value_lift_pos from value_monitor_point (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
            cJSON_Delete(value_monitor_point);
            return;
        }
    }

    std::string monitor_point_pos = value_monitor_point_pos->valuestring;
    std::string monitor_point_track_id = value_track_id->valuestring;
    double monitor_point_track_pos = value_track_pos->valuedouble;
    std::string monitor_point_arm_pos = value_arm_pos->valuestring;

    double monitor_point_lift_pos;
    if (device_type == "e_robot_crawler")
    {
        monitor_point_lift_pos = value_lift_pos->valuedouble;
    }

    cJSON_Delete(value_monitor_point);

    cJSON* value_monitor_point_last = cJSON_Parse(MonitorPointLastContent.c_str());
    if (value_monitor_point_last == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to parse monitor point last json (" << task_execute_info_local.task_ticket << " - " << monitor_point_last << ")!");
        return;
    }

    cJSON* value_monitor_point_last_pos = cJSON_GetObjectItem(value_monitor_point_last, "monitor_point_pos");

    if (value_monitor_point_last_pos == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to get value_monitor_point_last_pos from value_monitor_point_last (" << task_execute_info_local.task_ticket << " - " << monitor_point_last << ")!");
        cJSON_Delete(value_monitor_point_last);
        return;
    }

    std::string monitor_point_last_pos = value_monitor_point_last_pos->valuestring;

    cJSON_Delete(value_monitor_point_last);

    cJSON* value_monitor_point_next = cJSON_Parse(MonitorPointNextContent.c_str());
    if (value_monitor_point_next == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to parse monitor point next json (" << task_execute_info_local.task_ticket << " - " << monitor_point_next << ")!");
        return;
    }

    cJSON* value_monitor_point_next_pos = cJSON_GetObjectItem(value_monitor_point_next, "monitor_point_pos");

    if (value_monitor_point_next_pos == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to get value_monitor_point_next_pos from value_monitor_point_next (" << task_execute_info_local.task_ticket << " - " << monitor_point_next << ")!");
        cJSON_Delete(value_monitor_point_next);
        return;
    }

    std::string monitor_point_next_pos = value_monitor_point_next_pos->valuestring;

    cJSON_Delete(value_monitor_point_next);

    bool execute_stop_flag_local;
    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
        {
            execute_stop_flag_local = task_execute_infos[i].execute_stop_flag;
        }
    }

    bool robot_move_flag = true;
    if (execute_stop_flag_local)
    {
        robot_move_flag = false;
    }

    if (point_count_local == 0 || (monitor_point_pos != monitor_point_last_pos))
    {
        robot_msgs::srv::MotorCtrltoPosSrv::Request::SharedPtr MC2P_request;
        MC2P_request->track_id = monitor_point_track_id;
        MC2P_request->track_pos = monitor_point_track_pos;
        MC2P_request->run_speed = -10;



        if (robot_move_flag)
        {
            auto MC2P_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::MotorCtrltoPosSrv>::SharedFuture>();
            auto MC2P_res_callback = [this, MC2P_request, MC2P_shared_future_ptr, &robot_move_flag, task_execute_info_local, monitor_point, monitor_point_pos](rclcpp::Client<robot_msgs::srv::MotorCtrltoPosSrv>::SharedFuture future)
            {
                *MC2P_shared_future_ptr = future;
                auto response = future.get();
                if (!response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Robot nav process cancelled (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                    robot_move_flag = false;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Robot successfully move to " << monitor_point_pos << "!");
                }
            };
            
            auto MC2P_future = MotorCtrltoPosSrv_client->async_send_request(MC2P_request, MC2P_res_callback);
            *MC2P_shared_future_ptr = MC2P_future.future;
            
            auto MC2P_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                [this, MC2P_shared_future_ptr, &robot_move_flag, task_execute_info_local, monitor_point]()
                {
                    if (MC2P_shared_future_ptr->valid() &&
                        MC2P_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Robot failed to start nav process (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        robot_move_flag = false;
                    }
                });
        }
    }

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
        {
            execute_stop_flag_local = task_execute_infos[i].execute_stop_flag;
        }
    }

    bool arm_init_flag = true;
    if (!robot_move_flag)
    {
        arm_init_flag = false;
    }

    if (execute_stop_flag_local)
    {
        arm_init_flag = false;
    }

    if (point_count_local == 0 || (monitor_point_pos != monitor_point_last_pos))
    {
        if (device_type == "e_robot_crawler")
        {
            robot_msgs::srv::LiftCtrlSrv::Request::SharedPtr lift_request;
            robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;

            if (arm_init_flag)
            {
                lift_request->position = 0.0;
                auto lift_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture>();
                auto lift_res_callback = [this, lift_request, lift_shared_future_ptr, &arm_init_flag, task_execute_info_local, monitor_point, monitor_point_lift_pos](rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture future)
                {
                    *lift_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift failed to move to target pos(" << lift_request->position << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_init_flag = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift successfully move to target pos: " << monitor_point_lift_pos << "!");
                    }
                };
                
                auto lift_future = LiftCtrlSrv_client->async_send_request(lift_request, lift_res_callback);
                *lift_shared_future_ptr = lift_future.future;
                
                auto lift_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, lift_shared_future_ptr, lift_request, &arm_init_flag, task_execute_info_local, monitor_point]()
                    {
                        if (lift_shared_future_ptr->valid() &&
                            lift_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift target pos(" << lift_request->position << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                            arm_init_flag = false;
                        }
                    });
            }

            if (arm_init_flag)
            {
                arm_request->pos_key = "posinit";
                auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
                auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_init_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
                {
                    *arm_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_init_flag = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                    }
                };
                
                auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
                *arm_shared_future_ptr = arm_future.future;
                
                auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, arm_shared_future_ptr, arm_request, &arm_init_flag, task_execute_info_local, monitor_point]()
                    {
                        if (arm_shared_future_ptr->valid() &&
                            arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                            arm_init_flag = false;
                        }
                    });
            }

            if (arm_init_flag)
            {
                arm_request->pos_key = "posleftend";
                auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
                auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_init_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
                {
                    *arm_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_init_flag = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                    }
                };
                
                auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
                *arm_shared_future_ptr = arm_future.future;
                
                auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, arm_shared_future_ptr, arm_request, &arm_init_flag, task_execute_info_local, monitor_point]()
                    {
                        if (arm_shared_future_ptr->valid() &&
                            arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                            arm_init_flag = false;
                        }
                    });
            }
        }
        else if (device_type == "e_robot_track")
        {
            robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;
            if (arm_init_flag)
            {
                arm_request->pos_key = "posstart";
                auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
                auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_init_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
                {
                    *arm_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_init_flag = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                    }
                };
                
                auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
                *arm_shared_future_ptr = arm_future.future;
                
                auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, arm_shared_future_ptr, arm_request, &arm_init_flag, task_execute_info_local, monitor_point]()
                    {
                        if (arm_shared_future_ptr->valid() &&
                            arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                            arm_init_flag = false;
                        }
                    });
            }
        }
    }

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
        {
            execute_stop_flag_local = task_execute_infos[i].execute_stop_flag;
        }
    }

    bool arm_move_flag = true;
    if (!arm_init_flag)
    {
        arm_move_flag = false;
    }

    if (execute_stop_flag_local)
    {
        arm_move_flag = false;
    }

    if (device_type == "e_robot_crawler")
    {
        robot_msgs::srv::LiftCtrlSrv::Request::SharedPtr lift_request;
        robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;

        if (monitor_point_arm_pos == "pos2" || monitor_point_arm_pos == "pos11")
        {
            if (arm_move_flag)
            {
                arm_request->pos_key = monitor_point_arm_pos;
                auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
                auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
                {
                    *arm_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                    }
                };
                
                auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
                *arm_shared_future_ptr = arm_future.future;
                
                auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, arm_shared_future_ptr, arm_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                    {
                        if (arm_shared_future_ptr->valid() &&
                            arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                            arm_move_flag = false;
                        }
                    });
            }

            if (arm_move_flag)
            {
                lift_request->position = monitor_point_lift_pos;
                auto lift_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture>();
                auto lift_res_callback = [this, lift_request, lift_shared_future_ptr, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_lift_pos](rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture future)
                {
                    *lift_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift failed to move to target pos(" << lift_request->position << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift successfully move to target pos: " << monitor_point_lift_pos << "!");
                    }
                };
                
                auto lift_future = LiftCtrlSrv_client->async_send_request(lift_request, lift_res_callback);
                *lift_shared_future_ptr = lift_future.future;
                
                auto lift_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, lift_shared_future_ptr, lift_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                    {
                        if (lift_shared_future_ptr->valid() &&
                            lift_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift target pos(" << lift_request->position << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                            arm_move_flag = false;
                        }
                    });
            }
        }
        else
        {
            if (arm_move_flag)
            {
                lift_request->position = monitor_point_lift_pos;
                auto lift_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture>();
                auto lift_res_callback = [this, lift_request, lift_shared_future_ptr, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_lift_pos](rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture future)
                {
                    *lift_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift failed to move to target pos(" << lift_request->position << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift successfully move to target pos: " << monitor_point_lift_pos << "!");
                    }
                };
                
                auto lift_future = LiftCtrlSrv_client->async_send_request(lift_request, lift_res_callback);
                *lift_shared_future_ptr = lift_future.future;
                
                auto lift_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, lift_shared_future_ptr, lift_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                    {
                        if (lift_shared_future_ptr->valid() &&
                            lift_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift target pos(" << lift_request->position << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                            arm_move_flag = false;
                        }
                    });
            }

            if (arm_move_flag)
            {
                arm_request->pos_key = monitor_point_arm_pos;
                auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
                auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
                {
                    *arm_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                    }
                };
                
                auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
                *arm_shared_future_ptr = arm_future.future;
                
                auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, arm_shared_future_ptr, arm_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                    {
                        if (arm_shared_future_ptr->valid() &&
                            arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                            arm_move_flag = false;
                        }
                    });
            }
        }
    }
    else if (device_type == "e_robot_track")
    {
        robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;

        if (arm_move_flag)
        {
            arm_request->pos_key = monitor_point_arm_pos;
            auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
            auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
            {
                *arm_shared_future_ptr = future;
                auto response = future.get();
                if (!response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                    arm_move_flag = false;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                }
            };
            
            auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
            *arm_shared_future_ptr = arm_future.future;
            
            auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                [this, arm_shared_future_ptr, arm_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                {
                    if (arm_shared_future_ptr->valid() &&
                        arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                });
        }
    }
    else if (device_type == "e_robot_quadrupedal")
    {
        robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;

        if (arm_move_flag)
        {
            arm_request->pos_key = monitor_point_arm_pos;
            auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
            auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
            {
                *arm_shared_future_ptr = future;
                auto response = future.get();
                if (!response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                    arm_move_flag = false;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                }
            };
            
            auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
            *arm_shared_future_ptr = arm_future.future;
            
            auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                [this, arm_shared_future_ptr, arm_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                {
                    if (arm_shared_future_ptr->valid() &&
                        arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                });
        }
    }


    for(size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
        {
            execute_stop_flag_local = task_execute_infos[i].execute_stop_flag;
        }
    }

    bool TaskDataCollect_flag = true;
    if (!arm_move_flag)
    {
        TaskDataCollect_flag = false;
    }

    if (execute_stop_flag_local)
    {
        TaskDataCollect_flag = false;
    }

    if (TaskDataCollect_flag)
    {
        bool eLightOn_flag;
        if (device_type == "e_robot_track")
        {
            eLightOn_flag = true;
        }
        else if (device_type == "e_robot_crawler")
        {
            eLightOn_flag = false;
        }
        else if (device_type == "e_robot_quadrupedal")
        {
            eLightOn_flag = false;
        }
        else
        {
            eLightOn_flag = false;
        }

        if (eLightOn_flag)
        {
            robot_msgs::srv::ExtendDevCtrlSrv::Request::SharedPtr EDC_request;
            EDC_request->extend_dev_ctrl.device_type = "e_light";
            EDC_request->extend_dev_ctrl.command = "e_on";

            /*if (!ExtendDevCtrlSrv_client.call(ExtendDevCtrlSrv))*/
            /*{*/
            /*RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): ExtendDevCtrlSrv(turn on light) return false (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");*/
            /*TaskDataCollect_flag = false;*/
            /*}*/
            /*else*/
            /*{*/
            /*if (!ExtendDevCtrlSrv.response.execute_success.data)*/
            /*{*/
            /*RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to turn on ExtendDev(e_light) (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");*/
            /*TaskDataCollect_flag = false;*/
            /*}*/
            /*else*/
            /*{*/
            /*RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Successfully turn on ExtendDev(e_light): " << monitor_point << "!");*/
            /*}*/
            /*}*/
        }
    }

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
        {
            execute_stop_flag_local = task_execute_infos[i].execute_stop_flag;
        }
    }

    if (execute_stop_flag_local)
    {
        TaskDataCollect_flag = false;
    }

    if (TaskDataCollect_flag)
    {
        bool eWiperOn_flag;
        if (device_type == "e_robot_track")
        {
            eWiperOn_flag = true;
        }
        else if (device_type == "e_robot_crawler")
        {
            if (id_device == "robot-crawler-0002" || id_device == "robot-crawler-0003")
            {
                eWiperOn_flag = true;
            }
            else
            {
                eWiperOn_flag = false;
            }
        }
        else
        {
            eWiperOn_flag = false;
        }

        if (eWiperOn_flag)
        {
            robot_msgs::srv::ExtendDevCtrlSrv::Request::SharedPtr EDC_request;
            EDC_request->extend_dev_ctrl.device_type = "e_wiper";
            EDC_request->extend_dev_ctrl.command = "e_on";

            /*if (!ExtendDevCtrlSrv_client.call(ExtendDevCtrlSrv))*/
            /*{*/
            /*RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): ExtendDevCtrlSrv(turn on wiper) return false (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");*/
            /*TaskDataCollect_flag = false;*/
            /*}*/
            /*else*/
            /*{*/
            /*if (!ExtendDevCtrlSrv.response.execute_success.data)*/
            /*{*/
            /*RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to turn on ExtendDev(e_wiper) (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");*/
            /*TaskDataCollect_flag = false;*/
            /*}*/
            /*else*/
            /*{*/
            /*RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Successfully turn on ExtendDev(e_wiper): " << monitor_point << "!");*/
            /*}*/
            /*}*/
        }
    }

    if (TaskDataCollect_flag)
    {
        std::this_thread::sleep_for(std::chrono::seconds(WiperWorkDuration));
    }

    bool eWiperOff_flag;
    if (device_type == "e_robot_track")
    {
        eWiperOff_flag = true;
    }
    else if (device_type == "e_robot_crawler")
    {
        if (id_device == "robot-crawler-0002" || id_device == "robot-crawler-0003")
        {
            eWiperOff_flag = true;
        }
        else
        {
            eWiperOff_flag = false;
        }
    }
    else
    {
        eWiperOff_flag = false;
    }

    if (eWiperOff_flag)
    {
        robot_msgs::srv::ExtendDevCtrlSrv::Request::SharedPtr EDC_request;
        EDC_request->extend_dev_ctrl.device_type = "e_wiper";
        EDC_request->extend_dev_ctrl.command = "e_off";

        /*if (!ExtendDevCtrlSrv_client.call(ExtendDevCtrlSrv))*/
        /*{*/
        /*RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): ExtendDevCtrlSrv(turn off wiper) return false (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");*/
        /*TaskDataCollect_flag = false;*/
        /*}*/
        /*else*/
        /*{*/
        /*if (!ExtendDevCtrlSrv.response.execute_success.data)*/
        /*{*/
        /*RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to turn off ExtendDev(e_wiper) (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");*/
        /*TaskDataCollect_flag = false;*/
        /*}*/
        /*else*/
        /*{*/
        /*RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Successfully turn off ExtendDev(e_wiper): " << monitor_point << "!");*/
        /*}*/
        /*}*/
    }

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
        {
            execute_stop_flag_local = task_execute_infos[i].execute_stop_flag;
        }
    }

    if (execute_stop_flag_local)
    {
        TaskDataCollect_flag = false;
    }

    if (TaskDataCollect_flag)
    {
        robot_msgs::srv::TaskDataCollectSrv::Request::SharedPtr TDC_request;
        TDC_request->id_task_info = task_execute_info_local.id_task_info;
        TDC_request->task_ticket = task_execute_info_local.task_ticket;
        TDC_request->monitor_points = monitor_point;

        std::this_thread::sleep_for(std::chrono::seconds(ImgDataCollectWaitTime));

        auto TDC_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::TaskDataCollectSrv>::SharedFuture>();
        auto TDC_res_callback = [this, TDC_request, TDC_shared_future_ptr, &TaskDataCollect_flag, task_execute_info_local, monitor_point](rclcpp::Client<robot_msgs::srv::TaskDataCollectSrv>::SharedFuture future)
        {
            *TDC_shared_future_ptr = future;
            auto response = future.get();
            if (!response->execute_success)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): VisCam or IrCam or V3dCam failed to collect data (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                TaskDataCollect_flag = false;
            }
            else
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Successfully collect data in monitor_point: " << monitor_point << "!");
            }
        };
        
        auto TDC_future = TaskDataCollectSrv_client->async_send_request(TDC_request, TDC_res_callback);
        *TDC_shared_future_ptr = TDC_future.future;
        
        auto TDC_timer = this->create_wall_timer(std::chrono::milliseconds(500),
            [this, TDC_shared_future_ptr, &TaskDataCollect_flag, task_execute_info_local, monitor_point]()
            {
                if (TDC_shared_future_ptr->valid() &&
                    TDC_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): VisCam or IrCam or V3dCam failed to init (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                    TaskDataCollect_flag = false;
                }
            });
    }

    bool eLightOff_flag;
    if (device_type == "e_robot_track")
    {
        eLightOff_flag = true;
    }
    else if (device_type == "e_robot_crawler")
    {
        eLightOff_flag = false;
    }
    else if (device_type == "e_robot_quadrupedal")
    {
        eLightOff_flag = false;
    }
    else
    {
        eLightOff_flag = false;
    }

    if (eLightOff_flag)
    {
        robot_msgs::srv::ExtendDevCtrlSrv::Request::SharedPtr EDC_request;
        EDC_request->extend_dev_ctrl.device_type = "e_light";
        EDC_request->extend_dev_ctrl.command = "e_off";

        /*if (!ExtendDevCtrlSrv_client.call(ExtendDevCtrlSrv))*/
        /*{*/
        /*RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): ExtendDevCtrlSrv(turn off light) return false (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");*/
        /*}*/
        /*else*/
        /*{*/
        /*if (!ExtendDevCtrlSrv.response.execute_success.data)*/
        /*{*/
        /*RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to turn off ExtendDev(e_light) (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");*/
        /*}*/
        /*else*/
        /*{*/
        /*RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Successfully turn off ExtendDev(e_light): " << monitor_point << "!");*/
        /*}*/
        /*}*/
    }

    bool arm_home_flag = true;
    bool arm_home_flag_local = true;
    if (device_type == "e_robot_crawler")
    {
        robot_msgs::srv::LiftCtrlSrv::Request::SharedPtr lift_request;
        robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;
        
        if (monitor_point_arm_pos == "pos12" || monitor_point_pos != monitor_point_next_pos)
        {
            arm_request->pos_key = "posrightend";
            auto arm_shared_future_ptr1 = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
            auto arm_res_callback1 = [this, arm_request, arm_shared_future_ptr1, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
            {
                *arm_shared_future_ptr1 = future;
                auto response = future.get();
                if (!response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                    arm_move_flag = false;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                }
            };
            
            auto arm_future1 = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback1);
            *arm_shared_future_ptr1 = arm_future1.future;
            
            auto arm_timer1 = this->create_wall_timer(std::chrono::milliseconds(500),
                [this, arm_shared_future_ptr1, arm_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                {
                    if (arm_shared_future_ptr1->valid() &&
                        arm_shared_future_ptr1->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                });

            arm_request->pos_key = "posinit";
            auto arm_shared_future_ptr2 = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
            auto arm_res_callback2 = [this, arm_request, arm_shared_future_ptr2, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
            {
                *arm_shared_future_ptr2 = future;
                auto response = future.get();
                if (!response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                    arm_move_flag = false;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                }
            };
            
            auto arm_future2 = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback2);
            *arm_shared_future_ptr2 = arm_future2.future;
            
            auto arm_timer2 = this->create_wall_timer(std::chrono::milliseconds(500),
                [this, arm_shared_future_ptr2, arm_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                {
                    if (arm_shared_future_ptr2->valid() &&
                        arm_shared_future_ptr2->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                });

            arm_request->pos_key = "poshome";
            auto arm_shared_future_ptr3 = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
            auto arm_res_callback3 = [this, arm_request, arm_shared_future_ptr3, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
            {
                *arm_shared_future_ptr3 = future;
                auto response = future.get();
                if (!response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                    arm_move_flag = false;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                }
            };
            
            auto arm_future3 = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback3);
            *arm_shared_future_ptr3 = arm_future3.future;
            
            auto arm_timer3 = this->create_wall_timer(std::chrono::milliseconds(500),
                [this, arm_shared_future_ptr3, arm_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                {
                    if (arm_shared_future_ptr3->valid() &&
                        arm_shared_future_ptr3->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                });

            if (arm_home_flag)
            {
                lift_request->position = 0.0;
                auto lift_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture>();
                auto lift_res_callback = [this, lift_request, lift_shared_future_ptr, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_lift_pos](rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture future)
                {
                    *lift_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift failed to move to target pos(" << lift_request->position << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift successfully move to target pos: " << monitor_point_lift_pos << "!");
                    }
                };
                
                auto lift_future = LiftCtrlSrv_client->async_send_request(lift_request, lift_res_callback);
                *lift_shared_future_ptr = lift_future.future;
                
                auto lift_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, lift_shared_future_ptr, lift_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                    {
                        if (lift_shared_future_ptr->valid() &&
                            lift_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Lift target pos(" << lift_request->position << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                            arm_move_flag = false;
                        }
                    });
            }

            for (size_t i = 0; i < task_execute_infos.size(); i++)
            {
                if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
                {
                    task_execute_infos[i].arm_home = arm_home_flag_local;
                }
            }
        }
    }
    else if (device_type == "e_robot_track")
    {
        if ((TrackArmMode == "debug" && monitor_point_arm_pos == "pos12") || (TrackArmMode == "release" && monitor_point_arm_pos == "pos8") || monitor_point_pos != monitor_point_next_pos)
        {
            robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;
            if (arm_home_flag)
            {
                arm_request->pos_key = "poshome";
                auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
                auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
                {
                    *arm_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                    }
                };
                
                auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
                *arm_shared_future_ptr = arm_future.future;
                
                auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, arm_shared_future_ptr, arm_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                    {
                        if (arm_shared_future_ptr->valid() &&
                            arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                            arm_move_flag = false;
                        }
                    });
            }

            for (size_t i = 0; i < task_execute_infos.size(); i++)
            {
                if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
                {
                    task_execute_infos[i].arm_home = arm_home_flag_local;
                }
            }
        }
    }
    else if (device_type == "e_robot_quadrupedal")
    {
        if (monitor_point_arm_pos == "pos12" || monitor_point_pos != monitor_point_next_pos)
        {
            robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;
            if (arm_home_flag)
            {
                arm_request->pos_key = "poshome";
                auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
                auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_move_flag, task_execute_info_local, monitor_point, monitor_point_arm_pos](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
                {
                    *arm_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                        arm_move_flag = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm successfully move to target pos: " << monitor_point_arm_pos << "!");
                    }
                };
                
                auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
                *arm_shared_future_ptr = arm_future.future;
                
                auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, arm_shared_future_ptr, arm_request, &arm_move_flag, task_execute_info_local, monitor_point]()
                    {
                        if (arm_shared_future_ptr->valid() &&
                            arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                            arm_move_flag = false;
                        }
                    });
            }

            for (size_t i = 0; i < task_execute_infos.size(); i++)
            {
                if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
                {
                    task_execute_infos[i].arm_home = arm_home_flag_local;
                }
            }
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] monitor_point: " << monitor_point);
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] monitor_point_last: " << monitor_point_last);
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] monitor_point_next: " << monitor_point_next);
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] -------------------------------------------");
    //std::this_thread::sleep_for(std::chrono::seconds(5));

    uint64_t task_execute_point_count;

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
        {
            if (TaskDataCollect_flag)
            {
                task_execute_infos[i].point_count++;
            }
            task_execute_infos[i].execute_monitor_point_lock = false;
            task_execute_point_count = task_execute_infos[i].point_count;
        }
    }

    if (TaskDataCollect_flag)
    {
        std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
        std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
        if (!TaskInstanceIndexFileRead)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Can't open TaskInstanceIndex.json in local1 (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
            return;
        }

        std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
        TaskInstanceIndexFileRead.close();

        cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
        if (TaskInstanceIndexJson == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to parse TaskInstanceIndexJson (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
            return;
        }

        cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
        if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to get task_instance_index_array (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);
        if (TaskInstanceIndexArraySize == 0)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): TaskInstanceIndexArraySize is 0 (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        bool task_ticket_exist = false;
        bool loop_flag = true;
        size_t task_ticket_query_idx;
        for (size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
        {
            cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
            if (task_instance_index_temp == NULL)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to get task_instance_index_temp (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                cJSON_Delete(TaskInstanceIndexJson);
                return;
            }

            cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
            if (task_ticket_temp == NULL)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Failed to get task_ticket_temp (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
                cJSON_Delete(TaskInstanceIndexJson);
                return;
            }

            if (task_execute_info_local.task_ticket == task_ticket_temp->valuestring)
            {
                task_ticket_exist = true;
                task_ticket_query_idx = i;
                loop_flag = false;
            }
        }

        if (!task_ticket_exist)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): task_ticket is not exist in TaskInstanceIndex (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        cJSON* task_instance_index_new = cJSON_GetArrayItem(task_instance_index_array, task_ticket_query_idx);
        cJSON_ReplaceItemInObject(task_instance_index_new, "point_count", cJSON_CreateNumber(task_execute_point_count));

        char* temp_str = cJSON_Print(TaskInstanceIndexJson);
        cJSON_Delete(TaskInstanceIndexJson);

        if (!temp_str)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): cJSON_Print(TaskInstanceIndexJson) error (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
            return;
        }

        std::string TaskInstanceIndexContentNew = temp_str;
        free(temp_str);

        std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
        if (!TaskInstanceIndexFileWrite)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Can't open TaskInstanceIndex.json in local2 (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
            return;
        }

        TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

        if (!TaskInstanceIndexFileWrite.good())
        {
            TaskInstanceIndexFileWrite.close();

            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteMonitorPoint(): Error: Failed to write to TaskInstanceIndex.json (" << task_execute_info_local.task_ticket << " - " << monitor_point << ")!");
            return;
        }

        TaskInstanceIndexFileWrite.close();
    }
}

bool Task_Execute::TaskStartAuto(const std::string task_ticket)
{
    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartAuto(): Can't open TaskInstanceIndex.json in loacal1 (" << task_ticket << ")!");
        return false;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartAuto(): Failed to parse TaskInstanceIndexJson (" << task_ticket << ")!");
        return false;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartAuto(): Failed to get task_instance_index_array (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return false;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);
    if (TaskInstanceIndexArraySize == 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartAuto(): TaskInstanceIndexArraySize is 0 (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return false;
    }

    bool task_ticket_exist = false;
    bool loop_flag = true;
    size_t task_ticket_query_idx;
    for (size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
    {
        cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
        if (task_instance_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartAuto(): Failed to get task_instance_index_temp (" << task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
        if (task_ticket_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartAuto(): Failed to get task_ticket_temp (" << task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        if (task_ticket == task_ticket_temp->valuestring)
        {
            task_ticket_exist = true;
            task_ticket_query_idx = i;
            loop_flag = false;
        }
    }

    if (!task_ticket_exist)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartAuto(): task_ticket is not in TaskInstanceIndex (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return false;
    }

    cJSON* task_instance_index_new = cJSON_GetArrayItem(task_instance_index_array, task_ticket_query_idx);
    cJSON_ReplaceItemInObject(task_instance_index_new, "status", cJSON_CreateString("e_started"));

    char* temp_str = cJSON_Print(TaskInstanceIndexJson);
    cJSON_Delete(TaskInstanceIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartAuto(): cJSON_Print(TaskInstanceIndexJson) error (" << task_ticket << ")!");
        return false;
    }

    std::string TaskInstanceIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInstanceIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartAuto(): Can't open TaskInstanceIndex.json in local2 (" << task_ticket << ")!");
        return false;
    }

    TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

    if (!TaskInstanceIndexFileWrite.good())
    {
        TaskInstanceIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartAuto(): Error: Failed to write to TaskInstanceIndex.json (" << task_ticket << ")!");
        return false;
    }

    TaskInstanceIndexFileWrite.close();

    return true;
}

bool Task_Execute::TaskPauseAuto(const std::string task_ticket)
{
    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPauseAuto(): Can't open TaskInstanceIndex.json in local1 (" << task_ticket << ")!");
        return false;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPauseAuto(): Failed to parse TaskInstanceIndexJson (" << task_ticket << ")!");
        return false;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPauseAuto(): Failed to get task_instance_index_array (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return false;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);

    if (TaskInstanceIndexArraySize == 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPauseAuto(): TaskInstanceIndexArraySize is 0 (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return false;
    }

    bool task_ticket_exist = false;
    bool loop_flag = true;
    size_t task_ticket_query_idx;
    for (size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
    {
        cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
        if (task_instance_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPauseAuto(): Failed to get task_instance_index_temp (" << task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");

        if (task_ticket_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPauseAuto(): Failed to get task_ticket_temp (" << task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        if (task_ticket == task_ticket_temp->valuestring)
        {
            task_ticket_exist = true;
            task_ticket_query_idx = i;
            loop_flag = false;
        }
    }

    if (!task_ticket_exist)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPauseAuto(): task_ticket is not in TaskInstanceIndex (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return false;
    }

    cJSON* task_instance_index_new = cJSON_GetArrayItem(task_instance_index_array, task_ticket_query_idx);
    cJSON_ReplaceItemInObject(task_instance_index_new, "status", cJSON_CreateString("e_paused_auto"));

    char* temp_str = cJSON_Print(TaskInstanceIndexJson);
    cJSON_Delete(TaskInstanceIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPauseAuto(): cJSON_Print(TaskInstanceIndexJson) error (" << task_ticket << ")!");
        return false;
    }

    std::string TaskInstanceIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInstanceIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPauseAuto(): Can't open TaskInstanceIndex.json in local2 (" << task_ticket << ")!");
        return false;
    }

    TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

    if (!TaskInstanceIndexFileWrite.good())
    {
        TaskInstanceIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPauseAuto(): Error: Failed to write to TaskInstanceIndex.json (" << task_ticket << ")!");
        return false;
    }

    TaskInstanceIndexFileWrite.close();

    return true;
}

void Task_Execute::TaskStop(const std::string task_ticket)
{
    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): Can't open TaskInstanceIndex.json in local1 (" << task_ticket << ")!");
        return;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): Failed to parse TaskInstanceIndexJson (" << task_ticket << ")!");
        return;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): Failed to get task_instance_index_array (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);

    if (TaskInstanceIndexArraySize == 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): TaskInstanceIndexArraySize is 0 (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    bool task_ticket_exist = false;
    bool loop_flag = true;
    std::string start_time_query;
    size_t task_ticket_query_idx;
    for (size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
    {
        cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
        if (task_instance_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): Failed to get task_instance_index_temp (" << task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
        cJSON* start_time_temp = cJSON_GetObjectItem(task_instance_index_temp, "start_time");

        if (task_ticket_temp == NULL || start_time_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): Failed to get task_ticket_temp or start_time_temp (" << task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        if (task_ticket == task_ticket_temp->valuestring)
        {
            task_ticket_exist = true;
            start_time_query = start_time_temp->valuestring;
            task_ticket_query_idx = i;
            loop_flag = false;
        }
    }

    if (!task_ticket_exist)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): task_ticket is not in TaskInstanceIndex (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    std::string end_time;
    {
        auto now = std::chrono::system_clock::now();
        std::time_t time_now = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm = *std::localtime(&time_now);
        std::ostringstream oss;
        oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
        end_time = oss.str();
    }

    // 1.将字符串解析到 std::tm 结构
    std::tm tm_start = {};
    std::tm tm_end = {};

    {
        std::istringstream iss(start_time_query);
        // 按照 YYYY-MM-DD HH:MM:SS 格式解析
        iss >> std::get_time(&tm_start, "%Y-%m-%d %H:%M:%S");
        if (iss.fail())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): Failed to parse start_time (" << task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }
    }

    {
        std::istringstream iss(end_time);
        iss >> std::get_time(&tm_end, "%Y-%m-%d %H:%M:%S");
        if (iss.fail())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): Failed to parse end_time (" << task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }
    }

    // 2. 使用 mktime 将 std::tm 转成 time_t (单位是从 1970.1.1 以来的秒数)
    time_t time_start = std::mktime(&tm_start);
    time_t time_end = std::mktime(&tm_end);

    // 3. 计算二者差值，单位为秒
    double diff_seconds = std::difftime(time_end, time_start);

    cJSON* task_instance_index_new = cJSON_GetArrayItem(task_instance_index_array, task_ticket_query_idx);
    cJSON_ReplaceItemInObject(task_instance_index_new, "end_time", cJSON_CreateString(end_time.c_str()));
    cJSON_ReplaceItemInObject(task_instance_index_new, "execute_time", cJSON_CreateNumber(diff_seconds));

    char* temp_str = cJSON_Print(TaskInstanceIndexJson);
    cJSON_Delete(TaskInstanceIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): cJSON_Print(TaskInstanceIndexJson) error (" << task_ticket << ")!");
        return;
    }

    std::string TaskInstanceIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInstanceIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): Can't open TaskInstanceIndex.json in local2 (" << task_ticket << ")!");
        return;
    }

    TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

    if (!TaskInstanceIndexFileWrite.good())
    {
        TaskInstanceIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStop(): Error: Failed to write to TaskInstanceIndex.json (" << task_ticket << ")!");
        return;
    }

    TaskInstanceIndexFileWrite.close();
}

void Task_Execute::TaskFinish(const task_execute_info task_execute_info_local)
{
    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinish(): Can't open TaskInstanceIndex.json in local1 (" << task_execute_info_local.task_ticket << ")!");
        return;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinish(): Failed to parse TaskInstanceIndexJson (" << task_execute_info_local.task_ticket << ")!");
        return;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinish(): Failed to get task_info_index_array (" << task_execute_info_local.task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);

    if (TaskInstanceIndexArraySize == 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinish(): TaskInstanceIndexArraySize is 0 (" << task_execute_info_local.task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    bool task_ticket_exist = false;
    bool loop_flag = true;
    std::string start_time_query;
    size_t task_ticket_query_idx;
    for (size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
    {
        cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
        if (task_instance_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinish(): Failed to get task_info_index_temp (" << task_execute_info_local.task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
        cJSON* start_time_temp = cJSON_GetObjectItem(task_instance_index_temp, "start_time");

        if (task_ticket_temp == NULL || start_time_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinish(): Failed to get task_ticket_temp or start_time_temp (" << task_execute_info_local.task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        if (task_execute_info_local.task_ticket == task_ticket_temp->valuestring)
        {
            task_ticket_exist = true;
            start_time_query = start_time_temp->valuestring;
            task_ticket_query_idx = i;
            loop_flag = false;
        }
    }

    if (!task_ticket_exist)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinish(): task_ticket is not in TaskInstanceIndex (" << task_execute_info_local.task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    std::string end_time;
    {
        auto now = std::chrono::system_clock::now();
        std::time_t time_now = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm = *std::localtime(&time_now);
        std::ostringstream oss;
        oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
        end_time = oss.str();
    }

    // 1. 将字符串解析到 std::tm 结构
    std::tm tm_start = {};
    std::tm tm_end = {};

    {
        std::istringstream iss(start_time_query);
        // 按照 YYYY-MM-DD HH:MM:SS 格式解析
        iss >> std::get_time(&tm_start, "%Y-%m-%d %H:%M:%S");
        if (iss.fail())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinish(): Failed to parse start_time (" << task_execute_info_local.task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }
    }

    {
        std::istringstream iss(end_time);
        iss >> std::get_time(&tm_end, "%Y-%m-%d %H:%M:%S");
        if (iss.fail())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinish(): Failed to parse end_time (" << task_execute_info_local.task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }
    }

    // 2. 使用 mktime 将 std::tm 转成 time_t (单位是从 1970.1.1 以来的秒数)
    time_t time_start = std::mktime(&tm_start);
    time_t time_end = std::mktime(&tm_end);

    // 3. 计算二者差值，单位为秒
    double diff_seconds = std::difftime(time_end, time_start);

    cJSON* task_instance_index_new = cJSON_GetArrayItem(task_instance_index_array, task_ticket_query_idx);
    cJSON_ReplaceItemInObject(task_instance_index_new, "status", cJSON_CreateString("e_finished"));
    cJSON_ReplaceItemInObject(task_instance_index_new, "end_time", cJSON_CreateString(end_time.c_str()));
    cJSON_ReplaceItemInObject(task_instance_index_new, "execute_time", cJSON_CreateNumber(diff_seconds));

    char* temp_str = cJSON_Print(TaskInstanceIndexJson);
    cJSON_Delete(TaskInstanceIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinish(): cJSON_Print(TaskInstanceIndexJson) error (" << task_execute_info_local.task_ticket << ")!");
        return;
    }

    std::string TaskInstanceIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInstanceIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinish(): Can't open TaskInstanceIndex.json (" << task_execute_info_local.task_ticket << ")!");
        return;
    }

    TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

    if (!TaskInstanceIndexFileWrite.good())
    {
        TaskInstanceIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Error: Failed to write to TaskInstanceIndex.json (" << task_execute_info_local.task_ticket << ")!");
        return;
    }

    TaskInstanceIndexFileWrite.close();

    robot_msgs::msg::InspectTaskExecutionComplete InspectTaskExecutionCompleteMsg;
    InspectTaskExecutionCompleteMsg.id_task_info = id_device + "-task-info-" + task_execute_info_local.id_task_info;
    InspectTaskExecutionCompleteMsg.task_ticket = id_device + "-task-ticket-" + task_execute_info_local.task_ticket;
    InspectTaskExecutionCompleteMsg.point_count = task_execute_info_local.point_count;
    InspectTaskExecutionCompleteMsg.start_time = start_time_query;
    InspectTaskExecutionCompleteMsg.end_time = end_time;
    InspectTaskExecutionCompleteMsg.execute_time = diff_seconds;

    TaskStartOrFinishCache TaskStartOrFinishCache_temp;
    TaskStartOrFinishCache_temp.msg_send_time = std::time(nullptr);
    TaskStartOrFinishCache_temp.response_flag = false;
    TaskStartOrFinishCache_temp.start_or_finish = "complete";
    TaskStartOrFinishCache_temp.InspectTaskExecutionCompleteMsg = InspectTaskExecutionCompleteMsg;
    TaskStartOrFinishCache_vector.push_back(TaskStartOrFinishCache_temp);

    InspectTaskExecutionComplete_pub->publish(InspectTaskExecutionCompleteMsg);
}

void Task_Execute::TaskOuttime(const std::string task_ticket)
{
    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskOuttime(): Can't open TaskInstanceIndex.json (" << task_ticket << ")!");
        return;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskOuttime(): Failed to parse TaskInstanceIndexJson (" << task_ticket << ")!");
        return;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskOuttime(): Failed to get task_instance_index_array (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);
    if (TaskInstanceIndexArraySize == 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskOuttime(): TaskInfoIndexArraySize is 0 (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    bool task_ticket_exist = false;
    bool loop_flag = true;
    size_t task_ticket_query_idx;
    for (size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
    {
        cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
        if (task_instance_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskOuttime(): Failed to get task_instance_index_temp (" << task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
        if (task_ticket_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskOuttime(): Failed to get task_ticket_temp (" << task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        if (task_ticket == task_ticket_temp->valuestring)
        {
            task_ticket_exist = true;
            task_ticket_query_idx = i;
            loop_flag = false;
        }
    }

    if (!task_ticket_exist)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskOuttime(): task_ticket is not in TaskInstanceIndex (" << task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    cJSON* task_instance_index_new = cJSON_GetArrayItem(task_instance_index_array, task_ticket_query_idx);
    cJSON_ReplaceItemInObject(task_instance_index_new, "status", cJSON_CreateString("e_outtime"));

    char* temp_str = cJSON_Print(TaskInstanceIndexJson);
    cJSON_Delete(TaskInstanceIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskOuttime(): cJSON_Print(TaskInstanceIndexJson) error (" << task_ticket << ")!");
        return;
    }

    std::string TaskInstanceIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInstanceIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskOuttime(): Can't open TaskInstanceIndex.json (" << task_ticket << ")!");
        return;
    }

    TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

    if (!TaskInstanceIndexFileWrite.good())
    {
        TaskInstanceIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskOuttime(): Error: Failed to write to TaskInstanceIndex.json (" << task_ticket << ")!");
        return;
    }

    TaskInstanceIndexFileWrite.close();
}

void Task_Execute::PlanTaskStartAuto(const task_execute_info task_execute_info_local)
{
    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] PlanTaskStartAuto(): Can't open TaskInstanceIndex.json (" << task_execute_info_local.task_ticket << ")!");
        return;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] PlanTaskStartAuto(): Failed to parse TaskInstanceIndexJson (" << task_execute_info_local.task_ticket << ")!");
        return;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] PlanTaskStartAuto(): Failed to get task_instance_index_array (" << task_execute_info_local.task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);

    if (TaskInstanceIndexArraySize == 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] PlanTaskStartAuto(): TaskInstanceIndexArraySize is 0 (" << task_execute_info_local.task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    bool task_ticket_exist = false;
    bool loop_flag = true;
    size_t task_ticket_query_idx;
    for (size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
    {
        cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
        if (task_instance_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] PlanTaskStartAuto(): Failed to get task_instance_index_temp (" << task_execute_info_local.task_ticket << ")!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
        if (task_execute_info_local.task_ticket == task_ticket_temp->valuestring)
        {
            task_ticket_exist = true;
            task_ticket_query_idx = i;
            loop_flag = false;
        }
    }

    if (!task_ticket_exist)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] PlanTaskStartAuto(): task_ticket is not in TaskInstanceIndex (" << task_execute_info_local.task_ticket << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    std::string start_time;
    {
        auto now = std::chrono::system_clock::now();
        std::time_t time_now = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm = *std::localtime(&time_now);
        std::ostringstream oss;
        oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
        start_time = oss.str();
    }

    cJSON* task_instance_index_new = cJSON_GetArrayItem(task_instance_index_array, task_ticket_query_idx);
    cJSON_ReplaceItemInObject(task_instance_index_new, "status", cJSON_CreateString("e_started"));
    cJSON_ReplaceItemInObject(task_instance_index_new, "start_time", cJSON_CreateString(start_time.c_str()));

    char* temp_str = cJSON_Print(TaskInstanceIndexJson);
    cJSON_Delete(TaskInstanceIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] PlanTaskStartAuto(): cJSON_Print(TaskInstanceIndexJson) error (" << task_execute_info_local.task_ticket << ")!");
        return;
    }

    std::string TaskInstanceIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInstanceIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] PlanTaskStartAuto(): Can't open TaskInstanceIndex.json (" << task_execute_info_local.task_ticket << ")!");
        return;
    }

    TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

    if (!TaskInstanceIndexFileWrite.good())
    {
        TaskInstanceIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] PlanTaskStartAuto(): Error: Failed to write to TaskInstanceIndex.json (" << task_execute_info_local.task_ticket << ")!");
        return;
    }

    TaskInstanceIndexFileWrite.close();

    robot_msgs::msg::InspectTaskExecutionStart InspectTaskExecutionStartMsg;
    InspectTaskExecutionStartMsg.id_task_info = id_device + "-task-info-" + task_execute_info_local.id_task_info;
    InspectTaskExecutionStartMsg.id_task_plan = id_device + "-task-plan-" + task_execute_info_local.id_task_plan;
    InspectTaskExecutionStartMsg.task_ticket = id_device + "-task-ticket-" + task_execute_info_local.task_ticket;
    InspectTaskExecutionStartMsg.start_time = start_time;

    TaskStartOrFinishCache TaskStartOrFinishCache_temp;
    TaskStartOrFinishCache_temp.msg_send_time = std::time(nullptr);
    TaskStartOrFinishCache_temp.response_flag = false;
    TaskStartOrFinishCache_temp.start_or_finish = "start";
    TaskStartOrFinishCache_temp.InspectTaskExecutionStartMsg = InspectTaskExecutionStartMsg;
    TaskStartOrFinishCache_vector.push_back(TaskStartOrFinishCache_temp);

    InspectTaskExecutionStart_pub->publish(InspectTaskExecutionStartMsg);
}

bool Task_Execute::ExecuteRobotHomeStatus(const task_execute_info task_execute_info_local)
{
    bool execute_monitor_point_lock_local = true;
    while (execute_monitor_point_lock_local)
    {
        bool execute_monitor_point_locks = false;
        for (size_t i = 0; i < task_execute_infos.size(); i++)
        {
            execute_monitor_point_locks = execute_monitor_point_locks || task_execute_infos[i].execute_monitor_point_lock;
        }

        execute_monitor_point_lock_local = execute_monitor_point_locks;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    bool arm_home_local;
    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
        {
            task_execute_infos[i].execute_robot_home_lock = true;
            arm_home_local = task_execute_infos[i].arm_home;
        }
    }

    bool arm_home_flag_local = true;

    if (!arm_home_local)
    {
        bool arm_home_flag = true;

        if (device_type == "e_robot_crawler")
        {
            robot_msgs::srv::LiftCtrlSrv::Request::SharedPtr lift_request;
            robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;

            arm_request->pos_key = "posrightend";
            auto arm_shared_future_ptr1 = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
            auto arm_res_callback1 = [this, arm_request, arm_shared_future_ptr1, &arm_home_flag_local, task_execute_info_local](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
            {
                *arm_shared_future_ptr1 = future;
                auto response = future.get();
                if (!response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << ")!");
                    arm_home_flag_local = false;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm successfully move to target pos: " << arm_request->pos_key << "!");
                }
            };
            
            auto arm_future1 = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback1);
            *arm_shared_future_ptr1 = arm_future1.future;
            
            auto arm_timer1 = this->create_wall_timer(std::chrono::milliseconds(500),
                [this, arm_shared_future_ptr1, arm_request, &arm_home_flag_local, task_execute_info_local]()
                {
                    if (arm_shared_future_ptr1->valid() &&
                        arm_shared_future_ptr1->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << ")!");
                        arm_home_flag_local = false;
                    }
                });

            arm_request->pos_key = "posinit";
            auto arm_shared_future_ptr2 = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
            auto arm_res_callback2 = [this, arm_request, arm_shared_future_ptr2, &arm_home_flag_local, task_execute_info_local](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
            {
                *arm_shared_future_ptr2 = future;
                auto response = future.get();
                if (!response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << ")!");
                    arm_home_flag_local = false;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm successfully move to target pos: " << arm_request->pos_key << "!");
                }
            };
            
            auto arm_future2 = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback2);
            *arm_shared_future_ptr2 = arm_future2.future;
            
            auto arm_timer2 = this->create_wall_timer(std::chrono::milliseconds(500),
                [this, arm_shared_future_ptr2, arm_request, &arm_home_flag_local, task_execute_info_local]()
                {
                    if (arm_shared_future_ptr2->valid() &&
                        arm_shared_future_ptr2->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << ")!");
                        arm_home_flag_local = false;
                    }
                });

            arm_request->pos_key = "poshome";
            auto arm_shared_future_ptr3 = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
            auto arm_res_callback3 = [this, arm_request, arm_shared_future_ptr3, &arm_home_flag_local, task_execute_info_local](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
            {
                *arm_shared_future_ptr3 = future;
                auto response = future.get();
                if (!response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << ")!");
                    arm_home_flag_local = false;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm successfully move to target pos: " << arm_request->pos_key << "!");
                }
            };
            
            auto arm_future3 = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback3);
            *arm_shared_future_ptr3 = arm_future3.future;
            
            auto arm_timer3 = this->create_wall_timer(std::chrono::milliseconds(500),
                [this, arm_shared_future_ptr3, arm_request, &arm_home_flag_local, task_execute_info_local]()
                {
                    if (arm_shared_future_ptr3->valid() &&
                        arm_shared_future_ptr3->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << ")!");
                        arm_home_flag_local = false;
                    }
                });

            if (arm_home_flag)
            {
                lift_request->position = 0.0;
                auto lift_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture>();
                auto lift_res_callback = [this, lift_request, lift_shared_future_ptr, &arm_home_flag_local, task_execute_info_local](rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture future)
                {
                    *lift_shared_future_ptr = future;
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Lift failed to move to target pos(" << lift_request->position<< ") (" << task_execute_info_local.task_ticket << ")!");
                        arm_home_flag_local = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Lift successfully move to target pos: " << lift_request->position << "!");
                    }
                };
                
                auto lift_future = LiftCtrlSrv_client->async_send_request(lift_request, lift_res_callback);
                *lift_shared_future_ptr = lift_future.future;
                
                auto lift_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, lift_shared_future_ptr, lift_request, &arm_home_flag_local, task_execute_info_local]()
                    {
                        if (lift_shared_future_ptr->valid() &&
                            lift_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Lift target pos(" << lift_request->position<< ") is out of range (" << task_execute_info_local.task_ticket << ")!");
                            arm_home_flag_local = false;
                        }
                    });
            }
        }
        else if (device_type == "e_robot_track")
        {
            robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;
            if (arm_home_flag)
            {
                arm_request->pos_key = "poshome";
            auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
            auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_home_flag_local, task_execute_info_local](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
            {
                *arm_shared_future_ptr = future;
                auto response = future.get();
                if (!response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << ")!");
                    arm_home_flag_local = false;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm successfully move to target pos: " << arm_request->pos_key << "!");
                }
            };
            
            auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
            *arm_shared_future_ptr = arm_future.future;
            
            auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                [this, arm_shared_future_ptr, arm_request, &arm_home_flag_local, task_execute_info_local]()
                {
                    if (arm_shared_future_ptr->valid() &&
                        arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << ")!");
                        arm_home_flag_local = false;
                    }
                });
            }
        }
        else if (device_type == "e_robot_quadrupedal")
        {
            robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;
            if (arm_home_flag)
            {
                arm_request->pos_key = "poshome";
            auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
            auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_home_flag_local, task_execute_info_local](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
            {
                *arm_shared_future_ptr = future;
                auto response = future.get();
                if (!response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm failed to move to target pos(" << arm_request->pos_key << ") (" << task_execute_info_local.task_ticket << ")!");
                    arm_home_flag_local = false;
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm successfully move to target pos: " << arm_request->pos_key << "!");
                }
            };
            
            auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
            *arm_shared_future_ptr = arm_future.future;
            
            auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                [this, arm_shared_future_ptr, arm_request, &arm_home_flag_local, task_execute_info_local]()
                {
                    if (arm_shared_future_ptr->valid() &&
                        arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ExecuteRobotHomeStatus(): Arm target pos(" << arm_request->pos_key << ") is out of range (" << task_execute_info_local.task_ticket << ")!");
                        arm_home_flag_local = false;
                    }
                });
            }
        }

        for (size_t i = 0; i < task_execute_infos.size(); i++)
        {
            if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
            {
                task_execute_infos[i].arm_home = arm_home_flag_local;
            }
        }
    }

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_execute_info_local.task_ticket)
        {
            task_execute_infos[i].execute_robot_home_lock = false;
        }
    }

    return arm_home_flag_local;
}

/********************************************************任务计划*************************************************************/
void Task_Execute::CronPlanTaskListUpdate()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskListUpdate Thread Start!");

    while (rclcpp::ok())
    {
        {
            /* 睡到下一个整分钟，避免累加误差 */
            // 先拿当前的系统实时时间（精确到秒+纳秒）
            timespec nowts;
            clock_gettime(CLOCK_MONOTONIC, &nowts);

            // 转成本地时间
            tm localt;
            localtime_r(&nowts.tv_sec, &localt);

            // 将秒清零，并让分钟+1
            localt.tm_sec = 0;
            localt.tm_min += 1;

            // 处理分钟进位可能导致的小时/日变化，mktime 会自动进位并返回新的 time_t
            time_t next_min = mktime(&localt);

            // 构造要睡到的 timespec（整分钟时刻，纳秒为0）
            timespec ts;
            ts.tv_sec  = next_min;
            ts.tv_nsec = 500UL * 1000UL * 1000UL;

            // 让线程休眠到下一个整分钟
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
        }

        std::vector<std::pair<std::string, std::vector<std::pair<std::string, std::string>>>> task_plan_array; /* vector<pair<id_task_info, vector<pair<id_task_plan,plan_type>>>> */

        if (GetTaskPlanArray(task_plan_array))
        {
            /*ROS_INFO_STREAM("task_plan_array size: " << task_plan_array.size());*/
            /*for (size_t i = 0; i < task_plan_array.size(); i++)*/
            /*{*/
            /*ROS_INFO_STREAM("id_task_info: " << task_plan_array[i].first);*/
            /*ROS_INFO_STREAM("single_task_plan_array size: " << task_plan_array[i].second.size());*/
            /*for (size_t j = 0; j < task_plan_array[i].second.size(); j++)*/
            /*{*/
            /*ROS_INFO_STREAM("id_task_plan: " << task_plan_array[i].second[j].first);*/
            /*ROS_INFO_STREAM("plan_type: " << task_plan_array[i].second[j].second);*/
            /*}*/
            /*}*/

            /*ROS_INFO_STREAM("********************************************");*/

            // 先将TaskPlan文件里新的task_plan添加进cron_plan_task 数组中
            for (size_t i = 0; i < task_plan_array.size(); i++)
            {
                for (size_t j = 0; j < task_plan_array[i].second.size(); j++)
                {
                    bool task_plan_exist = false;
                    for (size_t k = 0; k < cron_plan_task.size(); k++)
                    {
                        if (task_plan_array[i].second[j].first == cron_plan_task[k].id_task_plan)
                        {
                            task_plan_exist = true;
                            break;
                        }
                    }

                    if (!task_plan_exist)
                    {
                        CronPlanTask cron_plan_task_temp;
                        cron_plan_task_temp.id_task_info = task_plan_array[i].first;
                        cron_plan_task_temp.id_task_plan = task_plan_array[i].second[j].first;
                        cron_plan_task_temp.plan_type = task_plan_array[i].second[j].second;
                        cron_plan_task_temp.first_analysed = false;
                        if (GetCronPlanTaskInfo(cron_plan_task_temp))
                        {
                            for (size_t m = 0; m < cron_plan_task_temp.cron_task.size(); m++)
                            {
                                cron_plan_task_temp.cron_task[m].expr = Cron_Analyse::CronExpr{};
                                cron_plan_task_temp.cron_task[m].parsed = false;
                                cron_plan_task_temp.cron_task[m].errmsg[0] = '\0';
                                cron_plan_task_temp.cron_task[m].last = {};
                            }

                            cron_plan_task.push_back(cron_plan_task_temp);
                        }
                        else
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Failed to get CronPlanTaskInfo!");
                        }
                    }
                }
            }

            // 再将TaskPlan文件中被删除的task_plan从cron_plan_task 数组中移除
            std::vector<size_t> cron_plan_task_erase_idx;

            for (size_t i = 0; i < cron_plan_task.size(); i++)
            {
                bool task_plan_exist = false;
                for (size_t j = 0; j < task_plan_array.size(); j++)
                {
                    for (size_t k = 0; k < task_plan_array[j].second.size(); k++)
                    {
                        if (cron_plan_task[i].id_task_plan == task_plan_array[j].second[k].first)
                        {
                            task_plan_exist = true;
                            break;
                        }
                    }

                    if (task_plan_exist)
                    {
                        break;
                    }
                }

                if (!task_plan_exist)
                {
                    cron_plan_task_erase_idx.push_back(i);
                }
            }

            for (auto it = cron_plan_task_erase_idx.rbegin(); it != cron_plan_task_erase_idx.rend(); ++it)
            {
                cron_plan_task.erase(cron_plan_task.begin() + *it);
            }

            /*ROS_INFO_STREAM("cron_plan_task size: " << cron_plan_task.size());*/

            /*for (size_t i = 0; i < cron_plan_task.size(); i++)*/
            /*{*/
            /*ROS_INFO_STREAM("cron_plan_task[" << i << "] - id_task_info: " << cron_plan_task[i].id_task_info);*/
            /*ROS_INFO_STREAM("cron_plan_task[" << i << "] - task_type: " << cron_plan_task[i].task_type);*/
            /*ROS_INFO_STREAM("cron_plan_task[" << i << "] - task_name: " << cron_plan_task[i].task_name);*/
            /*ROS_INFO_STREAM("cron_plan_task[" << i << "] - id_task_plan: " << cron_plan_task[i].id_task_plan);*/
            /*ROS_INFO_STREAM("cron_plan_task[" << i << "] - plan_type: " << cron_plan_task[i].plan_type);*/
            /*for (size_t j = 0; j < cron_plan_task[i].cron_task.size(); j++)*/
            /*ROS_INFO_STREAM("cron_plan_task[" << i << "] - cron_expr[" << j << "]: " << cron_plan_task[i].cron_task[j].expr_str);*/
            /*}*/

            /*ROS_INFO_STREAM("----------------------------------------------");*/

            // 解析任务cron表达式
            for (size_t i = 0; i < cron_plan_task.size(); i++)
            {
                if (!cron_plan_task[i].first_analysed)
                {
                    for (size_t j = 0; j < cron_plan_task[i].cron_task.size(); j++)
                    {
                        if (!Cron_Analyse::cron_parse(cron_plan_task[i].cron_task[j].expr_str.c_str(), &cron_plan_task[i].cron_task[j].expr, cron_plan_task[i].cron_task[j].errmsg, sizeof(cron_plan_task[i].cron_task[j].errmsg)))
                        {
                            cron_plan_task[i].cron_task[j].parsed = true;
                            cron_plan_task[i].cron_task[j].last.tm_sec = -1; // 标记未触发

                            RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] cron express: \"" << cron_plan_task[i].cron_task[j].expr_str << "\" parsed successfully!");
                        }
                        else
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] EXPR ERROR: \"" << cron_plan_task[i].cron_task[j].expr_str << "\" -> " << cron_plan_task[i].cron_task[j].errmsg);
                        }
                    }
                }

                cron_plan_task[i].first_analysed = true;
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Failed to get task_plan_array!");
        }
    }
}

void Task_Execute::CronPlanTaskTrigger()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskTrigger Thread Start!");

    while (rclcpp::ok())
    {
        {
            /* 睡到下一个整秒，避免累加误差 */
            struct timespec nowts;
            clock_gettime(CLOCK_MONOTONIC, &nowts);
            time_t next_sec = nowts.tv_sec + 1;
            // struct timespec ts = {.tv_sec = next_sec, .tv_nsec = 0};
            struct timespec ts = {next_sec, 0};
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        }

        /* 取当前本地时间 */
        time_t now = time(NULL);
        struct tm tm_now;
        localtime_r(&now, &tm_now);

        /* 遍历任务计划 */
        for (size_t i = 0; i < cron_plan_task.size(); i++)
        {
            for (size_t j = 0; j < cron_plan_task[i].cron_task.size(); j++)
            {
                if (!cron_plan_task[i].cron_task[j].parsed)
                {
                    continue; /* 解析失败的跳过 */
                }

                /* 防止同一秒重复触发 */
                if (cron_plan_task[i].cron_task[j].last.tm_year == tm_now.tm_year
                        && cron_plan_task[i].cron_task[j].last.tm_mon == tm_now.tm_mon
                        && cron_plan_task[i].cron_task[j].last.tm_mday == tm_now.tm_mday
                        && cron_plan_task[i].cron_task[j].last.tm_hour == tm_now.tm_hour
                        && cron_plan_task[i].cron_task[j].last.tm_min == tm_now.tm_min
                        && cron_plan_task[i].cron_task[j].last.tm_sec == tm_now.tm_sec)
                {
                    continue;
                }

                if (Cron_Analyse::cron_match(&cron_plan_task[i].cron_task[j].expr, &tm_now))
                {
                    cron_plan_task[i].cron_task[j].last = tm_now; /* 记录触发时间 */

                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] task_plan: \"" << cron_plan_task[i].id_task_plan << "\" -> cron_express: \"" << cron_plan_task[i].cron_task[j].expr_str << "\" -> triggered!");
                    }

                    CronPlanTaskStart(cron_plan_task[i]);
                }
            }
        }
    }
}

bool Task_Execute::GetTaskPlanArray(std::vector<std::pair<std::string, std::vector<std::pair<std::string, std::string>>>>& task_plan_array)
{
    std::string TaskPlanIndexFileDir = TaskFilesDir + "task_plan_files/TaskPlanIndex.json";
    std::ifstream TaskPlanIndexFileRead(TaskPlanIndexFileDir.c_str());
    if (!TaskPlanIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetTaskPlanArray(): Can't open TaskPlanIndex.json!");
        return false;
    }

    std::string TaskPlanIndexContent((std::istreambuf_iterator<char>(TaskPlanIndexFileRead)), std::istreambuf_iterator<char>());
    TaskPlanIndexFileRead.close();

    cJSON* TaskPlanIndexJson = cJSON_Parse(TaskPlanIndexContent.c_str());
    if (TaskPlanIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetTaskPlanArray(): Failed to parse TaskPlanIndexJson!");
        return false;
    }

    cJSON* value_id_task_info_plan = cJSON_GetObjectItem(TaskPlanIndexJson, "id_task_info_plan");
    if (value_id_task_info_plan == NULL || !cJSON_IsArray(value_id_task_info_plan))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetTaskPlanArray(): Failed to get value_id_task_info_plan!");
        cJSON_Delete(TaskPlanIndexJson);
        return false;
    }

    size_t IdTaskInfoPlanArraySize = cJSON_GetArraySize(value_id_task_info_plan);
    if (IdTaskInfoPlanArraySize == 0)
    {
        //ROS_WARN_STREAM("GetTaskPlanArray(): IdTaskInfoPlanArraySize is 0!");
        cJSON_Delete(TaskPlanIndexJson);
        return true;
    }

    std::vector<std::string> id_task_info_plan_array;
    for (size_t i = 0; i < IdTaskInfoPlanArraySize; i++)
    {
        cJSON* id_task_info_plan_temp = cJSON_GetArrayItem(value_id_task_info_plan, i);
        if (id_task_info_plan_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetTaskPlanArray(): Failed to get id_task_info_plan_temp!");
            cJSON_Delete(TaskPlanIndexJson);
            return false;
        }

        id_task_info_plan_array.push_back(id_task_info_plan_temp->valuestring);
    }

    for (size_t i = 0; i < IdTaskInfoPlanArraySize; i++)
    {
        std::pair<std::string, std::vector<std::pair<std::string, std::string>>> single_task_plan_array_temp;
        single_task_plan_array_temp.first = id_task_info_plan_array[i];

        cJSON* single_task_plan_index_array = cJSON_GetObjectItem(TaskPlanIndexJson, id_task_info_plan_array[i].c_str());
        if (single_task_plan_index_array == NULL || !cJSON_IsArray(single_task_plan_index_array))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetTaskPlanArray(): Failed to get single_task_plan_index_array of id_task_info(" << id_task_info_plan_array[i] << ")!");
            cJSON_Delete(TaskPlanIndexJson);
            return false;
        }

        size_t SingleTaskPlanIndexArraySize = cJSON_GetArraySize(single_task_plan_index_array);
        if (SingleTaskPlanIndexArraySize == 0)
        {
            continue;
        }

        for (size_t j = 0; j < SingleTaskPlanIndexArraySize; j++)
        {
            cJSON* task_plan_temp = cJSON_GetArrayItem(single_task_plan_index_array, j);
            if (task_plan_temp == NULL)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetTaskPlanArray(): Failed to get task_plan_temp of id_task_info(" << id_task_info_plan_array[i] << ")!");
                cJSON_Delete(TaskPlanIndexJson);
                return false;
            }

            cJSON* id_task_plan_temp = cJSON_GetObjectItem(task_plan_temp, "id_task_plan");
            cJSON* plan_type_temp = cJSON_GetObjectItem(task_plan_temp, "plan_type");
            if (id_task_plan_temp == NULL || plan_type_temp == NULL)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetTaskPlanArray(): Failed to get id_task_plan_temp or plan_type_temp of id_task_info(" << id_task_info_plan_array[i] << ")!");
                cJSON_Delete(TaskPlanIndexJson);
                return false;
            }

            std::pair<std::string, std::string> task_plan_single_temp;
            task_plan_single_temp.first = id_task_plan_temp->valuestring;
            task_plan_single_temp.second = plan_type_temp->valuestring;
            single_task_plan_array_temp.second.push_back(task_plan_single_temp);
        }

        task_plan_array.push_back(single_task_plan_array_temp);
    }

    cJSON_Delete(TaskPlanIndexJson);
    return true;
}

bool Task_Execute::GetCronPlanTaskInfo(CronPlanTask& single_cron_plan_task)
{
    std::string TaskInfoIndexFileDir = TaskFilesDir + "task_info_files/TaskInfoIndex.json";
    std::ifstream TaskInfoIndexFileRead(TaskInfoIndexFileDir.c_str());
    if (!TaskInfoIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): Can't open TaskInfoIndex.json!");
        return false;
    }

    std::string TaskInfoIndexContent((std::istreambuf_iterator<char>(TaskInfoIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInfoIndexFileRead.close();

    cJSON* TaskInfoIndexJson = cJSON_Parse(TaskInfoIndexContent.c_str());
    if (TaskInfoIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): Failed to parse TaskInfoIndexJson!");
        return false;
    }

    cJSON* task_info_index_array = cJSON_GetObjectItem(TaskInfoIndexJson, "TaskInfoIndex");
    if (task_info_index_array == NULL || !cJSON_IsArray(task_info_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): Failed to get task_info_index_array!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    size_t TaskInfoIndexArraySize = cJSON_GetArraySize(task_info_index_array);

    if (TaskInfoIndexArraySize == 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): TaskInfoIndexArraySize is 0!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    std::vector<std::tuple<std::string, std::string, std::string>> task_info_array;

    for (size_t i = 0; i < TaskInfoIndexArraySize; i++)
    {
        cJSON* task_info_index_temp = cJSON_GetArrayItem(task_info_index_array, i);
        if (task_info_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): Failed to get task_info_index_temp!");
            cJSON_Delete(TaskInfoIndexJson);
            return false;
        }

        cJSON* id_task_info_temp = cJSON_GetObjectItem(task_info_index_temp, "id_task_info");
        cJSON* task_type_temp = cJSON_GetObjectItem(task_info_index_temp, "task_type");
        cJSON* task_name_temp = cJSON_GetObjectItem(task_info_index_temp, "task_name");
        if (id_task_info_temp == NULL || task_type_temp == NULL || task_name_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): Failed to get id_task_info_temp or task_type_temp or task_name_temp!");
            cJSON_Delete(TaskInfoIndexJson);
            return false;
        }

        std::tuple<std::string, std::string, std::string> task_info_temp(id_task_info_temp->valuestring, task_type_temp->valuestring, task_name_temp->valuestring);
        task_info_array.push_back(task_info_temp);
    }

    cJSON_Delete(TaskInfoIndexJson);

    bool getCronPlanTaskInfoSuccess = false;

    for (size_t i = 0; i < task_info_array.size(); i++)
    {
        if (single_cron_plan_task.id_task_info == std::get<0>(task_info_array[i]))
        {
            single_cron_plan_task.task_type = std::get<1>(task_info_array[i]);
            single_cron_plan_task.task_name = std::get<2>(task_info_array[i]);
            getCronPlanTaskInfoSuccess = true;
            break;
        }
    }

    if (!getCronPlanTaskInfoSuccess)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): get Cron Plan Task Info Failed!");
        return false;
    }

    std::string TaskPlanFileDir = TaskFilesDir + "task_plan_files/" + single_cron_plan_task.plan_type + "/" + single_cron_plan_task.id_task_plan + ".json";
    std::ifstream TaskPlanFileRead(TaskPlanFileDir.c_str());
    if (!TaskPlanFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): Can't open task plan json file(" << single_cron_plan_task.id_task_plan << ")!");
        return false;
    }

    std::string TaskPlanContent((std::istreambuf_iterator<char>(TaskPlanFileRead)), std::istreambuf_iterator<char>());
    TaskPlanFileRead.close();

    cJSON* TaskPlanJson = cJSON_Parse(TaskPlanContent.c_str());
    if (TaskPlanJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): Failed to parse task plan json (" << single_cron_plan_task.id_task_plan << ")!");
        return false;
    }

    cJSON* value_express = cJSON_GetObjectItem(TaskPlanJson, "express");
    if (value_express == NULL || !cJSON_IsArray(value_express))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): Failed to get value_express from TaskPlanJson (" << single_cron_plan_task.id_task_plan << ")!");
        cJSON_Delete(TaskPlanJson);
        return false;
    }

    size_t ExpressArraySize = cJSON_GetArraySize(value_express);

    if (ExpressArraySize == 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): ExpressArraySize is 0 (" << single_cron_plan_task.id_task_plan << ")!");
        cJSON_Delete(TaskPlanJson);
        return false;
    }

    for (size_t i = 0; i < ExpressArraySize; i++)
    {
        cJSON* express_temp = cJSON_GetArrayItem(value_express, i);
        if (express_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] GetCronPlanTaskInfo(): Failed to get express_temp (" << single_cron_plan_task.id_task_plan << ")!");
            cJSON_Delete(TaskPlanJson);
            return false;
        }

        Cron_Analyse::CronTask cron_task_temp;
        cron_task_temp.expr_str = express_temp->valuestring;
        single_cron_plan_task.cron_task.push_back(cron_task_temp);
    }

    cJSON_Delete(TaskPlanJson);

    return true;
}

void Task_Execute::CronPlanTaskStart(CronPlanTask& single_cron_plan_task)
{   
    std::string plan_task_status = "e_started";
    robot_msgs::srv::CtrlModeQuery::Request::SharedPtr CMQ_request;
    auto CMQ_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::CtrlModeQuery>::SharedFuture>();
    auto CMQ_res_callback = [this, CMQ_request, CMQ_shared_future_ptr, &plan_task_status](rclcpp::Client<robot_msgs::srv::CtrlModeQuery>::SharedFuture future)
    {
        *CMQ_shared_future_ptr = future;
        auto response = future.get();
        if (response->runtime_ctrl_mode)
        {
            plan_task_status = "e_waiting";
        }
    };
    
    auto CMQ_future = CtrlModeQuery_client->async_send_request(CMQ_request, CMQ_res_callback);
    *CMQ_shared_future_ptr = CMQ_future.future;
    
    auto CMQ_timer = this->create_wall_timer(std::chrono::milliseconds(500),
        [this, CMQ_shared_future_ptr, CMQ_request, single_cron_plan_task]()
        {
            if (CMQ_shared_future_ptr->valid() &&
                CMQ_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): Failed to call CtrlModeQuery_service (" << single_cron_plan_task.id_task_plan << ")!");
                return;
            }
        });

    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): Can't open TaskInstanceIndex.json (" << single_cron_plan_task.id_task_plan << ")!");
        return;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): Failed to parse TaskInstanceIndexJson (" << single_cron_plan_task.id_task_plan << ")!");
        return;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): Failed to get task_instance_index_array (" << single_cron_plan_task.id_task_plan << ")!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);

    std::string task_ticket = generateRandomUUID();

    std::string start_time;
    {
        auto now = std::chrono::system_clock::now();
        std::time_t time_now = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm = *std::localtime(&time_now);
        std::ostringstream oss;
        oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
        start_time = oss.str();
    }

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_execute_flag == "execute" || task_execute_infos[i].task_execute_flag == "pause_auto" || task_execute_infos[i].task_execute_flag == "pause_manual")
        {
            plan_task_status = "e_waiting";
            break;
        }
    }

    if (TaskInstanceIndexArraySize == 0)
    {
        cJSON* task_instance_index_new = cJSON_CreateObject();
        cJSON_AddStringToObject(task_instance_index_new, "id_task_info", single_cron_plan_task.id_task_info.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_name", single_cron_plan_task.task_name.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_type", single_cron_plan_task.task_type.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_ticket", task_ticket.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_execute_type", "plan_task");
        cJSON_AddStringToObject(task_instance_index_new, "id_task_plan", single_cron_plan_task.id_task_plan.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "status", plan_task_status.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "start_time", start_time.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "end_time", "");
        cJSON_AddNumberToObject(task_instance_index_new, "execute_time", 0);
        cJSON_AddNumberToObject(task_instance_index_new, "point_count", 0);

        cJSON_AddItemToArray(task_instance_index_array, task_instance_index_new);

        char* temp_str = cJSON_Print(TaskInstanceIndexJson);
        cJSON_Delete(TaskInstanceIndexJson);

        if (!temp_str)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): cJSON_Print(TaskInstanceIndexJson) error (" << single_cron_plan_task.id_task_plan << ")!");
            return;
        }

        std::string TaskInstanceIndexContentNew = temp_str;
        free(temp_str);

        std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
        if (!TaskInstanceIndexFileWrite)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): Can't open TaskInstanceIndex.json (" << single_cron_plan_task.id_task_plan << ")!");
            return;
        }

        TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

        if (!TaskInstanceIndexFileWrite.good())
        {
            TaskInstanceIndexFileWrite.close();

            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): Error: Failed to write to TaskInstanceIndex.json (" << single_cron_plan_task.id_task_plan << ")!");
            return;
        }

        TaskInstanceIndexFileWrite.close();
    }
    else
    {
        std::vector<std::string> TaskTicketArray(TaskInstanceIndexArraySize);
        for (size_t i = 0; i < TaskInstanceIndexArraySize; i++)
        {
            cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
            if (task_instance_index_temp == NULL)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): Failed to get task_instance_index_temp (" << single_cron_plan_task.id_task_plan << ")!");
                cJSON_Delete(TaskInstanceIndexJson);
                return;
            }

            cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
            if (task_ticket_temp == NULL)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): Failed to get task_ticket_temp (" << single_cron_plan_task.id_task_plan << ")!");
                cJSON_Delete(TaskInstanceIndexJson);
                return;
            }

            TaskTicketArray[i] = task_ticket_temp->valuestring;
        }

        bool task_ticket_flag = true;
        while(task_ticket_flag)
        {
            size_t cnt = 0;
            bool loop_flag = true;
            for(size_t i = 0; i < TaskInstanceIndexArraySize && loop_flag; i++)
            {
                cnt++;
                if (task_ticket == TaskTicketArray[i])
                {
                    task_ticket = generateRandomUUID();
                    loop_flag = false;
                }
            }

            if (cnt == TaskInstanceIndexArraySize)
            {
                task_ticket_flag = false;
            }
        }

        cJSON* task_instance_index_new = cJSON_CreateObject();
        cJSON_AddStringToObject(task_instance_index_new, "id_task_info", single_cron_plan_task.id_task_info.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_name", single_cron_plan_task.task_name.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_type", single_cron_plan_task.task_type.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_ticket", task_ticket.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "task_execute_type", "plan_task");
        cJSON_AddStringToObject(task_instance_index_new, "id_task_plan", single_cron_plan_task.id_task_plan.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "status", plan_task_status.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "start_time", start_time.c_str());
        cJSON_AddStringToObject(task_instance_index_new, "end_time", "");
        cJSON_AddNumberToObject(task_instance_index_new, "execute_time", 0);
        cJSON_AddNumberToObject(task_instance_index_new, "point_count", 0);

        cJSON_AddItemToArray(task_instance_index_array, task_instance_index_new);

        char* temp_str = cJSON_Print(TaskInstanceIndexJson);
        cJSON_Delete(TaskInstanceIndexJson);

        if (!temp_str)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): cJSON_Print(TaskInstanceIndexJson) error (" << single_cron_plan_task.id_task_plan << ")!");
            return;
        }

        std::string TaskInstanceIndexContentNew = temp_str;
        free(temp_str);

        std::ofstream TaskInstanceIndexFileWrite(TaskInstanceIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
        if (!TaskInstanceIndexFileWrite)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): Can't open TaskInstanceIndex.json (" << single_cron_plan_task.id_task_plan << ")!");
            return;
        }

        TaskInstanceIndexFileWrite << TaskInstanceIndexContentNew;

        if (!TaskInstanceIndexFileWrite.good())
        {
            TaskInstanceIndexFileWrite.close();

            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] CronPlanTaskStart(): Error: Failed to write to TaskInstanceIndex.json (" << single_cron_plan_task.id_task_plan << ")!");
            return;
        }

        TaskInstanceIndexFileWrite.close();
    }

    task_execute_info task_execute_info_;
    task_execute_info_.task_execute_type = "plan_task";
    task_execute_info_.task_ticket = task_ticket;
    task_execute_info_.id_task_info = single_cron_plan_task.id_task_info;
    if (plan_task_status == "e_started")
    {
        task_execute_info_.task_execute_flag = "execute";
    }
    else if (plan_task_status == "e_waiting")
    {
        task_execute_info_.task_execute_flag = "waiting";
    }
    task_execute_info_.task_type = single_cron_plan_task.task_type;
    task_execute_info_.task_name = single_cron_plan_task.task_name;
    task_execute_info_.id_task_plan = single_cron_plan_task.id_task_plan;
    task_execute_info_.point_count = 0;
    task_execute_info_.execute_monitor_point_lock = false;
    task_execute_info_.execute_robot_home_lock = false;
    task_execute_info_.execute_stop_flag = false;
    task_execute_infos.push_back(task_execute_info_);

    std::thread task_execute_thread(&Task_Execute::TaskExecute, this, task_ticket);
    task_execute_thread.detach();

    if (plan_task_status == "e_started")
    {
        robot_msgs::msg::InspectTaskExecutionStart InspectTaskExecutionStartMsg;
        InspectTaskExecutionStartMsg.id_task_info = id_device + "-task-info-" + single_cron_plan_task.id_task_info;
        InspectTaskExecutionStartMsg.id_task_plan = id_device + "-task-plan-" + single_cron_plan_task.id_task_plan;
        InspectTaskExecutionStartMsg.task_ticket = id_device + "-task-ticket-" + task_ticket;
        InspectTaskExecutionStartMsg.start_time = start_time;

        TaskStartOrFinishCache TaskStartOrFinishCache_temp;
        TaskStartOrFinishCache_temp.msg_send_time = std::time(nullptr);
        TaskStartOrFinishCache_temp.response_flag = false;
        TaskStartOrFinishCache_temp.start_or_finish = "start";
        TaskStartOrFinishCache_temp.InspectTaskExecutionStartMsg = InspectTaskExecutionStartMsg;
        TaskStartOrFinishCache_vector.push_back(TaskStartOrFinishCache_temp);

        InspectTaskExecutionStart_pub->publish(InspectTaskExecutionStartMsg);
    }
}

/********************************************************任务执行初始化*************************************************************/
void Task_Execute::TaskExecuteInit()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecuteInit() Start!");

    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecuteInit(): Can't open TaskInstanceIndex.json!");
        return;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecuteInit(): Failed to parse TaskInstanceIndexJson!");
        return;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecuteInit(): Failed to get task_instance_index_array!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);

    if (TaskInstanceIndexArraySize == 0)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecuteInit() end with no task!");
        cJSON_Delete(TaskInstanceIndexJson);
        return;
    }

    std::vector<task_execute_info> task_execute_infos_temp;

    for (size_t i = 0; i < TaskInstanceIndexArraySize; i++)
    {
        cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
        if (task_instance_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecuteInit(): Failed to get task_instance_index_temp!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        cJSON* status_temp = cJSON_GetObjectItem(task_instance_index_temp, "status");
        if (status_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecuteInit(): Failed to get status_temp!");
            cJSON_Delete(TaskInstanceIndexJson);
            return;
        }

        std::string status_temp_str = status_temp->valuestring;
        if (status_temp_str == "e_started" || 
            status_temp_str == "e_paused_manual" || 
            status_temp_str == "e_paused_auto" || 
            status_temp_str == "e_waiting")
        {
            cJSON* task_execute_type_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_execute_type");
            cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
            cJSON* id_task_info_temp = cJSON_GetObjectItem(task_instance_index_temp, "id_task_info");
            cJSON* task_type_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_type");
            cJSON* task_name_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_name");
            cJSON* point_count_temp = cJSON_GetObjectItem(task_instance_index_temp, "point_count");

            if (task_execute_type_temp == NULL || 
                task_ticket_temp == NULL || 
                id_task_info_temp == NULL || 
                task_type_temp == NULL || 
                task_name_temp == NULL || 
                point_count_temp == NULL)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecuteInit(): Failed to get task_execute_type_temp or task_ticket_temp or id_task_info_temp or task_type_temp or task_name_temp or point_count_temp!");
                cJSON_Delete(TaskInstanceIndexJson);
                return;
            }

            cJSON* id_task_plan_temp;
            std::string task_execute_type_temp_str = task_execute_type_temp->valuestring;
            if (task_execute_type_temp_str == "plan_task")
            {
                id_task_plan_temp = cJSON_GetObjectItem(task_instance_index_temp, "id_task_plan");
                if (id_task_plan_temp == NULL)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecuteInit(): Failed to get id_task_plan_temp!");
                    cJSON_Delete(TaskInstanceIndexJson);
                    return;
                }
            }

            task_execute_info task_execute_info_temp;
            task_execute_info_temp.task_execute_type = task_execute_type_temp_str;
            task_execute_info_temp.task_ticket = task_ticket_temp->valuestring;
            task_execute_info_temp.id_task_info = id_task_info_temp->valuestring;
            if (status_temp_str == "e_started")
            {
                task_execute_info_temp.task_execute_flag = "execute";
            }
            else if (status_temp_str == "e_paused_manual")
            {
                task_execute_info_temp.task_execute_flag = "pause_manual";
            }
            else if (status_temp_str == "e_paused_auto")
            {
                task_execute_info_temp.task_execute_flag = "pause_auto";
            }
            else if (status_temp_str == "e_waiting")
            {
                task_execute_info_temp.task_execute_flag = "waiting";
            }
            task_execute_info_temp.task_type = task_type_temp->valuestring;
            task_execute_info_temp.task_name = task_name_temp->valuestring;
            task_execute_info_temp.point_count = point_count_temp->valueint;
            if (task_execute_type_temp_str == "plan_task")
            {
                task_execute_info_temp.id_task_plan = id_task_plan_temp->valuestring;
            }

            if (status_temp_str == "e_started")
            {
                task_execute_info_temp.execute_monitor_point_lock = false;
                task_execute_info_temp.execute_robot_home_lock = false;
                task_execute_info_temp.execute_stop_flag = false;
            }
            else if (status_temp_str == "e_paused_manual")
            {
                task_execute_info_temp.execute_monitor_point_lock = false;
                task_execute_info_temp.execute_robot_home_lock = false;
                task_execute_info_temp.pause_manual_execute_fitst_time = false;
                task_execute_info_temp.execute_stop_flag = true;
            }
            else if (status_temp_str == "e_paused_auto")
            {
                task_execute_info_temp.execute_monitor_point_lock = false;
                task_execute_info_temp.execute_robot_home_lock = false;
                task_execute_info_temp.pause_auto_execute_first_time = false;
                task_execute_info_temp.execute_stop_flag = true;
            }
            else if (status_temp_str == "e_waiting")
            {
                task_execute_info_temp.execute_monitor_point_lock = false;
                task_execute_info_temp.execute_robot_home_lock = false;
                task_execute_info_temp.execute_stop_flag = false;
            }

            task_execute_infos_temp.push_back(task_execute_info_temp);
        }
    }

    cJSON_Delete(TaskInstanceIndexJson);

    for (size_t i = 0; i < task_execute_infos_temp.size(); i++)
    {
        if (task_execute_infos_temp[i].task_execute_flag == "execute")
        {
            task_execute_infos.push_back(task_execute_infos_temp[i]);
            std::thread task_execute_thread(&Task_Execute::TaskExecute, this, task_execute_infos_temp[i].task_ticket);
            task_execute_thread.detach();
        }
    }

    for (size_t i = 0; i < task_execute_infos_temp.size(); i++)
    {
        if (task_execute_infos_temp[i].task_execute_flag == "pause_auto")
        {
            task_execute_infos.push_back(task_execute_infos_temp[i]);
            std::thread task_execute_thread(&Task_Execute::TaskExecute, this, task_execute_infos_temp[i].task_ticket);
            task_execute_thread.detach();
        }
    }

    for (size_t i = 0; i < task_execute_infos_temp.size(); i++)
    {
        if (task_execute_infos_temp[i].task_execute_flag == "pause_manual")
        {
            task_execute_infos.push_back(task_execute_infos_temp[i]);
            std::thread task_execute_thread(&Task_Execute::TaskExecute, this, task_execute_infos_temp[i].task_ticket);
            task_execute_thread.detach();
        }
    }

    for (size_t i = 0; i < task_execute_infos_temp.size(); i++)
    {
        if (task_execute_infos_temp[i].task_execute_flag == "waiting")
        {
            task_execute_infos.push_back(task_execute_infos_temp[i]);
            std::thread task_execute_thread(&Task_Execute::TaskExecute, this, task_execute_infos_temp[i].task_ticket);
            task_execute_thread.detach();
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskExecuteInit() end!");
}

/********************************************************服务中断函数*************************************************************/

bool Task_Execute::TaskExecuteStatusQueryHandle(robot_msgs::srv::TaskExecuteStatusQuery::Request::SharedPtr req, robot_msgs::srv::TaskExecuteStatusQuery::Response::SharedPtr res)
{
    (void) req;
    res->task_execute = false;
    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_execute_flag == "execute")
        {
            res->task_execute = true;
            res->task_ticket = id_device + "-task-ticket-" + task_execute_infos[i].task_ticket;
            break;
        }
    }

    return true;
}

bool Task_Execute::BatteryChargePauseTaskHandle(robot_msgs::srv::BatteryChargePauseTask::Request::SharedPtr req, robot_msgs::srv::BatteryChargePauseTask::Response::SharedPtr res)
{
    (void) req;
    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_execute_flag == "execute")
        {
            task_ticket_pause_by_battery_charge = task_execute_infos[i].task_ticket;
            task_execute_infos[i].task_execute_flag = "pause_auto";
            task_execute_infos[i].pause_auto_execute_first_time = false;
            task_execute_infos[i].execute_stop_flag = true;

            std_msgs::msg::Bool ArmCtrlStopMsg;
            ArmCtrlStopMsg.data = true;
            ArmCtrlStopCmd_pub->publish(ArmCtrlStopMsg);

            if (device_type == "e_robot_track")
            {
                std_msgs::msg::Int32 TrackMotorStopCtrlMsg;
                TrackMotorStopCtrlMsg.data = 0;
                TrackMotorStopCtrl_pub->publish(TrackMotorStopCtrlMsg);
            }

            if (device_type == "e_robot_crawler")
            {
                std_msgs::msg::Bool StopCtrlMsg;
                StopCtrlMsg.data = true;
                MotorStopCtrl_pub->publish(StopCtrlMsg);
                LiftCtrlStopCmd_pub->publish(StopCtrlMsg);
            }

            if (device_type == "e_robot_quadrupedal")
            {
                std_msgs::msg::Bool StopCtrlMsg;
                StopCtrlMsg.data = true;
                MotorStopCtrl_pub->publish(StopCtrlMsg);
            }
        }
    }

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_ticket_pause_by_battery_charge)
        {
            if (ExecuteRobotHomeStatus(task_execute_infos[i]))
            {
                if (TaskPauseAuto(task_execute_infos[i].task_ticket))
                {
                    res->execute_success = true;
                }
                else
                {
                    res->execute_success = false;
                }
            }
            else
            {
                res->execute_success = false;
            }
        }
    }

    return true;
}

bool Task_Execute::BatteryChargeStartTaskHandle(robot_msgs::srv::BatteryChargeStartTask::Request::SharedPtr req, robot_msgs::srv::BatteryChargeStartTask::Response::SharedPtr res)
{
    (void) req;
    if (!task_ticket_pause_by_battery_charge.empty())
    {
        for (size_t i = 0; i < task_execute_infos.size(); i++)
        {
            if (task_execute_infos[i].task_ticket == task_ticket_pause_by_battery_charge)
            {
                task_execute_infos[i].task_execute_flag = "execute";
                task_execute_infos[i].execute_stop_flag = false;
            }
        }

        if (TaskStartAuto(task_ticket_pause_by_battery_charge))
        {
            res->execute_success = true;
            task_ticket_pause_by_battery_charge = "";
        }
        else
        {
            res->execute_success = false;
        }
    }
    else
    {
        res->execute_success = true;
    }

    return true;
}

bool Task_Execute::ModeChangePauseTaskHandle(robot_msgs::srv::ModeChangePauseTask::Request::SharedPtr req, robot_msgs::srv::ModeChangePauseTask::Response::SharedPtr res)
{
    (void) req;
    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_execute_flag == "execute")
        {
            task_ticket_pause_by_mode_change = task_execute_infos[i].task_ticket;
            task_execute_infos[i].task_execute_flag = "pause_auto";
            task_execute_infos[i].pause_auto_execute_first_time = false;
            task_execute_infos[i].execute_stop_flag = true;

            std_msgs::msg::Bool ArmCtrlStopMsg;
            ArmCtrlStopMsg.data = true;
            ArmCtrlStopCmd_pub->publish(ArmCtrlStopMsg);

            if (device_type == "e_robot_track")
            {
                std_msgs::msg::Int32 TrackMotorStopCtrlMsg;
                TrackMotorStopCtrlMsg.data = 0;
                TrackMotorStopCtrl_pub->publish(TrackMotorStopCtrlMsg);
            }

            if (device_type == "e_robot_crawler")
            {
                std_msgs::msg::Bool StopCtrlMsg;
                StopCtrlMsg.data = true;
                MotorStopCtrl_pub->publish(StopCtrlMsg);
                LiftCtrlStopCmd_pub->publish(StopCtrlMsg);
            }

            if (device_type == "e_robot_quadrupedal")
            {
                std_msgs::msg::Bool StopCtrlMsg;
                StopCtrlMsg.data = true;
                MotorStopCtrl_pub->publish(StopCtrlMsg);
            }
        }
    }

    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_ticket == task_ticket_pause_by_mode_change)
        {
            if (ExecuteRobotHomeStatus(task_execute_infos[i]))
            {
                if (TaskPauseAuto(task_execute_infos[i].task_ticket))
                {
                    res->execute_success = true;
                }
                else
                {
                    res->execute_success = false;
                }
            }
            else
            {
                res->execute_success = false;
            }
        }
    }

    return true;
}

bool Task_Execute::ModeChangeStartTaskHandle(robot_msgs::srv::ModeChangeStartTask::Request::SharedPtr req, robot_msgs::srv::ModeChangeStartTask::Response::SharedPtr res)
{
    (void) req;
    if (!task_ticket_pause_by_mode_change.empty())
    {
        for (size_t i = 0; i < task_execute_infos.size(); i++)
        {
            if (task_execute_infos[i].task_ticket == task_ticket_pause_by_mode_change)
            {
                task_execute_infos[i].task_execute_flag = "execute";
                task_execute_infos[i].execute_stop_flag = false;
            }

        }

        if (TaskStartAuto(task_ticket_pause_by_mode_change))
        {
            res->execute_success = true;
            task_ticket_pause_by_mode_change = "";
        }
        else
        {
            res->execute_success = false;
        }
    }
    else
    {
        res->execute_success = true;
    }

    return true;
}

bool Task_Execute::AvoidObstaclesPauseTaskHandle(robot_msgs::srv::AvoidObstaclesPauseTask::Request::SharedPtr req, robot_msgs::srv::AvoidObstaclesPauseTask::Response::SharedPtr res)
{
    (void) req;
    for (size_t i = 0; i < task_execute_infos.size(); i++)
    {
        if (task_execute_infos[i].task_execute_flag == "execute")
        {
            task_ticket_pause_by_avoid_obstacle = task_execute_infos[i].task_ticket;
            task_execute_infos[i].task_execute_flag = "pause_auto";
            task_execute_infos[i].pause_auto_execute_first_time = false;
            task_execute_infos[i].execute_stop_flag = true;

            std_msgs::msg::Bool ArmCtrlStopMsg;
            ArmCtrlStopMsg.data = true;
            ArmCtrlStopCmd_pub->publish(ArmCtrlStopMsg);

            if (device_type == "e_robot_track")
            {
                std_msgs::msg::Int32 TrackMotorStopCtrlMsg;
                TrackMotorStopCtrlMsg.data = 0;
                TrackMotorStopCtrl_pub->publish(TrackMotorStopCtrlMsg);
            }

            if (device_type == "e_robot_crawler")
            {
                std_msgs::msg::Bool StopCtrlMsg;
                StopCtrlMsg.data = true;
                MotorStopCtrl_pub->publish(StopCtrlMsg);
                LiftCtrlStopCmd_pub->publish(StopCtrlMsg);
            }

            if (device_type == "e_robot_quadrupedal")
            {
                std_msgs::msg::Bool StopCtrlMsg;
                StopCtrlMsg.data = true;
                MotorStopCtrl_pub->publish(StopCtrlMsg);
            }
        }
    }

    if (TaskPauseAuto(task_ticket_pause_by_avoid_obstacle))
    {
        res->execute_success = true;
    }
    else
    {
        res->execute_success = false;
    }

    return true;
}

bool Task_Execute::AvoidObstaclesStartTaskHandle(robot_msgs::srv::AvoidObstaclesStartTask::Request::SharedPtr req, robot_msgs::srv::AvoidObstaclesStartTask::Response::SharedPtr res)
{
    (void) req;
    if (!task_ticket_pause_by_avoid_obstacle.empty())
    {
        for (size_t i = 0; i < task_execute_infos.size(); i++)
        {
            if (task_execute_infos[i].task_ticket == task_ticket_pause_by_avoid_obstacle)
            {
                task_execute_infos[i].task_execute_flag = "execute";
                task_execute_infos[i].execute_stop_flag = false;
            }
        }

        if (TaskStartAuto(task_ticket_pause_by_avoid_obstacle))
        {
            res->execute_success = true;
            task_ticket_pause_by_avoid_obstacle = "";
        }
        else
        {
            res->execute_success = false;
        }
    }
    else
    {
        res->execute_success = true;
    }

    return true;
}

bool Task_Execute::TaskStartExecuteCheckRobotHome()
{
    bool execute_robot_home_lock_local = true;
    while (execute_robot_home_lock_local)
    {
        bool execute_robot_home_locks = false;
        for (size_t i = 0; i < task_execute_infos.size(); i++)
        {
            execute_robot_home_locks = execute_robot_home_locks || task_execute_infos[i].execute_robot_home_lock;
        }

        execute_robot_home_lock_local = execute_robot_home_locks;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    bool arm_home_flag_local = true;
    bool arm_home_flag = true;

    if (device_type == "e_robot_crawler")
    {
        robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;
        arm_request->pos_key = "poshome";

        auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
        auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_home_flag, &arm_home_flag_local](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
        {
            *arm_shared_future_ptr = future;
            auto response = future.get();
            if (!response->execute_success)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartExecuteCheckRobotHome(): Arm failed to move to target pos(" << arm_request->pos_key << ")!");
                arm_home_flag = false;
                arm_home_flag_local = false;
            }
            else
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartExecuteCheckRobotHome(): Arm successfully move to target pos: " << arm_request->pos_key << "!");
                
                // 机械臂成功后调用升降服务
                robot_msgs::srv::LiftCtrlSrv::Request::SharedPtr lift_request;
                lift_request->position = 0.0;
                
                auto lift_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture>();
                auto lift_res_callback = [this, lift_request, &arm_home_flag, &arm_home_flag_local](rclcpp::Client<robot_msgs::srv::LiftCtrlSrv>::SharedFuture future)
                {
                    auto response = future.get();
                    if (!response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartExecuteCheckRobotHome(): Lift failed to move to target pos(" << lift_request->position << ")!");
                        arm_home_flag = false;
                        arm_home_flag_local = false;
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartExecuteCheckRobotHome(): Lift successfully move to target pos: " << lift_request->position << "!");
                    }
                };
                
                auto lift_future = LiftCtrlSrv_client->async_send_request(lift_request, lift_res_callback);
                *lift_shared_future_ptr = lift_future.future;
                
                auto lift_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                    [this, lift_shared_future_ptr, &arm_home_flag, &arm_home_flag_local]()
                    {
                        if (lift_shared_future_ptr->valid() &&
                            lift_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                        {
                            RCLCPP_ERROR(this->get_logger(), "LiftCtrlSrv call timeout!");
                            arm_home_flag = false;
                            arm_home_flag_local = false;
                        }
                    });
            }
        };
        
        auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
        *arm_shared_future_ptr = arm_future.future;
        
        auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
            [this, arm_shared_future_ptr, &arm_home_flag, &arm_home_flag_local]()
            {
                if (arm_shared_future_ptr->valid() &&
                    arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                {
                    RCLCPP_ERROR(this->get_logger(), "ArmCtrlSrv call timeout!");
                    arm_home_flag = false;
                    arm_home_flag_local = false;
                }
            });
    }
    else if (device_type == "e_robot_track" || device_type == "e_robot_crawler")
    {
        robot_msgs::srv::ArmCtrlSrv::Request::SharedPtr arm_request;
        arm_request->pos_key = "poshome";

        auto arm_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture>();
        auto arm_res_callback = [this, arm_request, arm_shared_future_ptr, &arm_home_flag_local](rclcpp::Client<robot_msgs::srv::ArmCtrlSrv>::SharedFuture future)
        {
            *arm_shared_future_ptr = future;
            auto response = future.get();
            if (!response->execute_success)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartExecuteCheckRobotHome(): Arm failed to move to target pos(" << arm_request->pos_key << ")!");
                arm_home_flag_local = false;
            }
            else
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartExecuteCheckRobotHome(): Arm successfully move to target pos: " << arm_request->pos_key << "!");
            }
        };
        
        auto arm_future = ArmCtrlSrv_client->async_send_request(arm_request, arm_res_callback);
        *arm_shared_future_ptr = arm_future.future;
        
        auto arm_timer = this->create_wall_timer(std::chrono::milliseconds(500),
            [this, arm_shared_future_ptr, &arm_home_flag_local]()
            {
                if (arm_shared_future_ptr->valid() &&
                    arm_shared_future_ptr->wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
                {
                    RCLCPP_ERROR(this->get_logger(), "ArmCtrlSrv call timeout!");
                    arm_home_flag_local = false;
                }
            });
    }

    auto start_time = std::chrono::steady_clock::now();
    while (arm_home_flag_local && std::chrono::steady_clock::now() - start_time < std::chrono::seconds(30))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return arm_home_flag_local;
}

bool Task_Execute::TaskStartOrFinishResponseHandle(robot_msgs::srv::TaskStartOrFinishResponse::Request::SharedPtr req, robot_msgs::srv::TaskStartOrFinishResponse::Response::SharedPtr res)
{
    if (req->start_or_finish == "start")
    {
        bool TaskStartOrFinishCache_exist = false;
        for (size_t i = 0; i < TaskStartOrFinishCache_vector.size(); i++)
        {
            if (TaskStartOrFinishCache_vector[i].start_or_finish == req->start_or_finish)
            {
                if (TaskStartOrFinishCache_vector[i].InspectTaskExecutionStartMsg.task_ticket == req->task_ticket)
                {
                    TaskStartOrFinishCache_vector[i].response_flag = true;
                    TaskStartOrFinishCache_exist = true;
                    break;
                }
            }
        }

        if (!TaskStartOrFinishCache_exist)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartOrFinishResponseHandle(): Start task_ticket[" << req->task_ticket << "] is not exist in TaskStartOrFinishCache_vector.");
        }

        res->task_ticket_exist = TaskStartOrFinishCache_exist;
        return true;
    }
    else if (req->start_or_finish == "complete")
    {
        bool TaskStartOrFinishCache_exist = false;
        for (size_t i = 0; i < TaskStartOrFinishCache_vector.size(); i++)
        {
            if (TaskStartOrFinishCache_vector[i].start_or_finish == req->start_or_finish)
            {
                if (TaskStartOrFinishCache_vector[i].InspectTaskExecutionCompleteMsg.task_ticket == req->task_ticket)
                {
                    TaskStartOrFinishCache_vector[i].response_flag = true;
                    TaskStartOrFinishCache_exist = true;
                    break;
                }
            }
        }

        if (!TaskStartOrFinishCache_exist)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartOrFinishResponseHandle(): Complete task_ticket[" << req->task_ticket << "] is not exist in TaskStartOrFinishCache_vector.");
        }

        res->task_ticket_exist = TaskStartOrFinishCache_exist;
        return true;
    }
    else
    {
        return false;
    }
}

void Task_Execute::TaskStartOrFinishResponseCheck()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskStartOrFinishResponseCheck Thread Start!");

    while(rclcpp::ok())
    {
        {
        struct timespec nowts;
        clock_gettime(CLOCK_MONOTONIC, &nowts);
        time_t next_sec = nowts.tv_sec + 1;
        // struct timespec ts = {.tv_sec = next_sec, .tv_nsec = 0};
        struct timespec ts = {next_sec, 0};
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        }

        std::vector<size_t> TaskStartOrFinishCache_vector_erase_idx;

        for (size_t i = 0; i < TaskStartOrFinishCache_vector.size(); i++)
        {
            if (TaskStartOrFinishCache_vector[i].response_flag)
            {
                TaskStartOrFinishCache_vector_erase_idx.push_back(i);
            }
            else
            {
                std::time_t current_time = std::time(nullptr);

                if ((current_time - TaskStartOrFinishCache_vector[i].msg_send_time) >= TaskStartOrFinishResponseDuration * 60)
                {
                    TaskStartOrFinishCache_vector[i].msg_send_time = current_time;

                    if (TaskStartOrFinishCache_vector[i].start_or_finish == "start")
                    {
                        InspectTaskExecutionStart_pub->publish(TaskStartOrFinishCache_vector[i].InspectTaskExecutionStartMsg);
                    }
                    else if (TaskStartOrFinishCache_vector[i].start_or_finish == "complete")
                    {
                        InspectTaskExecutionComplete_pub->publish(TaskStartOrFinishCache_vector[i].InspectTaskExecutionCompleteMsg);
                    }
                }
            }
        }

        for (auto it = TaskStartOrFinishCache_vector_erase_idx.rbegin(); it != TaskStartOrFinishCache_vector_erase_idx.rend(); ++it)
        {
            TaskStartOrFinishCache_vector.erase(TaskStartOrFinishCache_vector.begin() + *it);
        }
    }
}
