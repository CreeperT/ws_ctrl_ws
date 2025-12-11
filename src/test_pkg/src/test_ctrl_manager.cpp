#include "test_ctrl_manager.h"
using std::placeholders::_1;
using std::placeholders::_2;

Ctrl_Manager::Ctrl_Manager(const char* node_name) : rclcpp::Node(node_name)
{
    if(DeviceParamInit())
    {
        NodePublisherInit();
        NodeSubscriberInit();
        NodeServiceServerInit();
        NodeServiceClientInit();
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
    RCLCPP_INFO(this->get_logger(), "Publisher init finished!");
}
void Ctrl_Manager::NodeSubscriberInit()
{
    RCLCPP_INFO(this->get_logger(), "Subscriber init finished!");
}

void Ctrl_Manager::NodeServiceServerInit()
{    
    server_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    ChangeCtrlModeCmd_server = this->create_service<robot_msgs::srv::ChangeCtrlModeCmd>(
        "ChangeCtrlModeCmd_service", std::bind(&Ctrl_Manager::ChangeCtrlModeCmdHandle, this, _1, _2),
        rmw_qos_profile_services_default, server_cb_group
    );

    RCLCPP_INFO(this->get_logger(), "Service server init finished!");
}

void Ctrl_Manager::NodeServiceClientInit()
{
    client_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    TaskExecuteStatusQuery_client = this->create_client<robot_msgs::srv::TaskExecuteStatusQuery>("TaskExecuteStatusQuery_service", rmw_qos_profile_services_default, client_cb_group);

    ModeChangePauseTask_client = this->create_client<robot_msgs::srv::ModeChangePauseTask>("ModeChangePauseTask_service", rmw_qos_profile_services_default, client_cb_group);

    ModeChangeStartTask_client = this->create_client<robot_msgs::srv::ModeChangeStartTask>("ModeChangeStartTask_service", rmw_qos_profile_services_default, client_cb_group);

    RCLCPP_INFO(this->get_logger(), "Service client init finished!");
}

void Ctrl_Manager::NodeSpinnerStartup()
{
    // spinner_thread_ = std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());
    executor.spin();
}

bool Ctrl_Manager::ChangeCtrlModeCmdHandle(const robot_msgs::srv::ChangeCtrlModeCmd::Request::SharedPtr req,
                                                    robot_msgs::srv::ChangeCtrlModeCmd::Response::SharedPtr res)
{
    bool flag = false;

    auto shared_res = res;
    auto res_flag = false;

    RCLCPP_INFO(this->get_logger(), "ChangeCtrlModeCmdHandle: ctrl_mode: %d, req->ctrl_mode: %d", ctrl_mode, req->ctrl_mode);
    if (req->ctrl_mode == ctrl_mode)
    {
        res->execute_success = true;
        return true;
    }

    if (ctrl_mode == 2 && battery_charge_lock)
    {
        res->execute_success = false;
        return true;
    }

    if (req->ctrl_mode == 1)
    {
        RCLCPP_INFO(this->get_logger(), "req->ctrl_mode is 1");
        if (ctrl_mode == 0)
        {
            RCLCPP_INFO(this->get_logger(), "ctrl_mode is 0");
            auto task_execute_local = std::make_shared<std::atomic<bool>>(false);
            auto TESQ_request = std::make_shared<robot_msgs::srv::TaskExecuteStatusQuery::Request>();

            while (!TaskExecuteStatusQuery_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "TaskExecuteStatusQuery Service not available, waiting again...");
            }

            auto TESQ_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::TaskExecuteStatusQuery>::SharedFuture>();
            

            auto TESQ_res_callback = [this, &flag, TESQ_shared_future_ptr, &task_execute_local, req, &res_flag](rclcpp::Client<robot_msgs::srv::TaskExecuteStatusQuery>::SharedFuture future)
            {
                RCLCPP_INFO(this->get_logger(), "get in TESQ_callback");
                *TESQ_shared_future_ptr = future;
                auto response = future.get();
                if (response->task_execute)
                {
                    *task_execute_local = true;
                    RCLCPP_INFO(this->get_logger(), "TESQ response is true, res_flag is %d", res_flag);
                }
                else
                {
                    res_flag = true;
                    ctrl_mode = req->ctrl_mode;
                    operate_manual_last_time = std::time(nullptr);
                    flag = true;
                }
            };

            auto TESQ_future = TaskExecuteStatusQuery_client->async_send_request(TESQ_request, TESQ_res_callback);
            *TESQ_shared_future_ptr = TESQ_future.future;
            auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                [this, &flag, TESQ_shared_future_ptr, &res_flag]()
                                                {
                                                    if (TESQ_shared_future_ptr->valid() &&
                                                        TESQ_shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                    {
                                                        RCLCPP_WARN_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ChangerCtrlModeCmdHandle(): TaskExecuteStatusQuery srv return is false.");
                                                        res_flag = false;
                                                        flag = true;
                                                    }
                                                });

            if (task_execute_local)
            {
                auto MCPT_request = std::make_shared<robot_msgs::srv::ModeChangePauseTask::Request>();
                while (!ModeChangePauseTask_client->wait_for_service(std::chrono::seconds(1)))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                        return false;
                    }
                    RCLCPP_INFO(this->get_logger(), "ModeChangePauseTask Service not available, waiting again...");
                }

                auto MCPT_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ModeChangePauseTask>::SharedFuture>();

                auto MCPT_res_callback = [this, &flag, MCPT_shared_future_ptr, req, &res_flag](rclcpp::Client<robot_msgs::srv::ModeChangePauseTask>::SharedFuture future)
                {
                    RCLCPP_INFO(this->get_logger(), "get in MCPT_callback");
                    *MCPT_shared_future_ptr = future;
                    auto response = future.get();
                    if (response->execute_success)
                    {
                        res_flag = true;
                        ctrl_mode = req->ctrl_mode;
                        operate_manual_last_time = std::time(nullptr);
                        flag = true;
                        RCLCPP_INFO(this->get_logger(), "MCPT response is true, res_flag is %d", res_flag);
                    }
                    else
                    {
                        RCLCPP_WARN_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ChangerCtrlModeCmdHandle(): ModeChangePauseTask execute failed(res.exe_succ is false).");
                        res_flag = false;
                        flag = true;
                    }
                };

                auto MCPT_future = ModeChangePauseTask_client->async_send_request(MCPT_request, MCPT_res_callback);
                *MCPT_shared_future_ptr = MCPT_future.future;   
                auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                    [this, MCPT_shared_future_ptr, &res_flag]()
                                                    {
                                                        if (MCPT_shared_future_ptr->valid() &&
                                                            MCPT_shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                        {
                                                            RCLCPP_WARN_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ChangerCtrlModeCmdHandle(): ModeChangePauseTask execute failed(srv return is false).");
                                                            res_flag = false;
                                                        }
                                                    });

                if (flag)
                {
                    if (res_flag)
                        res->execute_success = true;
                    else
                        res->execute_success = false;
                    RCLCPP_INFO(this->get_logger(), "res->execute_success is %d", res->execute_success);
                    return true;
                }
            }
        }
        else if (ctrl_mode == 2)
        {
            RCLCPP_INFO(this->get_logger(), "ctrl_mode is 2");
            res->execute_success = true;
            ctrl_mode = req->ctrl_mode;
            operate_manual_last_time = std::time(nullptr);
            return true;
        }
    }

    if (req->ctrl_mode == 0)
    {
        RCLCPP_INFO(this->get_logger(), "req->ctrl_mode is 0");
        if (ctrl_mode == 1)
        {
            RCLCPP_INFO(this->get_logger(), "ctrl_mode is 1");
            // if (device_type == "e_robot_quadrupedal")
            // {
            //     std_msgs::msg::Bool QuadrupedalMotorStopCtrlMsg;
            //     QuadrupedalMotorStopCtrlMsg.data = true;
            //     QuadrupedalMotorStopCtrl_pub->publish(QuadrupedalMotorStopCtrlMsg);
            // }

            // std_msgs::msg::Bool ArmCtrlStopMsg;
            // ArmCtrlStopMsg.data = true;
            // ArmCtrlStopCmd_pub->publish(ArmCtrlStopMsg);

            auto MCST_request = std::make_shared<robot_msgs::srv::ModeChangeStartTask::Request>();
            
            while (!ModeChangeStartTask_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "ModeChangeStartTask Service not available, waiting again...");
            }

            auto MCST_future = ModeChangeStartTask_client->async_send_request(MCST_request);
            if (MCST_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
            {
                auto MCST_response = MCST_future.get();
                if (MCST_response->execute_success)
                {
                    res->execute_success = true;
                    ctrl_mode = req->ctrl_mode;
                    RCLCPP_INFO(this->get_logger(), "MCST response is true, res_flag is %d", res->execute_success);
                    return true;
                    
                }
                else
                {
                    RCLCPP_WARN_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ChangerCtrlModeCmdHandle(): ModeChangeStartTask execute failed(res.exe_succ is false).");
                    res->execute_success = false;
                    flag = true;
                }
                
            }
            else
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ChangerCtrlModeCmdHandle(): ModeChangeStartTask execute failed(srv return is false).");
                res_flag = false;
                flag = true;
            }

            RCLCPP_INFO(this->get_logger(), "res_flag is %d", res_flag);

            if (flag)
            {
                if (res_flag)
                    res->execute_success = true;
                else
                    res->execute_success = false;
                RCLCPP_INFO(this->get_logger(), "res->execute_success is %d", res->execute_success);
                return true;
            }
            
        }
        else if (ctrl_mode == 2)
        {
            RCLCPP_INFO(this->get_logger(), "ctrl_mode is 2");
            ctrl_mode = req->ctrl_mode;
            res->execute_success = true;
            return true;
        }
    }

    return true;    
}
