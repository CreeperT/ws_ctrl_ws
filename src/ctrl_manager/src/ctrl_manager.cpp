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

        HeartbeatBag_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        BatteryPercentManage_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        ManualMode2TaskModeAutoCheck_callback_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        HeartbeatBag_timer = this->create_wall_timer(std::chrono::seconds(60), std::bind(&Ctrl_Manager::RobotHeartbeatBagPub, this), HeartbeatBag_callback_group_);

        BatteryPercentManage_timer = this->create_wall_timer(std::chrono::seconds(60), std::bind(&Ctrl_Manager::BatteryPercentManage, this), BatteryPercentManage_callback_group_);

        ManualMode2TaskModeAutoCheck_timer = this->create_wall_timer(std::chrono::seconds(60), std::bind(&Ctrl_Manager::ManualMode2TaskModeAutoCheck, this), ManualMode2TaskModeAutoCheck_callback_group_);
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
    QuadrupedalMotorCtrlNormal_pub = this->create_publisher<robot_msgs::msg::MotorCtrlNormal>("/QuadrupedalMotorCtrlNormal_topic", 10);
    QuadrupedalMotorCtrltoPos_pub = this->create_publisher<robot_msgs::msg::MotorCtrltoPos>("/QuadrupedalMotorCtrltoPos_topic", 10);
    QuadrupedalMotorStopCtrl_pub = this->create_publisher<std_msgs::msg::Bool>("/QuadrupedalMotorStopCtrl_topic", 10);

    ArmCtrlMoveCmd_pub = this->create_publisher<robot_msgs::msg::ArmCtrl>("/ArmMoveCtrl_topic", 10);
    ArmCtrlStopCmd_pub = this->create_publisher<std_msgs::msg::Bool>("/ArmStopCtrl_topic", 10);

    ExtendDevCtrlInternal_pub = this->create_publisher<robot_msgs::msg::ExtendDevCtrl>("/ExtendDevCtrlInternal_topic", 10);
    
    HeartbeatBag_pub = this->create_publisher<robot_msgs::msg::HeartbeatBag>("/HeartbeatBag_topic", 10);

    RCLCPP_INFO(this->get_logger(), "Publisher init finished!");
}

void Ctrl_Manager::NodeSubscriberInit()
{
    MotorCtrlNormalCmd_sub = this->create_subscription<robot_msgs::msg::MotorCtrlNormal>(
        "/MotorCtrlNormal_topic", 10, std::bind(&Ctrl_Manager::MotorCtrlNormalCmdCallback, this, _1));
    MotorCtrltoPosCmd_sub = this->create_subscription<robot_msgs::msg::MotorCtrltoPos>(
        "MotorCtrltoPos_topic", 10, std::bind(&Ctrl_Manager::MotorCtrltoPosCmdCallback, this, _1));
    
    ArmCtrl_sub = this->create_subscription<robot_msgs::msg::ArmCtrl>(
        "ArmCtrl_topic", 10, std::bind(&Ctrl_Manager::ArmCtrlCmdCallback, this, _1));

    ExtendDevCtrlCmd_sub = this->create_subscription<robot_msgs::msg::ExtendDevCtrl>(
        "ExtendDevCtrl_topic", 10, std::bind(&Ctrl_Manager::ExtendDevCtrlCmdCallback, this, _1));

    RobotRuntimePosition_sub = this->create_subscription<robot_msgs::msg::RobotRuntimePosition>(
        "RobotRuntimePosition_topic", 10, std::bind(&Ctrl_Manager::RobotRuntimePositionCallback, this, _1));

    RobotBatteryStatus_sub = this->create_subscription<robot_msgs::msg::RobotBatteryStatus>(
        "RobotBatteryStatus_topic", 10, std::bind(&Ctrl_Manager::RobotBatteryStatusCallback, this, _1));

    TaskFinishAndExecuteBatteryCharge_sub = this->create_subscription<std_msgs::msg::Bool>(
        "TaskFinishAndExecuteBatteryCharge_topic", 10, std::bind(&Ctrl_Manager::TaskFinishAndExecuteBatteryChargeCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Subscriber init finished!");
}

void Ctrl_Manager::NodeServiceServerInit()
{   
    ChangeCtrlModeCmd_server = this->create_service<robot_msgs::srv::ChangeCtrlModeCmd>(
        "ChangeCtrlModeCmd_service", std::bind(&Ctrl_Manager::ChangeCtrlModeCmdHandle, this, _1, _2)
    );

    CtrlModeQuery_server = this->create_service<robot_msgs::srv::CtrlModeQuery>(
        "CtrlModeQuery_service", std::bind(&Ctrl_Manager::CtrlModeQueryHandle, this, _1, _2)
    );

    RCLCPP_INFO(this->get_logger(), "Service server init finished!");
}

void Ctrl_Manager::NodeServiceClientInit()
{    
    BatteryChargeNav_client = this->create_client<robot_msgs::srv::BatteryChargeNav>("BatteryChargeNav_service");

    TaskExecuteStatusQuery_client = this->create_client<robot_msgs::srv::TaskExecuteStatusQuery>("TaskExecuteStatusQuery_service");

    BatteryChargeExecuteProgram_client = this->create_client<robot_msgs::srv::BatteryChargeExecuteProgram>("BatteryChargeExecuteProgram_service");

    BatteryChargePauseTask_client = this->create_client<robot_msgs::srv::BatteryChargePauseTask>("BatteryChargePauseTask_service");

    BatteryChargeStartTask_client = this->create_client<robot_msgs::srv::BatteryChargeStartTask>("BatteryChargeStartTask_service");

    ModeChangePauseTask_client = this->create_client<robot_msgs::srv::ModeChangePauseTask>("ModeChangePauseTask_service");

    ModeChangeStartTask_client = this->create_client<robot_msgs::srv::ModeChangeStartTask>("ModeChangeStartTask_service");

    RCLCPP_INFO(this->get_logger(), "Service client init finished!");
}

void Ctrl_Manager::NodeSpinnerStartup()
{
    // spinner_thread_ = std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());
    executor.spin();
}

/***********************************************机器人控制相关***********************************************/

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
            if (device_type == "e_robot_quadrupedal")
            {
                std_msgs::msg::Bool QuadrupedalMotorStopCtrlMsg;
                QuadrupedalMotorStopCtrlMsg.data = true;
                QuadrupedalMotorStopCtrl_pub->publish(QuadrupedalMotorStopCtrlMsg);
            }

            std_msgs::msg::Bool ArmCtrlStopMsg;
            ArmCtrlStopMsg.data = true;
            ArmCtrlStopCmd_pub->publish(ArmCtrlStopMsg);

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

            auto MCST_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ModeChangeStartTask>::SharedFuture>();

            auto MCST_res_callback = [this, &flag, MCST_shared_future_ptr, req, &res](rclcpp::Client<robot_msgs::srv::ModeChangeStartTask>::SharedFuture future)
            {
                RCLCPP_INFO(this->get_logger(), "get in MCST_callback");
                *MCST_shared_future_ptr = future;
                auto response = future.get();
                if (response->execute_success)
                {
                    res->execute_success = true;
                    ctrl_mode = req->ctrl_mode;
                    flag = true;
                    RCLCPP_INFO(this->get_logger(), "MCST response is true, res_flag is %d", res->execute_success);
                }
                else
                {
                    RCLCPP_WARN_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ChangerCtrlModeCmdHandle(): ModeChangeStartTask execute failed(res.exe_succ is false).");
                    res->execute_success = false;
                    flag = true;
                }
            };

            auto MCST_future = ModeChangeStartTask_client->async_send_request(MCST_request, MCST_res_callback);
            *MCST_shared_future_ptr = MCST_future.future;

            auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                [this, &flag, MCST_shared_future_ptr, &res_flag, &res]()
                                                {
                                                    if (MCST_shared_future_ptr->valid() &&
                                                        MCST_shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                    {
                                                        RCLCPP_WARN_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ChangerCtrlModeCmdHandle(): ModeChangeStartTask execute failed(srv return is false).");
                                                        res_flag = false;
                                                        flag = true;
                                                    }
                                                });
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
    RCLCPP_INFO(this->get_logger(), "right here");
    sleep(1);
    RCLCPP_INFO(this->get_logger(), "return!");
    return true;
}


bool Ctrl_Manager::CtrlModeQueryHandle(const robot_msgs::srv::CtrlModeQuery::Request::SharedPtr req,
                                                robot_msgs::srv::CtrlModeQuery::Response::SharedPtr res)
{
    (void) req;
    res->runtime_ctrl_mode = ctrl_mode;
    return true;
}

void Ctrl_Manager::MotorCtrlNormalCmdCallback(const robot_msgs::msg::MotorCtrlNormal::SharedPtr msg)
{
    /*四足机器人的手动控制实现已放在b2_manual_ctrl这个包里*/
    if (ctrl_mode == 1)
    {
        if (device_type == "e_robot_quadrupedal")
        {
            if (msg->command == "move_stop")
            {
                std_msgs::msg::Bool QuadrupedalMotorStopCtrlMsg;
                QuadrupedalMotorStopCtrlMsg.data = true;
                QuadrupedalMotorStopCtrl_pub->publish(QuadrupedalMotorStopCtrlMsg);
            }
            else
            {
                robot_msgs::msg::MotorCtrlNormal QuadrupedalMotorCtrlNormalMsg;
                QuadrupedalMotorCtrlNormalMsg.command = msg->command;
                QuadrupedalMotorCtrlNormalMsg.run_speed = msg->run_speed;
                QuadrupedalMotorCtrlNormal_pub->publish(QuadrupedalMotorCtrlNormalMsg);
            }
        }

        operate_manual_last_time = std::time(nullptr);
    }
}

void Ctrl_Manager::MotorCtrltoPosCmdCallback(const robot_msgs::msg::MotorCtrltoPos::SharedPtr msg)
{
    if (ctrl_mode == 1)
    {
        if (device_type == "e_robot_quadrupedal")
        {
            std_msgs::msg::Bool QuadrupedalMotorStopCtrlMsg;
            QuadrupedalMotorStopCtrlMsg.data = true;
            QuadrupedalMotorStopCtrl_pub->publish(QuadrupedalMotorStopCtrlMsg);

            if (msg->command == "runto_position")
            {
                robot_msgs::msg::MotorCtrltoPos QuadrupedalMotorCtrltoPosMsg;
                QuadrupedalMotorCtrltoPosMsg.command = msg->command;
                QuadrupedalMotorCtrltoPosMsg.track_id = msg->track_id;
                QuadrupedalMotorCtrltoPosMsg.track_pos = msg->track_pos;
                QuadrupedalMotorCtrltoPosMsg.run_speed = msg->run_speed;
                QuadrupedalMotorCtrltoPos_pub->publish(QuadrupedalMotorCtrltoPosMsg);
            }
            else if (msg->command == "go_home")
            {
                std::thread RobotGoHomeThread(&Ctrl_Manager::RobotGoHomeBatteryCharge, this);
                RobotGoHomeThread.detach();
            }
        }
    }
}

void Ctrl_Manager::RobotGoHomeBatteryCharge()
{
    if (device_type == "e_robot_quadrupedal")
    {
        robot_msgs::srv::BatteryChargeNav::Request::SharedPtr BCN_request;
        BCN_request->nav_goal.pose.position.x = batteryChargeTargetNavPose.target_pos_x;
        BCN_request->nav_goal.pose.position.y = batteryChargeTargetNavPose.target_pos_y;
        BCN_request->nav_goal.pose.position.z = batteryChargeTargetNavPose.target_pos_z;
        BCN_request->nav_goal.pose.orientation.z = batteryChargeTargetNavPose.target_ori_z;
        BCN_request->nav_goal.pose.orientation.w = batteryChargeTargetNavPose.target_ori_w;
        while (!BatteryChargeNav_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "BatteryChargeNav Service not available, waiting again...");
        }

        auto BCN_future = BatteryChargeNav_client->async_send_request(BCN_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), BCN_future) == rclcpp::FutureReturnCode::SUCCESS)
        {   
            auto BCN_response = BCN_future.get();
            if (!BCN_response->execute_success)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] RobotGoHomeBatteryCharge(): Failed to nav to battery charge target pose.");
                return;
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] RobotGoHomeBatteryCharge(): Nav program is wrong.");
            return;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] RobotGoHomeBatteryCharge(): Successfully nav to battery charge target pose.");
    }

    robot_msgs::srv::BatteryChargeExecuteProgram::Request::SharedPtr BCEP_request;
    auto BCEP_future = BatteryChargeExecuteProgram_client->async_send_request(BCEP_request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), BCEP_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto BCEP_response = BCEP_future.get();
        if (BCEP_response->execute_success)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] RobotGoHomeBatteryCharge(): Successfully execute battery charge program.");
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] RobotGoHomeBatteryCharge(): Failed to execute battery charge program(res.exe_succ is false).");
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] RobotGoHomeBatteryCharge(): Failed to execute battery charge program(srv call return false).");
    }
}

/***********************************************外设相关***********************************************/
void Ctrl_Manager::ArmCtrlCmdCallback(const robot_msgs::msg::ArmCtrl::SharedPtr msg)
{
    if (ctrl_mode == 1)
    {
        if (device_type == "e_robot_quadrupedal")
        {
            if (msg->command == "e_arm_stop")
            {
                std_msgs::msg::Bool ArmCtrlStopMsg;
                ArmCtrlStopMsg.data = true;
                ArmCtrlStopCmd_pub->publish(ArmCtrlStopMsg);
            }
            else
            {
                robot_msgs::msg::ArmCtrl ArmCtrlMoveMsg;
                ArmCtrlMoveMsg.command = msg->command;
                ArmCtrlMoveMsg.mode = msg->mode;
                ArmCtrlMoveMsg.key = msg->key;
                ArmCtrlMoveMsg.run_speed = msg->run_speed;
                ArmCtrlMoveMsg.position = msg->position;
                ArmCtrlMoveCmd_pub->publish(ArmCtrlMoveMsg);
            }
        }
    }
}

void Ctrl_Manager::ExtendDevCtrlCmdCallback(const robot_msgs::msg::ExtendDevCtrl::SharedPtr msg)
{
    if (ctrl_mode == 1)
    {
        robot_msgs::msg::ExtendDevCtrl ExtendDevCtrlInternalMsg;
        ExtendDevCtrlInternalMsg.device_type = msg->device_type;
        ExtendDevCtrlInternalMsg.command = msg->command;
        ExtendDevCtrlInternal_pub->publish(ExtendDevCtrlInternalMsg);
    }
}

/***********************************************机器人状态相关***********************************************/
void Ctrl_Manager::RobotRuntimePositionCallback(const robot_msgs::msg::RobotRuntimePosition::SharedPtr msg)
{
    runtime_pos_id_track = msg->id_track;
    runtime_pos_track_pos = msg->track_pos;
}

void Ctrl_Manager::RobotBatteryStatusCallback(const robot_msgs::msg::RobotBatteryStatus::SharedPtr msg)
{
    battery_percent = msg->battery_percent;
    charge_status = msg->charge_status;
}

void Ctrl_Manager::TaskFinishAndExecuteBatteryChargeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        if (device_type == "e_robot_quadrupedal")
        {
            robot_msgs::srv::BatteryChargeNav::Request::SharedPtr BCN_request;
            BCN_request->nav_goal.pose.position.x = batteryChargeTargetNavPose.target_pos_x;
            BCN_request->nav_goal.pose.position.y = batteryChargeTargetNavPose.target_pos_y;
            BCN_request->nav_goal.pose.position.z = batteryChargeTargetNavPose.target_pos_z;
            BCN_request->nav_goal.pose.orientation.z = batteryChargeTargetNavPose.target_ori_z;
            BCN_request->nav_goal.pose.orientation.w = batteryChargeTargetNavPose.target_ori_w;
            while (!BatteryChargeNav_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "BatteryChargeNav Service not available, waiting again...");
            }

            auto BCN_future = BatteryChargeNav_client->async_send_request(BCN_request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), BCN_future) == rclcpp::FutureReturnCode::SUCCESS)
            {   
                auto BCN_response = BCN_future.get();
                if (!BCN_response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinishAndExecuteBatteryChargeCallback(): Failed to nav to battery charge target pose.");
                    return;
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinishAndExecuteBatteryChargeCallback(): Nav program is wrong.");
                return;
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinishAndExecuteBatteryChargeCallback(): Successfully nav to battery charge target pose.");
        }

        robot_msgs::srv::BatteryChargeExecuteProgram::Request::SharedPtr BCEP_request;
        auto BCEP_future = BatteryChargeExecuteProgram_client->async_send_request(BCEP_request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), BCEP_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto BCEP_response = BCEP_future.get();
            if (BCEP_response->execute_success)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinishAndExecuteBatteryChargeCallback(): Successfully execute battery charge program.");
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinishAndExecuteBatteryChargeCallback(): Failed to execute battery charge program(res.exe_succ is false).");
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskFinishAndExecuteBatteryChargeCallback(): Failed to execute battery charge program(srv call return false).");
        }
    }
}

void Ctrl_Manager::BatteryPercentManage()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage Thread Start!");
    
    {
        /* 睡到下一个整秒，避免累加误差 */
        struct timespec nowts;
        clock_gettime(CLOCK_MONOTONIC, &nowts);
        time_t next_sec = nowts.tv_sec + 1;
        struct timespec ts = {next_sec, 0};
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
    }

    if (battery_percent < battery_percent_low_threshold && !execute_battery_charge)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Stage1 Start");

        ctrl_mode = 2;
        battery_charge_lock = true;
        
        if (device_type == "e_robot_quadrupedal")
        {
            if (AvoidObstaclesPauseTaskOrMove != "none")
                return;
        }

        auto TESQ_request = std::make_shared<robot_msgs::srv::TaskExecuteStatusQuery::Request>();

        while (!TaskExecuteStatusQuery_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "TaskExecuteStatusQuery Service not available, waiting again...");
        }
        auto TESQ_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::TaskExecuteStatusQuery>::SharedFuture>();

        auto TESQ_res_callback = [this, TESQ_shared_future_ptr](rclcpp::Client<robot_msgs::srv::TaskExecuteStatusQuery>::SharedFuture future)
        {
            auto TESQ_response = future.get();
            *TESQ_shared_future_ptr = future;
            if (TESQ_response->task_execute)
            {
                auto BCPT_request = std::make_shared<robot_msgs::srv::BatteryChargePauseTask::Request>();
                while (!BatteryChargePauseTask_client->wait_for_service(std::chrono::seconds(1)))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "BatteryChargePauseTask Service not available, waiting again...");
                }
                auto BCPT_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::BatteryChargePauseTask>::SharedFuture>();
                auto BCPT_res_callback = [this, BCPT_shared_future_ptr](rclcpp::Client<robot_msgs::srv::BatteryChargePauseTask>::SharedFuture future)
                {
                    auto BCPT_response = future.get();
                    *BCPT_shared_future_ptr = future;
                    if (!BCPT_response->execute_success)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): BatteryChargePauseTask execute failed.");
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): BatteryChargePauseTask execute successfully.");
                        finish_battery_charge_and_execute_task = false;
                    }
                };

                auto BCPT_future = BatteryChargePauseTask_client->async_send_request(BCPT_request, BCPT_res_callback);
                *BCPT_shared_future_ptr = BCPT_future.future;

                auto BCPT_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                        [this, TESQ_shared_future_ptr]()
                                                        {
                                                            if (TESQ_shared_future_ptr->valid() &&
                                                                TESQ_shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                            {
                                                                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): BatteryChargePauseTask_client call return false.");
                                                            }
                                                        });
            }
            else
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): No task is executing.");

                if (device_type == "e_robot_quadrupedal")
                {
                    std_msgs::msg::Bool QuadrupedalMotorStopCtrlMsg;
                    QuadrupedalMotorStopCtrlMsg.data = true;
                    QuadrupedalMotorStopCtrl_pub->publish(QuadrupedalMotorStopCtrlMsg);
                }
            }
        };

        auto TESQ_future = TaskExecuteStatusQuery_client->async_send_request(TESQ_request, TESQ_res_callback);
        *TESQ_shared_future_ptr = TESQ_future.future;

        auto TESQ_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                [this, TESQ_shared_future_ptr]()
                                                {
                                                    if (TESQ_shared_future_ptr->valid() &&
                                                        TESQ_shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                    {
                                                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): TaskExecuteStatusQuery_client call return false.");
                                                    }
                                                });

        
        // 定位移动平台到充电点(四足)
        if (device_type == "e_robot_quadrupedal")
        {
            robot_msgs::srv::BatteryChargeNav::Request::SharedPtr BCN_request;
            BCN_request->nav_goal.pose.position.x = batteryChargeTargetNavPose.target_pos_x;
            BCN_request->nav_goal.pose.position.y = batteryChargeTargetNavPose.target_pos_y;
            BCN_request->nav_goal.pose.position.z = batteryChargeTargetNavPose.target_pos_z;
            BCN_request->nav_goal.pose.orientation.z = batteryChargeTargetNavPose.target_ori_z;
            BCN_request->nav_goal.pose.orientation.w = batteryChargeTargetNavPose.target_ori_w;
            while (!BatteryChargeNav_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "BatteryChargeNav Service not available, waiting again...");
            }
            auto BCN_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::BatteryChargeNav>::SharedFuture>();
            auto BCN_res_callback = [this, BCN_shared_future_ptr](rclcpp::Client<robot_msgs::srv::BatteryChargeNav>::SharedFuture future)
            {
                auto BCN_response = future.get();
                *BCN_shared_future_ptr = future;
                if (!BCN_response->execute_success)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Failed to nav to battery charge target pose.");
                }
            };

            auto BCN_future = BatteryChargeNav_client->async_send_request(BCN_request, BCN_res_callback);
            *BCN_shared_future_ptr = BCN_future.future;
            auto BCN_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                    [this, BCN_shared_future_ptr]()
                                                    {
                                                        if (BCN_shared_future_ptr->valid() &&
                                                            BCN_shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                        {
                                                            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Nav program is wrong.");
                                                        }
                                                    });
            
            RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Successfully nav to battery charge target pose.");
        }

        robot_msgs::srv::BatteryChargeExecuteProgram::Request::SharedPtr BCEP_request;
        while (!BatteryChargeExecuteProgram_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "BatteryChargeExecuteProgram Service not available, waiting again...");
        }
        auto BCEP_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::BatteryChargeExecuteProgram>::SharedFuture>();
        auto BCEP_res_callback = [this, BCEP_shared_future_ptr](rclcpp::Client<robot_msgs::srv::BatteryChargeExecuteProgram>::SharedFuture future)
        {
            auto BCEP_response = future.get();
            *BCEP_shared_future_ptr = future;
            if (BCEP_response->execute_success)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Successfully execute battery charge program.");
                execute_battery_charge = true;
                RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Stage1 Finish.");
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Failed to execute battery charge program(res.exe_succ is false).");
                return;
            }
        };

        auto BCEP_future = BatteryChargeExecuteProgram_client->async_send_request(BCEP_request, BCEP_res_callback);
        *BCEP_shared_future_ptr = BCEP_future.future;
        auto BCEP_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                [this, BCEP_shared_future_ptr]()
                                                {
                                                    if (BCEP_shared_future_ptr->valid() &&
                                                        BCEP_shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                    {
                                                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Failed to execute battery charge program(srv call return false).");
                                                        return;
                                                    }
                                                });
    }

    if (battery_percent >= battery_percent_high_threshold && battery_charge_lock)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Stage2 Start.");

        battery_charge_lock = false;
        execute_battery_charge = false;

        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Stage2 Finish.");
    }

    if (device_type == "e_robot_quadrupedal")
    {
        if (battery_percent >= battery_percent_full_threshold && !finish_battery_charge_and_execute_task)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Stage3 Start.");

            ctrl_mode = 0;

            robot_msgs::srv::BatteryChargeStartTask::Request::SharedPtr BCST_request;
            while (!BatteryChargeStartTask_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "BatteryChargeStartTask Service not available, waiting again...");
            }
            auto BCST_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::BatteryChargeStartTask>::SharedFuture>();
            auto BCST_res_callback = [this, BCST_shared_future_ptr](rclcpp::Client<robot_msgs::srv::BatteryChargeStartTask>::SharedFuture future)
            {
                auto BCST_response = future.get();
                *BCST_shared_future_ptr = future;
                if (BCST_response->execute_success)
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): BatteryChargeStartTask execute Successfully.");

                    finish_battery_charge_and_execute_task = true;

                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): Stage3 Finish.");
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): BatteryChargeStartTask failed(res.exe_succ is false).");
                    return;
                }
            };

            auto BCST_future = BatteryChargeStartTask_client->async_send_request(BCST_request, BCST_res_callback);
            *BCST_shared_future_ptr = BCST_future.future;
            auto BCST_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                     [this, BCST_shared_future_ptr]()
                                                     {
                                                         if (BCST_shared_future_ptr->valid() &&
                                                             BCST_shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                         {
                                                             RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] BatteryPercentManage(): BatteryChargeStartTask failed(srv call return false).");
                                                             return;
                                                         }
                                                     });

        }
    }
}

void Ctrl_Manager::ManualMode2TaskModeAutoCheck()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ManualMode2TaskModeAutoCheck Thread Start!");

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

        // 处理分钟进位可能导致的小时/日变化，mktime会自动进位并返回新的 time_t
        time_t next_min = mktime(&localt);

        // 构造要睡到的 timespec (整分钟时刻，纳秒为0)
        timespec ts;
        ts.tv_sec = next_min;
        ts.tv_nsec = 0;

        // 让线程休眠到下一个整分钟
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    }

    if (ctrl_mode == 1)
    {
        std::time_t current_time = std::time(nullptr);
        struct tm localt;
        localtime_r(&current_time, &localt);

        if ((current_time - operate_manual_last_time) >= ManualMode2TaskModeDuration * 60)
        {
            if (device_type == "e_robot_quadrupedal")
            {
                std_msgs::msg::Bool QuadrupedalMotorStopCtrlMsg;
                QuadrupedalMotorStopCtrlMsg.data = true;
                QuadrupedalMotorStopCtrl_pub->publish(QuadrupedalMotorStopCtrlMsg);
            }

            std_msgs::msg::Bool ArmCtrlStopMsg;
            ArmCtrlStopMsg.data = true;
            ArmCtrlStopCmd_pub->publish(ArmCtrlStopMsg);

            auto MCST_request = std::make_shared<robot_msgs::srv::ModeChangeStartTask::Request>();
            auto MCST_shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ModeChangeStartTask>::SharedFuture>();
            auto MCST_res_callback = [this, MCST_shared_future_ptr](rclcpp::Client<robot_msgs::srv::ModeChangeStartTask>::SharedFuture future)
            {
                *MCST_shared_future_ptr = future;
                auto MCST_response = future.get();
                if (MCST_response->execute_success)
                {
                    ctrl_mode = 0;
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ManualMode2TaskModeAutoCheck(): Manual Mode auto transfer to Task Mode.");
                    return;
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ManualMode2TaskModeAutoCheck(): ModeChangeStartTask execute failed(res.exe_succ is false).");
                    return;
                }
            };
            auto MCST_future = ModeChangeStartTask_client->async_send_request(MCST_request, MCST_res_callback);
            *MCST_shared_future_ptr = MCST_future.future;
            auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                [this, MCST_shared_future_ptr]()
                                                {
                                                    if (MCST_shared_future_ptr->valid() &&
                                                        MCST_shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                                                    {
                                                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] ManualMode2TaskModeAutoCheck(): ModeChangeStartTask execute failed(srv call return false).");  
                                                        return;
                                                    }
                                                });
        }
    }
}

/***********************************************心跳包相关***********************************************/
void Ctrl_Manager::RobotHeartbeatBagPub()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] RobotHeartbeatBagPub Thread Start!");

    {
        /* 睡到下一秒的500毫秒，避免累加误差 */
        struct timespec nowts;
        clock_gettime(CLOCK_MONOTONIC, &nowts);
        time_t next_sec = nowts.tv_sec + 1;
        struct timespec ts = {next_sec, 500UL * 1000UL * 1000UL};
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
    }

    // 仅供调试用

    battery_percent = 80;
    runtime_pos_id_track = "模拟轨道";
    runtime_pos_track_pos = 1234;
    ctrl_mode = 1;
    runtime_status = 1;
    charge_status = 0;
    error_code = 0000;


    auto heartbeat_msg = std::make_shared<robot_msgs::msg::HeartbeatBag>();
    
    heartbeat_msg->runtime_battery_percent = battery_percent;
    
    heartbeat_msg->runtime_track_id = runtime_pos_id_track;
    heartbeat_msg->runtime_track_pos = runtime_pos_track_pos;
    
    heartbeat_msg->runtime_ctrl_mode = ctrl_mode;
    
    // if (charge_status)
    //     runtime_status = 3;
    // else
    // {
    //     if (ctrl_mode == 1)
    //         runtime_status = 1;
    //     else if (ctrl_mode == 0)
    //     {
    //         robot_msgs::srv::TaskExecuteStatusQuery::Request::SharedPtr request;
    //         while (!TaskExecuteStatusQuery_client->wait_for_service(std::chrono::seconds(1)))
    //         {
    //             if (!rclcpp::ok())
    //             {
    //                 RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
    //                 return;
    //             }
    //             RCLCPP_INFO(this->get_logger(), "TaskExecuteStatusQuery Service not available, waiting again...");
    //         }

    //         auto res_back = [this, &heartbeat_msg](rclcpp::Client<robot_msgs::srv::TaskExecuteStatusQuery>::SharedFuture future)
    //         {
    //             auto response = future.get();
    //             if (response->task_execute)
    //             {
    //                 runtime_status = 2;
    //                 heartbeat_msg->runtime_task_ticket = response->task_ticket;
    //             }
    //             else
    //                 runtime_status = 0;
    //         };
    //         auto future = TaskExecuteStatusQuery_client->async_send_request(request, res_back);             
    //     }
    // }

    heartbeat_msg->runtime_status = runtime_status;
    heartbeat_msg->runtime_charge_status = charge_status;
    heartbeat_msg->runtime_timestamp = std::time(NULL);
    heartbeat_msg->runtime_error_code = error_code;

    HeartbeatBag_pub->publish(*heartbeat_msg);
}   

