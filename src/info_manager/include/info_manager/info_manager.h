#ifndef _INFO_MANAGER_H_
#define _INFO_MANAGER_H_

#include "rclcpp/rclcpp.hpp"
#include <cjson/cJSON.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <string>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include "robot_msgs/srv/robot_info_query.hpp" // 机器人信息srv

class Info_Manager : public rclcpp::Node
{
public:
    Info_Manager(const char *node_name);
    ~Info_Manager();

    // 初始化函数
    bool DeviceParamInit();       // 设备参数初始化
    void NodePublisherInit();     // 节点话题发布初始化
    void NodeSubscriberInit();    // 节点话题订阅初始化
    void NodeServiceServerInit(); // 节点服务服务端初始化
    void NodeServiceClientInit(); // 节点服务客户端初始化

    // 机器人信息查询
    bool RobotInfoQueryHandle(robot_msgs::srv::RobotInfoQuery::Request::SharedPtr req,
                              robot_msgs::srv::RobotInfoQuery::Response::SharedPtr res);

    inline std::string getCurrentTimeStr()
    {
        time_t now = time(NULL);
        struct tm localt;
        localtime_r(&now, &localt);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);
        return time_buf;
    }

public:
    rclcpp::Service<robot_msgs::srv::RobotInfoQuery>::SharedPtr RobotInfoQuery_server;

    // 机器人设备相关信息
    std::string id_device;
    std::string device_type;
    std::string devel_mode;
    std::string TrackArmMode;
};

#endif