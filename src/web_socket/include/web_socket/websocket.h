#ifndef _WEBSOCKET_H_
#define _WEBSOCKET_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio.hpp>

#include <fstream>
#include <cstdlib>
#include <cjson/cJSON.h>
#include <thread>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

class WebSocket_Client : public rclcpp::Node
{
public:
    WebSocket_Client();

    bool DeviceParamInit(); // 参数初始化

    void NodePublisherInit(); // 发布者初始化
    void NodeSubscriberInit(); // 订阅者初始化

    void WSsendCallback(const std_msgs::msg::String::ConstSharedPtr& msg); // 向ws服务端发送消息的回调函数

    void WS_connect(); // 连接ws服务端
    void WS_retry_connect(); // 重连
    void WS_send(const std::string& msg); // 向ws服务端发送消息
    void WS_receive(); // 接收ws服务端发送的消息  
    
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
    rclcpp::executors::MultiThreadedExecutor executor_; 
    std::thread spinner_thread_; 

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr WSmsgs_receive_pub; 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr WSmsgs_send_sub; 

    net::io_context ioc_;
    tcp::resolver resolver_;
    websocket::stream<tcp::socket> ws_;
    std::string host_;
    std::string port_;
    std::string param_;
    bool reconnect_flag;

    std::string devel_mode;
    std::string id_device;

};

#endif