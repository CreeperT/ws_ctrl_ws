#include "websocket.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include <memory>
#include <cjson/cJSON.h>

WebSocket_Client::WebSocket_Client() 
    : Node("web_socket_client"), resolver_(ioc_), ws_(ioc_), reconnect_flag(true)
{
    if (DeviceParamInit())
    {
        NodePublisherInit();
        NodeSubscriberInit();
    }
}

bool WebSocket_Client::DeviceParamInit()
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
        cJSON* value_server_host = cJSON_GetObjectItem(value_device_param, "WebSocket_Server_host");
        cJSON* value_server_port = cJSON_GetObjectItem(value_device_param, "WebSocket_Server_port");
        cJSON* value_server_param = cJSON_GetObjectItem(value_device_param, "WebSocket_Server_param");
        cJSON* value_devel_mode = cJSON_GetObjectItem(value_device_param, "Devel_Mode");
        cJSON* value_id_device = cJSON_GetObjectItem(value_device_param, "id_device");
        if (value_server_host != NULL && value_server_port != NULL && value_server_param != NULL && value_devel_mode != NULL && value_id_device != NULL)
        {
            host_ = value_server_host->valuestring;
            port_ = value_server_port->valuestring;
            param_ = value_server_param->valuestring;
            devel_mode = value_devel_mode->valuestring;
            id_device = value_id_device->valuestring;
            cJSON_Delete(value_device_param);
            return true;
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Device param file error!");
            cJSON_Delete(value_device_param);
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Device param file error!");
        return false;
    }
}

void WebSocket_Client::NodePublisherInit()
{
    WSmsgs_receive_pub = this->create_publisher<std_msgs::msg::String>("/WSmsgs_receive_topic", 10);
}

void WebSocket_Client::NodeSubscriberInit()
{
    WSmsgs_send_sub= this->create_subscription<std_msgs::msg::String>(
        "WSmsgs_send_topic", 10, std::bind(&WebSocket_Client::WSsendCallback, this, std::placeholders::_1));
}

void WebSocket_Client::NodeSpinnerStartup()
{
    executor_.add_node(shared_from_this());
    spinner_thread_ = std::thread([this]() { executor_.spin(); });
}

void WebSocket_Client::WSsendCallback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
    if (ws_.is_open())
    {
        WS_send(msg->data);
    }
    else
    {
        WS_retry_connect();
        WS_send(msg->data);
    }
}

void WebSocket_Client::WS_connect()
{
    try
    {
        // 解析域名
        auto const results = resolver_.resolve(host_, port_);

        // 连接到服务器
        net::connect(ws_.next_layer(), results.begin(), results.end());

        // 握手，连接到 WebSocket 服务器
        ws_.handshake(host_, param_);
        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Connected to WebSocket server: " << host_ << ":" << port_);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Connection failed: " << e.what());
        WS_retry_connect();
    }
}

void WebSocket_Client::WS_retry_connect()
{
    const int max_retries = 10; // 最大重连次数
    int retry_count = 0;

    if (reconnect_flag)
    {
        while (retry_count < max_retries)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Attempting to reconnect... (" << retry_count + 1 << "/" << max_retries << ")");

            std::this_thread::sleep_for(std::chrono::seconds(5));  // 等待 5 秒再重连

            try
            {
                WS_connect();
                if (ws_.is_open())
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Reconnected to WebSocket server.");
                    return;
                }
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Reconnection failed: " << e.what());
            }

            retry_count++;
        }

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Failed to reconnect after " << max_retries << " attempts.");
    }
}

void WebSocket_Client::WS_send(const std::string& msg)
{
    try
    {
        ws_.write(net::buffer(msg));
        if (devel_mode == "debug")
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Sended message: " << msg);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Failed to send message: " << e.what());
        WS_retry_connect();
    }
}

void WebSocket_Client::WS_receive()
{
    try
    {
        beast::flat_buffer buffer;
        ws_.read(buffer);
        if (devel_mode == "debug")
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Received message: " << beast::buffers_to_string(buffer.data()));
        }
        auto WSmsgs_receive = std::make_shared<std_msgs::msg::String>();
        WSmsgs_receive->data = beast::buffers_to_string(buffer.data());
        WSmsgs_receive_pub->publish(*WSmsgs_receive);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Failed to receive message: " << e.what());
        WS_retry_connect();
    }
}
