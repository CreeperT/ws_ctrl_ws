#include "websocket.h"
#include "rclcpp/rclcpp.hpp"
#include <signal.h>
#include <ctime>
#include <iomanip>

std::shared_ptr<WebSocket_Client> pWSClient;

void signalHandler(int signum)
{
    {
        time_t now = time(NULL);
        struct tm localt;
        localtime_r(&now, &localt);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("web_socket_node"), "[" << time_buf << "] Interrupt signal " << signum << " received. Closing WebSocket connection.");
    }

    if (pWSClient)
    {
        try
        {
            pWSClient->reconnect_flag = false;
            pWSClient->ws_.close(websocket::close_code::normal);
        }
        catch (const std::exception& e)
        {
            time_t now = time(NULL);
            struct tm localt;
            localtime_r(&now, &localt);
            char time_buf[64];
            strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &localt);

            RCLCPP_ERROR_STREAM(rclcpp::get_logger("web_socket_node"), "[" << time_buf << "] Exception when closing WebSocket: " << e.what());
        }
        pWSClient.reset();  // 释放指针
    }

    rclcpp::shutdown();  // 优雅地关闭 ROS2
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    setlocale(LC_CTYPE, "zh_CN.utf8");

    pWSClient = std::make_shared<WebSocket_Client>();

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    pWSClient->WS_connect();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pWSClient);

    std::thread spinner_thread([&executor]() {
        executor.spin();
    });

    while (rclcpp::ok())
    {
        pWSClient->WS_receive();
    }

    spinner_thread.join();

    return EXIT_SUCCESS;
}
