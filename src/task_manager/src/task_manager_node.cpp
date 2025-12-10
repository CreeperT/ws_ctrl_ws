#include "task_manager.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    setlocale(LC_CTYPE, "zh_CN.utf8");

    // 使用 shared_ptr 创建节点实例，以符合 ROS2 规范
    auto task_manager_client = std::make_shared<Task_Manager>("task_manager_node");
    task_manager_client->DeviceParamInit();
    auto task_file_manager = std::make_shared<Task_File_Manager>(task_manager_client->id_device, task_manager_client->devel_mode);
    auto task_execute = std::make_shared<Task_Execute>(task_manager_client->id_device, task_manager_client->device_type, 
                                                       task_manager_client->devel_mode, task_manager_client->TrackArmMode, 
                                                       task_manager_client->ImgDataCollectWaitTime, 
                                                       task_manager_client->TaskStartOrFinishResponseDuration, 
                                                       task_manager_client->WiperWorkDuration);
    

    // 调用 NodeSpinnerStartup()，保持原逻辑
    task_manager_client->NodeSpinnerStartup();
    task_file_manager->NodeSpinnerStartup();
    task_execute->NodeSpinnerStartup();

    // 等待 spinning 线程完成（相当于 ROS1 的 ros::waitForShutdown()）
    task_manager_client->joinSpinnerThread();
    task_file_manager->joinSpinnerThread();
    task_execute->joinSpinnerThread();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}