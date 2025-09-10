from launch import LaunchDescription
from launch_ros.actions import Node
#封装终端指令相关类--------------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
#参数声明与获取-----------------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
#文件包含相关-------------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
#分组相关----------------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
#事件相关----------------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
#获取功能包下share目录路径-------------
# from ament_index_python.packages import get_package_share_directory
# import os

def generate_launch_description():

    # 1.运行websocket
    action_ws_node = Node(
        package="websocket",
        executable="websocket_node"
    )

    # 2.运行ws_msgs_manager
    action_ws_msgs_manager_node = Node(
        package="websocket",
        executable="ws_msgs_manager_node"
    )

    # 3.运行ctrl_manager
    action_ctrl_manager_node = Node(
        package="websocket",
        executable="ctrl_manager_node"
    )

    # 4.运行消息转化节点
    action_msg_tf_node = Node(
        package="b2_manual_ctrl",
        executable="motor_ctrl_2_twist"
    )

    # 5.运行手动控制节点
    action_sport_ctrl_node = Node(
        package="b2_manual_ctrl",
        executable="b2_sport_ctrl_node"
    )

    return LaunchDescription([
        action_ws_node,
        action_ws_msgs_manager_node,
        action_ctrl_manager_node,
        action_msg_tf_node,
        action_sport_ctrl_node
    ])