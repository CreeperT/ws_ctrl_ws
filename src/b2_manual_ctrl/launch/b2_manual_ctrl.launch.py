from launch import LaunchDescription
from launch_ros.actions import Node
#封装终端指令相关类--------------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
#参数声明与获取-----------------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
#文件包含相关-------------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
#分组相关----------------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
#事件相关----------------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
#获取功能包下share目录路径-------------
from ament_index_python.packages import get_package_share_directory
import os

"""
launch文件实现功能
    1. 运行websocket的launch文件
    2. 运行ctrl_manager的launch文件
    3. 运行motor_ctrl_2_twist节点，完成手动控制中MotorCtrlNormal.msg转化Twist.msg
    4. 运行sport_ctrl_mode节点，完成对B2的控制
"""


def generate_launch_description():
    # cm_pkg = get_package_share_directory("ctrl_manager")
    # cm_launch_path = os.path.join(cm_pkg, "launch", "ctrl_manager.launch.py")
    # # 1.运行websocket
    # action_ws_node = Node(
    #     package="web_socket",
    #     executable="web_socket_node"
    # )
# 
    # action_ws_msg_manage_node = Node(
    #     package="web_socket",
    #     executable="ws_msgs_manage_node"
    # )
# 
    # # 2.运行ctrl_manager
    # action_cm_launch = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         launch_file_path=cm_launch_path
    #     )
    # )

    # 3.运行消息转化节点
    action_msg_tf_node = Node(
        package="b2_manual_ctrl",
        executable="motor_ctrl_2_twist"
    )

    # 4.运行手动控制节点
    action_sport_ctrl_node = Node(
        package="b2_manual_ctrl",
        executable="b2_sport_ctrl_node"
    )

    return LaunchDescription([
        # action_ws_node,
        # action_ws_msg_manage_node,
        # action_cm_launch,
        action_msg_tf_node,
        action_sport_ctrl_node
    ])