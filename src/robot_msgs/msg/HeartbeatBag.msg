# 心跳包消息
float32 runtime_battery_percent  # 运行时电池百分比
string runtime_track_id          # 运行时轨道ID
uint64 runtime_track_pos         # 运行时轨道位置
uint16 runtime_ctrl_mode         # 运行时控制模式
uint16 runtime_status            # 运行时状态
string runtime_task_ticket       # 运行时任务票据
uint16 runtime_charge_status     # 运行时充电状态
uint64 runtime_timestamp         # 运行时时间戳(考虑使用builtin_interfaces/Time如果需要标准时间)
uint16 runtime_error_code        # 运行时错误代码