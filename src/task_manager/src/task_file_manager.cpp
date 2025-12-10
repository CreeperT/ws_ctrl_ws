#include "task_file_manager.h"
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

Task_File_Manager::Task_File_Manager(const std::string id_device_, const std::string devel_mode_) : Node("task_file_manager")
{
    id_device = id_device_;
    devel_mode = devel_mode_;

    getTaskFilesDir();
}

/********************************************************初始化*************************************************************/
/*自然序比较器*/
bool naturalLess(const std::string& a, const std::string& b)
{
    // 把字符串拆成「连续数字」或「连续非数字」两类 token
    static const std::regex re(R"((\d+)|(\D+))");
    std::sregex_token_iterator ia(a.begin(), a.end(), re);
    std::sregex_token_iterator ib(b.begin(), b.end(), re);
    std::sregex_token_iterator end;

    for ( ; ia != end && ib != end; ++ia, ++ib)
    {
        const std::string sa = ia->str();
        const std::string sb = ib->str();

        const bool na = std::isdigit(sa[0]);
        const bool nb = std::isdigit(sb[0]);

        if (na && nb)                  // 两段都是数字 → 按整数大小比
        {
            long long va = std::stoll(sa);
            long long vb = std::stoll(sb);
            if (va != vb) return va < vb;
        }
        else                           // 至少有一段不是全数字 → 按字符串比
        {
            // 若想忽略大小写，可改成 std::lexicographical_compare
            if (sa != sb) return sa < sb;
        }
    }
    // 走到这里说明较短的那串已比完：前缀短的排前面
    return a.size() < b.size();
}

void Task_File_Manager::getTaskFilesDir()
{
    const char* homeDir = getenv("HOME");
    TaskFilesDir = homeDir;
    TaskFilesDir = TaskFilesDir  + "/ws_ctrl_ws/src/task_manager/task_files/";
}

std::string Task_File_Manager::generateRandomUUID()
{
    std::random_device rd;
    std::mt19937 gen(rd());  // 使用Mersenne Twister随机数引擎
    std::uniform_int_distribution<int> dis(0, 15);  // 生成0到15的随机数 (十六进制范围)

    std::stringstream uuid;
    uuid << std::uppercase;  // 使用大写字母

    // 生成8-4-4-4-12格式的UUID
    int lengths[] = {8, 4, 4, 4, 12};  // 每段的长度
    for (int i = 0; i < 5; ++i)
    {
        if (i > 0)
        {
            uuid << "-";  // 插入分隔符
        }
        for (int j = 0; j < lengths[i]; ++j)
        {
            uuid << std::hex << dis(gen);  // 生成十六进制字符
        }
    }

    return uuid.str();
}

void Task_File_Manager::NodeSpinnerStartup()
{
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());
    executor.spin();
}

/********************************************************任务信息*************************************************************/
bool Task_File_Manager::TaskInfoListQuery(const robot_msgs::srv::InspectTaskInfoListQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoListQuery::Response::SharedPtr res)
{
    size_t TaskTypeNum = req->task_type.size();
    size_t TaskInfoNum = 0;
    std::string TaskInfoFileDirectoryDir;
    for (size_t i = 0; i < TaskTypeNum; i++)
    {
        TaskInfoFileDirectoryDir = TaskFilesDir + "task_info_files/" + req->task_type[i];

        DIR* dir = opendir(TaskInfoFileDirectoryDir.c_str()); // 打开目录
        if (dir == nullptr)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoListQuery(): Error: Could not open task info json file directory!");
            return false;
        }

        struct dirent* entry;
        size_t cnt = 0;
        while ((entry = readdir(dir)) != nullptr)
        {
            // 跳过"."和".."目录项
            if (std::string(entry->d_name) == "." || std::string(entry->d_name) == "..")
            {
                continue;
            }

            // 判断是否为普通文件
            if (entry->d_type == DT_REG)
            {
                std::string TaskInfoFileDir = TaskInfoFileDirectoryDir + "/" + entry->d_name;
                std::ifstream TaskInfoFileRead(TaskInfoFileDir.c_str());
                if (!TaskInfoFileRead)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoListQuery(): Can't open task info json file(" << entry->d_name << ")!");
                    closedir(dir);
                    return false;
                }

                std::string TaskInfoContent((std::istreambuf_iterator<char>(TaskInfoFileRead)), std::istreambuf_iterator<char>());
                TaskInfoFileRead.close();

                cJSON* TaskInfoJson = cJSON_Parse(TaskInfoContent.c_str());
                if (TaskInfoJson != NULL)
                {
                    cJSON* value_id_task_info = cJSON_GetObjectItem(TaskInfoJson, "id_task_info");
                    cJSON* value_task_name = cJSON_GetObjectItem(TaskInfoJson, "task_name");
                    cJSON* value_task_type = cJSON_GetObjectItem(TaskInfoJson, "task_type");
                    cJSON* value_create_time = cJSON_GetObjectItem(TaskInfoJson, "create_time");
                    if (value_id_task_info != NULL && value_task_name != NULL && value_task_type != NULL && value_create_time != NULL)
                    {
                        robot_msgs::msg::InspectTaskInfoSimple InspectTaskInfoSimple;
                        InspectTaskInfoSimple.id_task_info = id_device + "-task-info-" + value_id_task_info->valuestring;
                        InspectTaskInfoSimple.task_name = value_task_name->valuestring;
                        InspectTaskInfoSimple.task_type = value_task_type->valuestring;
                        InspectTaskInfoSimple.create_time = value_create_time->valuestring;
                        cJSON_Delete(TaskInfoJson);
                        res->inspect_task_info_simple_list.push_back(InspectTaskInfoSimple);
                        cnt++;
                    }
                    else
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoListQuery(): Failed to get value_id_task_info or value_task_name or value_task_type or value_create_time (" << entry->d_name << ")!");
                        cJSON_Delete(TaskInfoJson);
                        closedir(dir);
                        return false;
                    }
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoListQuery(): Failed to parse TaskInfoJson (" << entry->d_name << ")!");
                    closedir(dir);
                    return false;
                }
            }
        }
        closedir(dir);
        TaskInfoNum += cnt;
    }

    res->task_info_num = TaskInfoNum;

    RCLCPP_INFO_STREAM(this->get_logger(),"[" << getCurrentTimeStr() << "] Successfully query inspect task info list.");

    return true;
}

bool Task_File_Manager::TaskInfoDetailQuery(const robot_msgs::srv::InspectTaskInfoDetailQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoDetailQuery::Response::SharedPtr res)
{
    std::string TaskInfoIndexFileDir = TaskFilesDir + "task_info_files/" + "TaskInfoIndex.json";
    std::ifstream TaskInfoIndexFileRead(TaskInfoIndexFileDir.c_str());
    if (!TaskInfoIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDetailQuery(): Can't open TaskInfoIndex.json!");
        return false;
    }

    std::string TaskInfoIndexContent((std::istreambuf_iterator<char>(TaskInfoIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInfoIndexFileRead.close();

    cJSON* TaskInfoIndexJson = cJSON_Parse(TaskInfoIndexContent.c_str());
    if (TaskInfoIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDetailQuery(): Failed to parse TaskInfoIndexJson!");
        return false;
    }

    cJSON* task_info_index_array = cJSON_GetObjectItem(TaskInfoIndexJson, "TaskInfoIndex");
    if (task_info_index_array == NULL || !cJSON_IsArray(task_info_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDetailQuery(): Failed to get task_info_index_array!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    if (cJSON_GetArraySize(task_info_index_array) == 0)
    {
        res->task_not_exist = true;
        cJSON_Delete(TaskInfoIndexJson);
        return true;
    }

    std::string id_task_info_query = req->id_task_info;
    std::string id_task_info_erase = id_device + "-task-info-";
    std::regex pattern(id_task_info_erase);
    id_task_info_query = std::regex_replace(id_task_info_query, pattern, "");

    size_t TaskInfoIndexArraySize = cJSON_GetArraySize(task_info_index_array);
    bool task_exist = false;
    bool loop_flag = true;
    std::string task_type_query;
    for (size_t i = 0; i < TaskInfoIndexArraySize && loop_flag; i++)
    {
        cJSON* task_info_index_temp = cJSON_GetArrayItem(task_info_index_array, i);
        if (task_info_index_temp != NULL)
        {
            cJSON* id_task_info_temp = cJSON_GetObjectItem(task_info_index_temp, "id_task_info");
            cJSON* task_type_temp = cJSON_GetObjectItem(task_info_index_temp, "task_type");
            if (id_task_info_temp != NULL && task_type_temp != NULL)
            {
                if (id_task_info_query == id_task_info_temp->valuestring)
                {
                    task_exist = true;
                    task_type_query = task_type_temp->valuestring;
                    loop_flag = false;
                    cJSON_Delete(TaskInfoIndexJson);
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDetailQuery(): Failed to get id_task_info_temp or task_type_temp (" << id_task_info_query << ")!");
                cJSON_Delete(TaskInfoIndexJson);
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDetailQuery(): Failed to get task_info_index_temp (" << id_task_info_query << ")!");
            cJSON_Delete(TaskInfoIndexJson);
            return false;
        }
    }

    if (!task_exist)
    {
        res->task_not_exist = true;
        cJSON_Delete(TaskInfoIndexJson);
        return true;
    }

    std::string TaskInfoFileDir = TaskFilesDir + "task_info_files/" + task_type_query + "/" + id_task_info_query + ".json";
    std::ifstream TaskInfoFileRead(TaskInfoFileDir.c_str());
    if (!TaskInfoFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDetailQuery(): Can't open task info json file (" << id_task_info_query << ")!");
        return false;
    }

    std::string TaskInfoContent((std::istreambuf_iterator<char>(TaskInfoFileRead)), std::istreambuf_iterator<char>());
    TaskInfoFileRead.close();

    cJSON* TaskInfoJson = cJSON_Parse(TaskInfoContent.c_str());
    if (TaskInfoJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDetailQuery(): Failed to parse TaskInfoJson (" << id_task_info_query << ")!");
        return false;
    }

    cJSON* value_task_name = cJSON_GetObjectItem(TaskInfoJson, "task_name");
    cJSON* value_task_type = cJSON_GetObjectItem(TaskInfoJson, "task_type");
    cJSON* value_monitor_points = cJSON_GetObjectItem(TaskInfoJson, "monitor_points");
    cJSON* value_create_time = cJSON_GetObjectItem(TaskInfoJson, "create_time");
    if (value_task_name != NULL && value_task_type != NULL && value_monitor_points != NULL && cJSON_IsArray(value_monitor_points) && value_create_time != NULL)
    {
        res->task_not_exist = false;
        res->task_name = value_task_name->valuestring;
        res->task_type = value_task_type->valuestring;
        res->create_time = value_create_time->valuestring;
        for (size_t i = 0; i < (size_t)cJSON_GetArraySize(value_monitor_points); i++)
        {
            cJSON* monitor_points_temp = cJSON_GetArrayItem(value_monitor_points, i);
            if (monitor_points_temp != NULL)
            {
                std_msgs::msg::String data_temp;
                data_temp.data = monitor_points_temp->valuestring;
                res->monitor_points.push_back(data_temp.data);
            }
            else
            {
                cJSON_Delete(TaskInfoJson);
                return false;
            }
        }

        cJSON_Delete(TaskInfoJson);

        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Successfully query inspect task info detail.");

        return true;
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDetailQuery(): Failed to get value_task_name or value_task_type or value_monitor_points or value_create_time (" << id_task_info_query << ")!");
        cJSON_Delete(TaskInfoJson);
        return false;
    }
}

bool Task_File_Manager::TaskInfoConfigure(const robot_msgs::srv::InspectTaskInfoConfigureCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoConfigureCmd::Response::SharedPtr res)
{
    std::string TaskInfoIndexFileDir = TaskFilesDir + "task_info_files/" + "TaskInfoIndex.json";
    std::ifstream TaskInfoIndexFileRead(TaskInfoIndexFileDir.c_str());
    if (!TaskInfoIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Can't open TaskInfoIndex.json!");
        return false;
    }

    std::string TaskInfoIndexContent((std::istreambuf_iterator<char>(TaskInfoIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInfoIndexFileRead.close();

    cJSON* TaskInfoIndexJson = cJSON_Parse(TaskInfoIndexContent.c_str());
    if (TaskInfoIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Failed to parse TaskInfoIndexJson!");
        return false;
    }

    cJSON* task_info_index_array = cJSON_GetObjectItem(TaskInfoIndexJson, "TaskInfoIndex");
    if (task_info_index_array == NULL || !cJSON_IsArray(task_info_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Failed to get task_info_index_array!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    if (cJSON_GetArraySize(task_info_index_array) == 0)
    {
        res->task_not_exist = true;
        cJSON_Delete(TaskInfoIndexJson);
        return true;
    }

    std::string id_task_info_config = req->id_task_info;
    std::string id_task_info_erase = id_device + "-task-info-";
    std::regex pattern(id_task_info_erase);
    id_task_info_config = std::regex_replace(id_task_info_config, pattern, "");

    size_t TaskInfoIndexArraySize = cJSON_GetArraySize(task_info_index_array);
    bool task_exist = false;
    size_t id_task_info_config_idx;
    size_t task_name_config_idx = TaskInfoIndexArraySize;
    for (size_t i = 0; i < TaskInfoIndexArraySize; i++)
    {
        cJSON* task_info_index_temp = cJSON_GetArrayItem(task_info_index_array, i);
        if (task_info_index_temp != NULL)
        {
            cJSON* id_task_info_temp = cJSON_GetObjectItem(task_info_index_temp, "id_task_info");
            cJSON* task_name_temp = cJSON_GetObjectItem(task_info_index_temp, "task_name");
            if (id_task_info_temp != NULL && task_name_temp != NULL)
            {
                if (id_task_info_config == id_task_info_temp->valuestring)
                {
                    task_exist = true;
                    id_task_info_config_idx = i;
                }

                if (req->task_name == task_name_temp->valuestring)
                {
                    task_name_config_idx = i;
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Failed to get id_task_info_temp or task_name_temp!");
                cJSON_Delete(TaskInfoIndexJson);
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Failed to get task_info_index_temp!");
            cJSON_Delete(TaskInfoIndexJson);
            return false;
        }
    }

    if (!task_exist)
    {
        res->task_not_exist = true;
        cJSON_Delete(TaskInfoIndexJson);
        return true;
    }

    if (task_name_config_idx != TaskInfoIndexArraySize && task_name_config_idx != id_task_info_config_idx)
    {
        res->task_not_exist = false;
        res->name_used = true;
        cJSON_Delete(TaskInfoIndexJson);
        return true;
    }

    cJSON* task_info_index_config = cJSON_GetArrayItem(task_info_index_array, id_task_info_config_idx);
    if (task_info_index_config == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Failed to get task_info_index_config (" << id_task_info_config << ")!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    cJSON* task_type_old = cJSON_GetObjectItem(task_info_index_config, "task_type");
    if (task_type_old == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Failed to get task_type_old (" << id_task_info_config << ")!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    std::string TaskInfoOldFileDir = TaskFilesDir + "task_info_files/" + task_type_old->valuestring + "/" + id_task_info_config + ".json";
    if (std::remove(TaskInfoOldFileDir.c_str()) != 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Error deleting old task info json file (" << id_task_info_config << ")!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    cJSON_ReplaceItemInObject(task_info_index_config, "task_name", cJSON_CreateString(req->task_name.c_str()));
    cJSON_ReplaceItemInObject(task_info_index_config, "task_type", cJSON_CreateString(req->task_type.c_str()));

    char* temp_str = cJSON_Print(TaskInfoIndexJson);
    cJSON_Delete(TaskInfoIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): cJSON_Print(TaskInfoIndexJson) error (" << id_task_info_config << ")!");
        return false;
    }

    std::string TaskInfoIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskInfoIndexFileWrite(TaskInfoIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInfoIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Can't open TaskInfoIndex.json!");
        return false;
    }

    TaskInfoIndexFileWrite << TaskInfoIndexContentNew;

    if (!TaskInfoIndexFileWrite.good())
    {
        TaskInfoIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Error: Failed to write to TaskInfoIndex.json!");
        return false;
    }

    TaskInfoIndexFileWrite.close();

    cJSON* task_info_new = cJSON_CreateObject();
    cJSON_AddStringToObject(task_info_new, "id_task_info", id_task_info_config.c_str());
    cJSON_AddStringToObject(task_info_new, "task_name", req->task_name.c_str());
    cJSON_AddStringToObject(task_info_new, "task_type", req->task_type.c_str());
    cJSON* monitor_points_new =  cJSON_CreateArray();

    std::vector<std::string> monitor_points_new_vec;
    for (size_t i = 0; i < req->monitor_points.size(); i++)
    {
        monitor_points_new_vec.push_back(req->monitor_points[i]);
    }

    std::sort(monitor_points_new_vec.begin(), monitor_points_new_vec.end(), naturalLess);

    for (size_t i = 0; i < req->monitor_points.size(); i++)
    {
        cJSON_AddItemToArray(monitor_points_new, cJSON_CreateString(monitor_points_new_vec[i].c_str()));
    }

    cJSON_AddItemToObject(task_info_new, "monitor_points", monitor_points_new);
    cJSON_AddStringToObject(task_info_new, "create_time", req->create_time.c_str());

    temp_str = cJSON_Print(task_info_new);
    cJSON_Delete(task_info_new);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): cJSON_Print(task_info_new) error (" << id_task_info_config << ")!");
        return false;
    }

    std::string TaskInfoContentNew = temp_str;
    free(temp_str);

    std::string TaskInfoNewFileDir = TaskFilesDir + "task_info_files/" + req->task_type + "/" + id_task_info_config + ".json";
    std::ofstream TaskInfoNewFileWrite(TaskInfoNewFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInfoNewFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Cant't open new task info json file (" << id_task_info_config << ")!");
        return false;
    }

    TaskInfoNewFileWrite << TaskInfoContentNew;

    if (!TaskInfoNewFileWrite.good())
    {
        TaskInfoNewFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoConfigure(): Error: Failed to write to new task info json file (" << id_task_info_config << ")!");
        return false;
    }

    TaskInfoNewFileWrite.close();

    res->task_not_exist = false;
    res->name_used = false;
    res->id_task_info = req->id_task_info;

    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Successfully configure task info.");

    return true;
}

bool Task_File_Manager::TaskInfoCreate(const robot_msgs::srv::InspectTaskInfoConfigureCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoConfigureCmd::Response::SharedPtr res)
{
    std::string TaskInfoIndexFileDir = TaskFilesDir + "task_info_files/" + "TaskInfoIndex.json";
    std::ifstream TaskInfoIndexFileRead(TaskInfoIndexFileDir.c_str());
    if (!TaskInfoIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Can't open TaskInfoIndex.json!");
        return false;
    }

    std::string TaskInfoIndexContent((std::istreambuf_iterator<char>(TaskInfoIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInfoIndexFileRead.close();

    cJSON* TaskInfoIndexJson = cJSON_Parse(TaskInfoIndexContent.c_str());
    if (TaskInfoIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Failed to parse TaskInfoIndexJson!");
        return false;
    }

    cJSON* task_info_index_array = cJSON_GetObjectItem(TaskInfoIndexJson, "TaskInfoIndex");
    if (task_info_index_array == NULL || !cJSON_IsArray(task_info_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Failed to get task_info_index_array!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    if (cJSON_GetArraySize(task_info_index_array) == 0)
    {
        std::string id_task_info_new = generateRandomUUID();
        cJSON* task_info_index_new = cJSON_CreateObject();
        cJSON_AddStringToObject(task_info_index_new, "id_task_info", id_task_info_new.c_str());
        cJSON_AddStringToObject(task_info_index_new, "task_name", req->task_name.c_str());
        cJSON_AddStringToObject(task_info_index_new, "task_type", req->task_type.c_str());
        cJSON_AddItemToArray(task_info_index_array, task_info_index_new);

        char* temp_str = cJSON_Print(TaskInfoIndexJson);
        cJSON_Delete(TaskInfoIndexJson);

        if (!temp_str)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): cJSON_Print(TaskInfoIndexJson) error (" << id_task_info_new << ")!");
            return false;
        }

        std::string TaskInfoIndexContentNew = temp_str;
        free(temp_str);

        std::ofstream TaskInfoIndexFileWrite(TaskInfoIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
        if (!TaskInfoIndexFileWrite)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Can't open TaskInfoIndex.json (" << id_task_info_new << ")!");
            return false;
        }

        TaskInfoIndexFileWrite << TaskInfoIndexContentNew;

        if (!TaskInfoIndexFileWrite.good())
        {
            TaskInfoIndexFileWrite.close();

            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Error: Failed to write to TaskInfoIndex.json (" << id_task_info_new << ")!");
            return false;
        }

        TaskInfoIndexFileWrite.close();

        cJSON* task_info_new = cJSON_CreateObject();
        cJSON_AddStringToObject(task_info_new, "id_task_info", id_task_info_new.c_str());
        cJSON_AddStringToObject(task_info_new, "task_name", req->task_name.c_str());
        cJSON_AddStringToObject(task_info_new, "task_type", req->task_type.c_str());
        cJSON* monitor_points_new =  cJSON_CreateArray();

        std::vector<std::string> monitor_points_new_vec;
        for (size_t i = 0; i < req->monitor_points.size(); i++)
        {
            monitor_points_new_vec.push_back(req->monitor_points[i]);
        }

        std::sort(monitor_points_new_vec.begin(), monitor_points_new_vec.end(), naturalLess);

        for (size_t i = 0; i < req->monitor_points.size(); i++)
        {
            cJSON_AddItemToArray(monitor_points_new, cJSON_CreateString(monitor_points_new_vec[i].c_str()));
        }

        cJSON_AddItemToObject(task_info_new, "monitor_points", monitor_points_new);
        cJSON_AddStringToObject(task_info_new, "create_time", req->create_time.c_str());

        temp_str = cJSON_Print(task_info_new);
        cJSON_Delete(task_info_new);

        if (!temp_str)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): cJSON_Print(task_info_new) error (" << id_task_info_new << ")!");
            return false;
        }

        std::string TaskInfoContentNew = temp_str;
        free(temp_str);

        std::string TaskInfoNewFileDir = TaskFilesDir + "task_info_files/" + req->task_type + "/" + id_task_info_new + ".json";
        std::ofstream TaskInfoNewFileWrite(TaskInfoNewFileDir.c_str(), std::ios::out | std::ios::trunc);
        if (!TaskInfoNewFileWrite)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Cant't open new task info json file (" << id_task_info_new << ")!");
            return false;
        }

        TaskInfoNewFileWrite << TaskInfoContentNew;

        if (!TaskInfoNewFileWrite.good())
        {
            TaskInfoNewFileWrite.close();

            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Error: Failed to write to new task info json file (" << id_task_info_new << ")!");
            return false;
        }

        TaskInfoNewFileWrite.close();

        res->task_not_exist = false;
        res->name_used = false;
        res->id_task_info = id_device + "-task-info-" + id_task_info_new;

        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Successfully create inspect task info.");

        return true;
    }
    else
    {
        size_t TaskInfoIndexArraySize = cJSON_GetArraySize(task_info_index_array);
        std::vector<std::string> IdTaskInfoArray(TaskInfoIndexArraySize);
        for (size_t i = 0; i < TaskInfoIndexArraySize; i++)
        {
            cJSON* task_info_index_temp = cJSON_GetArrayItem(task_info_index_array, i);
            if (task_info_index_temp != NULL)
            {
                cJSON* id_task_info_temp = cJSON_GetObjectItem(task_info_index_temp, "id_task_info");
                cJSON* task_name_temp = cJSON_GetObjectItem(task_info_index_temp, "task_name");
                if (id_task_info_temp != NULL && task_name_temp != NULL)
                {
                    IdTaskInfoArray[i] = id_task_info_temp->valuestring;
                    if (req->task_name == task_name_temp->valuestring)
                    {
                        res->task_not_exist = false;
                        res->name_used = true;
                        cJSON_Delete(TaskInfoIndexJson);
                        return true;
                    }
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Failed to get id_task_info_temp or task_name_temp!");
                    cJSON_Delete(TaskInfoIndexJson);
                    return false;
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Failed to get task_info_index_temp!");
                cJSON_Delete(TaskInfoIndexJson);
                return false;
            }
        }

        std::string id_task_info_new = generateRandomUUID();
        bool id_task_info_flag = true;
        while (id_task_info_flag)
        {
            size_t cnt = 0;
            bool loop_flag = true;
            for (size_t i = 0; i < TaskInfoIndexArraySize && loop_flag; i++)
            {
                cnt++;
                if (id_task_info_new == IdTaskInfoArray[i])
                {
                    id_task_info_new = generateRandomUUID();
                    loop_flag = false;
                    break;
                }
            }

            if (cnt == TaskInfoIndexArraySize)
            {
                id_task_info_flag = false;
            }
        }

        cJSON* task_info_index_new = cJSON_CreateObject();
        cJSON_AddStringToObject(task_info_index_new, "id_task_info", id_task_info_new.c_str());
        cJSON_AddStringToObject(task_info_index_new, "task_name", req->task_name.c_str());
        cJSON_AddStringToObject(task_info_index_new, "task_type", req->task_type.c_str());
        cJSON_AddItemToArray(task_info_index_array, task_info_index_new);

        char* temp_str = cJSON_Print(TaskInfoIndexJson);
        cJSON_Delete(TaskInfoIndexJson);

        if (!temp_str)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): cJSON_Print(TaskInfoIndexJson) error (" << id_task_info_new << ")!");
            return false;
        }

        std::string TaskInfoIndexContentNew = temp_str;
        free(temp_str);

        std::ofstream TaskInfoIndexFileWrite(TaskInfoIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
        if (!TaskInfoIndexFileWrite)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Can't open TaskInfoIndex.json (" << id_task_info_new << ")!");
            return false;
        }

        TaskInfoIndexFileWrite << TaskInfoIndexContentNew;

        if (!TaskInfoIndexFileWrite.good())
        {
            TaskInfoIndexFileWrite.close();

            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Error: Failed to write to TaskInfoIndex.json (" << id_task_info_new << ")!");
            return false;
        }

        TaskInfoIndexFileWrite.close();

        cJSON* task_info_new = cJSON_CreateObject();
        cJSON_AddStringToObject(task_info_new, "id_task_info", id_task_info_new.c_str());
        cJSON_AddStringToObject(task_info_new, "task_name", req->task_name.c_str());
        cJSON_AddStringToObject(task_info_new, "task_type", req->task_type.c_str());
        cJSON* monitor_points_new =  cJSON_CreateArray();

        std::vector<std::string> monitor_points_new_vec;
        for (size_t i = 0; i < req->monitor_points.size(); i++)
        {
            monitor_points_new_vec.push_back(req->monitor_points[i]);
        }

        std::sort(monitor_points_new_vec.begin(), monitor_points_new_vec.end(), naturalLess);

        for (size_t i = 0; i < req->monitor_points.size(); i++)
        {
            cJSON_AddItemToArray(monitor_points_new, cJSON_CreateString(monitor_points_new_vec[i].c_str()));
        }

        cJSON_AddItemToObject(task_info_new, "monitor_points", monitor_points_new);
        cJSON_AddStringToObject(task_info_new, "create_time", req->create_time.c_str());

        temp_str = cJSON_Print(task_info_new);
        cJSON_Delete(task_info_new);

        if (!temp_str)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): cJSON_Print(task_info_new) error (" << id_task_info_new << ")!");
            return false;
        }

        std::string TaskInfoContentNew = temp_str;
        free(temp_str);

        std::string TaskInfoNewFileDir = TaskFilesDir + "task_info_files/" + req->task_type + "/" + id_task_info_new + ".json";
        std::ofstream TaskInfoNewFileWrite(TaskInfoNewFileDir.c_str(), std::ios::out | std::ios::trunc);
        if (!TaskInfoNewFileWrite)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoCreate(): Cant't open new task info json file (" << id_task_info_new << ")!");
            return false;
        }

        TaskInfoNewFileWrite << TaskInfoContentNew;

        if (!TaskInfoNewFileWrite.good())
        {
            TaskInfoNewFileWrite.close();

            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Error: Failed to write to new task info json file (" << id_task_info_new << ")!");
            return false;
        }

        TaskInfoNewFileWrite.close();

        res->task_not_exist = false;
        res->name_used = false;
        res->id_task_info = id_device + "-task-info-" + id_task_info_new;

        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Successfully create inspect task info.");

        return true;
    }
}

bool Task_File_Manager::TaskInfoDelete(const robot_msgs::srv::InspectTaskInfoDeleteCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskInfoDeleteCmd::Response::SharedPtr res)
{
    std::string TaskInfoIndexFileDir = TaskFilesDir + "task_info_files/" + "TaskInfoIndex.json";
    std::ifstream TaskInfoIndexFileRead(TaskInfoIndexFileDir.c_str());
    if (!TaskInfoIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): Can't open TaskInfoIndex.json!");
        return false;
    }

    std::string TaskInfoIndexContent((std::istreambuf_iterator<char>(TaskInfoIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInfoIndexFileRead.close();

    cJSON* TaskInfoIndexJson = cJSON_Parse(TaskInfoIndexContent.c_str());
    if (TaskInfoIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): Failed to parse TaskInfoIndexJson!");
        return false;
    }

    cJSON* task_info_index_array = cJSON_GetObjectItem(TaskInfoIndexJson, "TaskInfoIndex");
    if (task_info_index_array == NULL || !cJSON_IsArray(task_info_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): Failed to get task_info_index_array!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    if (cJSON_GetArraySize(task_info_index_array) == 0)
    {
        res->task_not_exist = true;
        cJSON_Delete(TaskInfoIndexJson);
        return true;
    }

    std::string id_task_info_delete = req->id_task_info;
    std::string id_task_info_erase = id_device + "-task-info-";
    std::regex pattern(id_task_info_erase);
    id_task_info_delete = std::regex_replace(id_task_info_delete, pattern, "");

    size_t TaskInfoIndexArraySize = cJSON_GetArraySize(task_info_index_array);
    bool task_exist = false;
    size_t id_task_info_delete_idx;
    for (size_t i = 0; i < TaskInfoIndexArraySize; i++)
    {
        cJSON* task_info_index_temp = cJSON_GetArrayItem(task_info_index_array, i);
        if (task_info_index_temp != NULL)
        {
            cJSON* id_task_info_temp = cJSON_GetObjectItem(task_info_index_temp, "id_task_info");
            if (id_task_info_temp != NULL)
            {
                if (id_task_info_delete == id_task_info_temp->valuestring)
                {
                    task_exist = true;
                    id_task_info_delete_idx = i;
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): Failed to get id_task_info_temp (" << id_task_info_delete << ")!");
                cJSON_Delete(TaskInfoIndexJson);
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): Failed to get task_info_index_temp (" << id_task_info_delete << ")!");
            cJSON_Delete(TaskInfoIndexJson);
            return false;
        }
    }

    if (!task_exist)
    {
        res->task_not_exist = true;
        cJSON_Delete(TaskInfoIndexJson);
        return true;
    }

    cJSON* task_info_index_delete = cJSON_GetArrayItem(task_info_index_array, id_task_info_delete_idx);
    if (task_info_index_delete == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): Failed to get task_info_index_delete (" << id_task_info_delete << ")!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    cJSON* task_type_delete = cJSON_GetObjectItem(task_info_index_delete, "task_type");
    if (task_type_delete == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): Failed to get task_type_delete (" << id_task_info_delete << ")!");
        cJSON_Delete(TaskInfoIndexJson);
        return false;
    }

    std::string TaskInfoDeleteFileDir = TaskFilesDir + "task_info_files/" + task_type_delete->valuestring + "/" + id_task_info_delete + ".json";
    if (std::remove(TaskInfoDeleteFileDir.c_str()) != 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): Error deleting task info json file (" << id_task_info_delete << ")!");
        res->task_not_exist = false;
        res->deletion_success = false;
        cJSON_Delete(TaskInfoIndexJson);
        return true;
    }

    cJSON* task_info_index_del = cJSON_DetachItemFromArray(task_info_index_array, id_task_info_delete_idx);
    if (task_info_index_del != NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): Failed to detach task_info_index_del (" << id_task_info_delete << ")!");
        cJSON_Delete(task_info_index_del);
    }
    else
    {
        res->task_not_exist = false;
        res->deletion_success = false;
        cJSON_Delete(TaskInfoIndexJson);
        return true;
    }

    char* temp_str = cJSON_Print(TaskInfoIndexJson);
    cJSON_Delete(TaskInfoIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): cJSON_Print(TaskInfoIndexJson) error (" << id_task_info_delete << ")!");
        return false;
    }

    std::string TaskInfoIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskInfoIndexFileWrite(TaskInfoIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskInfoIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): Can't open TaskInfoIndex.json (" << id_task_info_delete << ")!");
        res->task_not_exist = false;
        res->deletion_success = false;
        return true;
    }

    TaskInfoIndexFileWrite << TaskInfoIndexContentNew;

    if (!TaskInfoIndexFileWrite.good())
    {
        TaskInfoIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInfoDelete(): Error: Failed to write to TaskInfoIndex.json (" << id_task_info_delete << ")!");
        res->task_not_exist = false;
        res->deletion_success = false;
        return true;
    }

    TaskInfoIndexFileWrite.close();

    res->task_not_exist = false;
    res->deletion_success = true;

    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Successfully delete inspect task info.");

    return true;
}

/********************************************************任务计划*************************************************************/
bool Task_File_Manager::TaskPlanListQuery(const robot_msgs::srv::InspectTaskPlanListQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanListQuery::Response::SharedPtr res)
{
    std::string id_task_info_query = req->id_task_info;
    std::string id_task_info_erase = id_device + "-task-info-";
    std::regex pattern(id_task_info_erase);
    id_task_info_query = std::regex_replace(id_task_info_query, pattern, "");

    std::string TaskInfoIndexFileDir = TaskFilesDir + "task_info_files/" + "TaskInfoIndex.json";
    std::ifstream TaskInfoIndexFileRead(TaskInfoIndexFileDir.c_str());
    if (!TaskInfoIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanListQuery(): Can't open TaskInfoIndex.json!");
        return false;
    }

    std::string TaskInfoIndexContent((std::istreambuf_iterator<char>(TaskInfoIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInfoIndexFileRead.close();

    if (TaskInfoIndexContent.find(id_task_info_query) == std::string::npos)
    {
        res->task_not_exist = true;
        return true;
    }

    std::string TaskPlanIndexFileDir = TaskFilesDir + "task_plan_files/" + "TaskPlanIndex.json";
    std::ifstream TaskPlanIndexFileRead(TaskPlanIndexFileDir.c_str());
    if (!TaskPlanIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanListQuery(): Can't open TaskPlanIndex.json!");
        return false;
    }

    std::string TaskPlanIndexContent((std::istreambuf_iterator<char>(TaskPlanIndexFileRead)), std::istreambuf_iterator<char>());
    TaskPlanIndexFileRead.close();

    cJSON* TaskPlanIndexJson = cJSON_Parse(TaskPlanIndexContent.c_str());
    std::vector<std::string> id_task_plan_query_array;
    std::vector<std::string> task_plan_type_query_array;
    if (TaskPlanIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanListQuery(): Failed to parse TaskPlanIndexJson!");
        return false;
    }

    cJSON* task_info_plan_index_array = cJSON_GetObjectItem(TaskPlanIndexJson, id_task_info_query.c_str());
    if (task_info_plan_index_array == NULL)
    {
        cJSON_Delete(TaskPlanIndexJson);
        res->task_not_exist = false;

        RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Successfully query inspect task plan list.");
        return true;
    }
    else if (task_info_plan_index_array != NULL && cJSON_IsArray(task_info_plan_index_array))
    {
        size_t TaskInfoPlanIndexArraySize = cJSON_GetArraySize(task_info_plan_index_array);
        for (size_t i = 0; i < TaskInfoPlanIndexArraySize; i++)
        {
            cJSON* task_info_plan_index_temp = cJSON_GetArrayItem(task_info_plan_index_array, i);
            if (task_info_plan_index_temp != NULL)
            {
                cJSON* id_task_plan_temp = cJSON_GetObjectItem(task_info_plan_index_temp, "id_task_plan");
                cJSON* task_plan_type_temp = cJSON_GetObjectItem(task_info_plan_index_temp, "plan_type");
                if (id_task_plan_temp != NULL && task_plan_type_temp != NULL)
                {
                    id_task_plan_query_array.push_back(id_task_plan_temp->valuestring);
                    task_plan_type_query_array.push_back(task_plan_type_temp->valuestring);
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanListQuery(): Failed to get id_task_plan_temp or task_plan_type_temp!");
                    cJSON_Delete(TaskPlanIndexJson);
                    return false;
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanListQuery(): Failed to get task_info_plan_index_temp!");
                cJSON_Delete(TaskPlanIndexJson);
                return false;
            }
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanListQuery(): Failed to get task_info_plan_index_array!");
        cJSON_Delete(TaskPlanIndexJson);
        return false;
    }

    cJSON_Delete(TaskPlanIndexJson);

    for (size_t i = 0; i < id_task_plan_query_array.size(); i++)
    {
        std::string TaskPlanFileDir = TaskFilesDir + "task_plan_files/" + task_plan_type_query_array[i] + "/" + id_task_plan_query_array[i] + ".json";
        std::ifstream TaskPlanFileRead(TaskPlanFileDir.c_str());
        if (!TaskPlanFileRead)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanListQuery(): Can't open task plan json file (" << id_task_plan_query_array[i] << ")!");
            return false;
        }

        std::string TaskPlanContent((std::istreambuf_iterator<char>(TaskPlanFileRead)), std::istreambuf_iterator<char>());
        TaskPlanFileRead.close();

        cJSON* TaskPlanJson = cJSON_Parse(TaskPlanContent.c_str());
        if (TaskPlanJson == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanListQuery(): Failed to parse TaskPlanJson (" << id_task_plan_query_array[i] << ")!");
            return false;
        }

        cJSON* express_query_array = cJSON_GetObjectItem(TaskPlanJson, "express");
        cJSON* create_time_query = cJSON_GetObjectItem(TaskPlanJson, "create_time");
        if (express_query_array != NULL && cJSON_IsArray(express_query_array) && create_time_query != NULL)
        {
            robot_msgs::msg::InspectTaskPlan TaskPlanTemp;
            TaskPlanTemp.id_task_plan = id_device + "-task-plan-" + id_task_plan_query_array[i];
            TaskPlanTemp.plan_type = task_plan_type_query_array[i];
            TaskPlanTemp.create_time = create_time_query->valuestring;
            for (size_t j = 0; j < (size_t)cJSON_GetArraySize(express_query_array); j++)
            {
                cJSON* express_temp = cJSON_GetArrayItem(express_query_array, j);
                if (express_temp != NULL)
                {
                    std_msgs::msg::String express_msg;
                    express_msg.data = express_temp->valuestring;
                    TaskPlanTemp.express.push_back(express_msg.data);
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanListQuery(): Failed to get express_temp (" << id_task_plan_query_array[i] << ")!");
                    cJSON_Delete(TaskPlanJson);
                    return false;
                }
            }
            res->inspect_task_plan_list.push_back(TaskPlanTemp);
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanListQuery(): Failed to get express_query_array or create_time_query (" << id_task_plan_query_array[i] << ")!");
            cJSON_Delete(TaskPlanJson);
            return false;
        }

        cJSON_Delete(TaskPlanJson);
    }

    res->task_not_exist = false;

    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Successfully query inspect task plan list.");

    return true;
}

bool Task_File_Manager::TaskPlanAdd(const robot_msgs::srv::InspectTaskPlanAddCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanAddCmd::Response::SharedPtr res)
{
    std::string id_task_info_plan = req->id_task_info;
    std::string id_task_info_erase = id_device + "-task-info-";
    std::regex pattern(id_task_info_erase);
    id_task_info_plan = std::regex_replace(id_task_info_plan, pattern, "");

    std::string TaskInfoIndexFileDir = TaskFilesDir + "task_info_files/" + "TaskInfoIndex.json";
    std::ifstream TaskInfoIndexFileRead(TaskInfoIndexFileDir.c_str());
    if (!TaskInfoIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Can't open TaskInfoIndex.json!");
        return false;
    }

    std::string TaskInfoIndexContent((std::istreambuf_iterator<char>(TaskInfoIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInfoIndexFileRead.close();

    if (TaskInfoIndexContent.find(id_task_info_plan) == std::string::npos)
    {
        res->task_not_exist = true;
        return true;
    }

    std::string TaskPlanIndexFileDir = TaskFilesDir + "task_plan_files/" + "TaskPlanIndex.json";
    std::ifstream TaskPlanIndexFileRead(TaskPlanIndexFileDir.c_str());
    if (!TaskPlanIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Can't open TaskPlanIndex.json!");
        return false;
    }

    std::string TaskPlanIndexContent((std::istreambuf_iterator<char>(TaskPlanIndexFileRead)), std::istreambuf_iterator<char>());
    TaskPlanIndexFileRead.close();

    cJSON* TaskPlanIndexJson = cJSON_Parse(TaskPlanIndexContent.c_str());
    if (TaskPlanIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Failed to parse TaskPlanIndexJson!");
        return false;
    }

    cJSON* id_task_info_plan_array = cJSON_GetObjectItem(TaskPlanIndexJson, "id_task_info_plan");
    if (id_task_info_plan_array == NULL)
    {
        cJSON* id_task_info_plan_array = cJSON_CreateArray();
        cJSON_AddItemToArray(id_task_info_plan_array, cJSON_CreateString(id_task_info_plan.c_str()));
        cJSON_AddItemToObject(TaskPlanIndexJson, "id_task_info_plan", id_task_info_plan_array);
    }
    else
    {
        if (cJSON_IsArray(id_task_info_plan_array))
        {
            size_t IdTaskInfoPlanArraySize = cJSON_GetArraySize(id_task_info_plan_array);
            bool id_task_info_plan_exist = false;
            bool loop_flag = true;
            for (size_t i = 0; i < IdTaskInfoPlanArraySize && loop_flag; i++)
            {
                cJSON* id_task_info_plan_temp = cJSON_GetArrayItem(id_task_info_plan_array, i);
                if (id_task_info_plan_temp != NULL)
                {
                    if (id_task_info_plan == id_task_info_plan_temp->valuestring)
                    {
                        id_task_info_plan_exist = true;
                        loop_flag = false;
                    }
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Failed to get id_task_info_plan_temp!");
                    cJSON_Delete(TaskPlanIndexJson);
                    res->task_not_exist = false;
                    res->create_success = false;
                    return true;
                }
            }

            if (!id_task_info_plan_exist)
            {
                cJSON_AddItemToArray(id_task_info_plan_array, cJSON_CreateString(id_task_info_plan.c_str()));
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Failed to get id_task_info_plan_array!");
            cJSON_Delete(TaskPlanIndexJson);
            res->task_not_exist = false;
            res->create_success = false;
            return true;
        }
    }

    std::string id_task_plan_new = generateRandomUUID();
    cJSON* id_task_plan_used_array = cJSON_GetObjectItem(TaskPlanIndexJson, "id_task_plan_used");
    if (id_task_plan_used_array == NULL)
    {
        cJSON* id_task_plan_used_array = cJSON_CreateArray();
        cJSON_AddItemToArray(id_task_plan_used_array, cJSON_CreateString(id_task_plan_new.c_str()));
        cJSON_AddItemToObject(TaskPlanIndexJson, "id_task_plan_used", id_task_plan_used_array);
    }
    else
    {
        if (cJSON_IsArray(id_task_plan_used_array))
        {
            size_t IdTaskPlanUsedArraySize = cJSON_GetArraySize(id_task_plan_used_array);
            bool id_task_plan_flag = true;
            while (id_task_plan_flag)
            {
                size_t cnt = 0;
                bool loop_flag = true;
                for (size_t i = 0; i < IdTaskPlanUsedArraySize && loop_flag; i++)
                {
                    cJSON* id_task_plan_used_temp = cJSON_GetArrayItem(id_task_plan_used_array, i);
                    if (id_task_plan_used_temp != NULL)
                    {
                        cnt++;
                        if (id_task_plan_new == id_task_plan_used_temp->valuestring)
                        {
                            id_task_plan_new = generateRandomUUID();
                            loop_flag = false;
                            break;
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Failed to get id_task_plan_used_temp!");
                        cJSON_Delete(TaskPlanIndexJson);
                        res->task_not_exist = false;
                        res->create_success = false;
                        return true;
                    }
                }

                if (cnt == IdTaskPlanUsedArraySize)
                {
                    id_task_plan_flag = false;
                }
            }

            cJSON_AddItemToArray(id_task_plan_used_array, cJSON_CreateString(id_task_plan_new.c_str()));
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Failed to get id_task_plan_used_array!");
            cJSON_Delete(TaskPlanIndexJson);
            res->task_not_exist = false;
            res->create_success = false;
            return true;
        }
    }

    cJSON* task_info_plan_index = cJSON_CreateObject();
    cJSON_AddStringToObject(task_info_plan_index, "id_task_plan", id_task_plan_new.c_str());
    cJSON_AddStringToObject(task_info_plan_index, "plan_type", req->plan_type.c_str());

    cJSON* task_info_plan_index_array = cJSON_GetObjectItem(TaskPlanIndexJson, id_task_info_plan.c_str());
    if (task_info_plan_index_array != NULL)
    {
        if (cJSON_IsArray(task_info_plan_index_array))
        {
            cJSON_AddItemToArray(task_info_plan_index_array, task_info_plan_index);
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Failed to get task_info_plan_index_array!");
            cJSON_Delete(TaskPlanIndexJson);
            res->task_not_exist = false;
            res->create_success = false;
            return true;
        }
    }
    else
    {
        cJSON* task_info_plan_index_array = cJSON_CreateArray();
        cJSON_AddItemToArray(task_info_plan_index_array, task_info_plan_index);
        cJSON_AddItemToObject(TaskPlanIndexJson, id_task_info_plan.c_str(), task_info_plan_index_array);
    }

    char* temp_str = cJSON_Print(TaskPlanIndexJson);
    cJSON_Delete(TaskPlanIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "]TaskPlanAdd(): cJSON_Print(TaskPlanIndexJson) error!");
        return false;
    }

    std::string TaskPlanIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskPlanIndexFileWrite(TaskPlanIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskPlanIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Can't open TaskPlanIndex.json!");
        res->task_not_exist = false;
        res->create_success = false;
        return true;
    }

    TaskPlanIndexFileWrite << TaskPlanIndexContentNew;

    if (!TaskPlanIndexFileWrite.good())
    {
        TaskPlanIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Error: Failed to write to TaskPlanIndex.json!");
        res->task_not_exist = false;
        res->create_success = false;
        return true;
    }

    TaskPlanIndexFileWrite.close();

    cJSON* task_plan_new = cJSON_CreateObject();
    cJSON_AddStringToObject(task_plan_new, "id_task_info", id_task_info_plan.c_str());
    cJSON_AddStringToObject(task_plan_new, "id_task_plan", id_task_plan_new.c_str());
    cJSON_AddStringToObject(task_plan_new, "plan_type", req->plan_type.c_str());
    cJSON* express_new = cJSON_CreateArray();
    for (size_t i = 0; i < req->express.size(); i++)
    {
        cJSON_AddItemToArray(express_new, cJSON_CreateString(req->express[i].c_str()));
    }
    cJSON_AddItemToObject(task_plan_new, "express", express_new);
    cJSON_AddStringToObject(task_plan_new, "create_time", req->create_time.c_str());

    temp_str = cJSON_Print(task_plan_new);
    cJSON_Delete(task_plan_new);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): cJSON_Print(task_plan_new) error!");
        return false;
    }

    std::string TaskPlanContentNew = temp_str;
    free(temp_str);

    std::string TaskPlanNewFileDir = TaskFilesDir + "task_plan_files/" + req->plan_type + "/" + id_task_plan_new + ".json";
    std::ofstream TaskPlanNewFileWrite(TaskPlanNewFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskPlanNewFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Can't open new task plan json file!");
        res->task_not_exist = false;
        res->create_success = false;
        return true;
    }

    TaskPlanNewFileWrite << TaskPlanContentNew;

    if (!TaskPlanNewFileWrite.good())
    {
        TaskPlanNewFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanAdd(): Error: Failed to write to new task plan json file!");
        res->task_not_exist = false;
        res->create_success = false;
        return true;
    }

    TaskPlanNewFileWrite.close();

    res->task_not_exist = false;
    res->create_success = true;
    res->id_task_plan = id_device + "-task-plan-" + id_task_plan_new;

    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Successfully create inspect task plan.");

    return true;
}

bool Task_File_Manager::TaskPlanDelete(const robot_msgs::srv::InspectTaskPlanDeleteCmd::Request::SharedPtr req, robot_msgs::srv::InspectTaskPlanDeleteCmd::Response::SharedPtr res)
{
    std::string id_task_plan_delete = req->id_task_plan;
    std::string id_task_plan_erase = id_device + "-task-plan-";
    std::regex pattern(id_task_plan_erase);
    id_task_plan_delete = std::regex_replace(id_task_plan_delete, pattern, "");

    std::string TaskPlanIndexFileDir = TaskFilesDir + "task_plan_files/" + "TaskPlanIndex.json";
    std::ifstream TaskPlanIndexFileRead(TaskPlanIndexFileDir.c_str());
    if (!TaskPlanIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Can't open TaskPlanIndex.json!");
        return false;
    }

    std::string TaskPlanIndexContent((std::istreambuf_iterator<char>(TaskPlanIndexFileRead)), std::istreambuf_iterator<char>());
    TaskPlanIndexFileRead.close();

    if (TaskPlanIndexContent.find(id_task_plan_delete) == std::string::npos)
    {
        res->task_plan_not_exist = true;
        return true;
    }

    cJSON* TaskPlanIndexJson = cJSON_Parse(TaskPlanIndexContent.c_str());
    std::string task_plan_type_delete;
    if (TaskPlanIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Failed to parse TaskPlanIndexJson!");
        return false;
    }

    std::vector<std::string> id_task_info_plan_vector;
    cJSON* id_task_info_plan_array = cJSON_GetObjectItem(TaskPlanIndexJson, "id_task_info_plan");
    if (id_task_info_plan_array != NULL && cJSON_IsArray(id_task_info_plan_array))
    {
        size_t IdTaskInfoPlanArraySize = cJSON_GetArraySize(id_task_info_plan_array);
        for (size_t i = 0; i < IdTaskInfoPlanArraySize; i++)
        {
            cJSON* id_task_info_plan_temp = cJSON_GetArrayItem(id_task_info_plan_array, i);
            if (id_task_info_plan_temp != NULL)
            {
                id_task_info_plan_vector.push_back(id_task_info_plan_temp->valuestring);
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Failed to get id_task_info_plan_temp!");
                cJSON_Delete(TaskPlanIndexJson);
                res->task_plan_not_exist = false;
                res->deletion_success = false;
                return true;
            }
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Failed to get id_task_info_plan_array!");
        cJSON_Delete(TaskPlanIndexJson);
        res->task_plan_not_exist = false;
        res->deletion_success = false;
        return true;
    }

    cJSON* id_task_plan_used_array = cJSON_GetObjectItem(TaskPlanIndexJson, "id_task_plan_used");
    if (id_task_plan_used_array != NULL && cJSON_IsArray(id_task_plan_used_array))
    {
        size_t IdTaskPlanUsedArraySize = cJSON_GetArraySize(id_task_plan_used_array);
        bool loop_flag = true;
        for (size_t i = 0; i < IdTaskPlanUsedArraySize && loop_flag; i++)
        {
            cJSON* id_task_plan_used_temp = cJSON_GetArrayItem(id_task_plan_used_array, i);
            if (id_task_plan_used_temp != NULL)
            {
                if (id_task_plan_delete == id_task_plan_used_temp->valuestring)
                {
                    cJSON* id_task_plan_used_delete = cJSON_DetachItemFromArray(id_task_plan_used_array, i);
                    if (id_task_plan_used_delete != NULL)
                    {
                        cJSON_Delete(id_task_plan_used_delete);
                    }
                    else
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Failed to detach id_task_plan_delete (" << id_task_plan_delete << ")!");
                        cJSON_Delete(TaskPlanIndexJson);
                        res->task_plan_not_exist = false;
                        res->deletion_success = false;
                        return true;
                    }
                    loop_flag = false;
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Failed to get id_task_plan_used_temp!");
                cJSON_Delete(TaskPlanIndexJson);
                res->task_plan_not_exist = false;
                res->deletion_success = false;
                return true;
            }
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Failed to get id_task_plan_used_array!");
        cJSON_Delete(TaskPlanIndexJson);
        res->task_plan_not_exist = false;
        res->deletion_success = false;
        return true;
    }

    bool loop_flag = true;
    for (size_t i = 0; i < id_task_info_plan_vector.size() && loop_flag; i++)
    {
        cJSON* task_info_plan_index_array = cJSON_GetObjectItem(TaskPlanIndexJson, id_task_info_plan_vector[i].c_str());
        if (task_info_plan_index_array != NULL && cJSON_IsArray(task_info_plan_index_array))
        {
            size_t TaskInfoPlanIndexArraySize = cJSON_GetArraySize(task_info_plan_index_array);
            for (size_t j = 0; j < TaskInfoPlanIndexArraySize && loop_flag; j++)
            {
                cJSON* task_info_plan_index_temp = cJSON_GetArrayItem(task_info_plan_index_array, j);
                if (task_info_plan_index_temp != NULL)
                {
                    cJSON* id_task_plan_temp = cJSON_GetObjectItem(task_info_plan_index_temp, "id_task_plan");
                    cJSON* task_plan_type_temp = cJSON_GetObjectItem(task_info_plan_index_temp, "plan_type");
                    if (id_task_plan_temp != NULL && task_plan_type_temp != NULL)
                    {
                        if (id_task_plan_delete == id_task_plan_temp->valuestring)
                        {
                            task_plan_type_delete = task_plan_type_temp->valuestring;
                            cJSON* task_info_plan_index_delete = cJSON_DetachItemFromArray(task_info_plan_index_array, j);
                            if (task_info_plan_index_delete != NULL)
                            {
                                cJSON_Delete(task_info_plan_index_delete);
                            }
                            else
                            {
                                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Failed to detach task_info_plan_index_delete (" << id_task_plan_delete << ")!");
                                cJSON_Delete(TaskPlanIndexJson);
                                res->task_plan_not_exist = false;
                                res->deletion_success = false;
                                return true;
                            }
                            loop_flag = false;
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Failed to get id_task_plan_temp or task_plan_type_temp!");
                        cJSON_Delete(TaskPlanIndexJson);
                        res->task_plan_not_exist = false;
                        res->deletion_success = false;
                        return true;
                    }
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Failed to get task_info_plan_index_temp!");
                    cJSON_Delete(TaskPlanIndexJson);
                    res->task_plan_not_exist = false;
                    res->deletion_success = false;
                    return true;
                }
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Failed to get task_info_plan_index_array!");
            cJSON_Delete(TaskPlanIndexJson);
            res->task_plan_not_exist = false;
            res->deletion_success = false;
            return true;
        }

    }

    char* temp_str = cJSON_Print(TaskPlanIndexJson);
    cJSON_Delete(TaskPlanIndexJson);

    if (!temp_str)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): cJSON_Print(TaskPlanIndexJson) error (" << id_task_plan_delete << ")!");
        return false;
    }

    std::string TaskPlanIndexContentNew = temp_str;
    free(temp_str);

    std::ofstream TaskPlanIndexFileWrite(TaskPlanIndexFileDir.c_str(), std::ios::out | std::ios::trunc);
    if (!TaskPlanIndexFileWrite)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Can't open TaskPlanIndex.json!");
        res->task_plan_not_exist = false;
        res->deletion_success = false;
        return true;
    }

    TaskPlanIndexFileWrite << TaskPlanIndexContentNew;

    if (!TaskPlanIndexFileRead.good())
    {
        TaskPlanIndexFileWrite.close();

        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Error: Failed to write to TaskPlanIndex.json");
        res->task_plan_not_exist = false;
        res->deletion_success = false;
        return true;
    }

    std::string TaskPlanDeleteFileDir = TaskFilesDir + "task_plan_files/" + task_plan_type_delete + "/" + id_task_plan_delete + ".json";
    if (std::remove(TaskPlanDeleteFileDir.c_str()) != 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskPlanDelete(): Error deleting task plan json file (" << id_task_plan_delete << ")!");
        res->task_plan_not_exist = false;
        res->deletion_success = false;
        return true;
    }

    res->task_plan_not_exist = false;
    res->deletion_success = true;

    RCLCPP_INFO_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] Successfully delete inspect task plan.");

    return true;
}

/********************************************************任务实体*************************************************************/
bool Task_File_Manager::TaskInstanceQuery(const robot_msgs::srv::InspectTaskInstanceQuery::Request::SharedPtr req, robot_msgs::srv::InspectTaskInstanceQuery::Response::SharedPtr res)
{
    std::string TaskInstanceIndexFileDir = TaskFilesDir + "task_instance_files/TaskInstanceIndex.json";
    std::ifstream TaskInstanceIndexFileRead(TaskInstanceIndexFileDir.c_str());
    if (!TaskInstanceIndexFileRead)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Can't open TaskInstanceIndex.json!");
        return false;
    }

    std::string TaskInstanceIndexContent((std::istreambuf_iterator<char>(TaskInstanceIndexFileRead)), std::istreambuf_iterator<char>());
    TaskInstanceIndexFileRead.close();

    cJSON* TaskInstanceIndexJson = cJSON_Parse(TaskInstanceIndexContent.c_str());
    if (TaskInstanceIndexJson == NULL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Failed to parse TaskInstanceIndexJson!");
        return false;
    }

    cJSON* task_instance_index_array = cJSON_GetObjectItem(TaskInstanceIndexJson, "TaskInstanceIndex");
    if (task_instance_index_array == NULL || !cJSON_IsArray(task_instance_index_array))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Failed to get task_instance_index_array!");
        cJSON_Delete(TaskInstanceIndexJson);
        return false;
    }

    size_t TaskInstanceIndexArraySize = cJSON_GetArraySize(task_instance_index_array);

    if (TaskInstanceIndexArraySize == 0)
    {
        cJSON_Delete(TaskInstanceIndexJson);
        return true;
    }

    std::vector<std::tuple<robot_msgs::msg::InspectTaskInstance, std::time_t, std::time_t>> task_instance_array;

    for (size_t i = 0; i < TaskInstanceIndexArraySize; i++)
    {
        cJSON* task_instance_index_temp = cJSON_GetArrayItem(task_instance_index_array, i);
        if (task_instance_index_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Failed to get task_instance_index_temp!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        cJSON* id_task_info_temp = cJSON_GetObjectItem(task_instance_index_temp, "id_task_info");
        cJSON* task_name_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_name");
        cJSON* task_ticket_temp = cJSON_GetObjectItem(task_instance_index_temp, "task_ticket");
        cJSON* status_temp = cJSON_GetObjectItem(task_instance_index_temp, "status");
        cJSON* start_time_temp = cJSON_GetObjectItem(task_instance_index_temp, "start_time");
        cJSON* end_time_temp = cJSON_GetObjectItem(task_instance_index_temp, "end_time");
        cJSON* execute_time_temp = cJSON_GetObjectItem(task_instance_index_temp, "execute_time");

        if (id_task_info_temp == NULL || task_name_temp == NULL || task_ticket_temp == NULL || status_temp == NULL || start_time_temp == NULL || end_time_temp == NULL || execute_time_temp == NULL)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Failed to get id_task_info_temp or task_name_temp or task_ticket_temp or status_temp or start_time_temp or end_time_temp or execute_time_temp!");
            cJSON_Delete(TaskInstanceIndexJson);
            return false;
        }

        robot_msgs::msg::InspectTaskInstance task_instance_temp;
        task_instance_temp.id_task_info = id_device + "-task-info-" + id_task_info_temp->valuestring;
        task_instance_temp.task_name = task_name_temp->valuestring;
        task_instance_temp.task_ticket = id_device + "-task-ticket-" + task_ticket_temp->valuestring;
        task_instance_temp.status = status_temp->valuestring;
        task_instance_temp.start_time = start_time_temp->valuestring;
        task_instance_temp.end_time = end_time_temp->valuestring;
        task_instance_temp.execute_time = execute_time_temp->valueint;

        std::time_t timeStamp_start, timeStamp_end;

        if (!task_instance_temp.start_time.empty())
        {
            std::tm tm = {};

            // 将字符串转换为 std::tm 结构体
            std::istringstream ss(task_instance_temp.start_time);
            ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

            if (ss.fail())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Error parsing start_time.");
                cJSON_Delete(TaskInstanceIndexJson);
                return false;
            }

            // 转换为 Unix 时间戳
            timeStamp_start = std::mktime(&tm);

            if (timeStamp_start == -1)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Error converting start_time to timestamp.");
                cJSON_Delete(TaskInstanceIndexJson);
                return false;
            }
        }

        if (!task_instance_temp.end_time.empty())
        {
            std::tm tm = {};

            // 将字符串转换为 std::tm 结构体
            std::istringstream ss(task_instance_temp.end_time);
            ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

            if (ss.fail())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Error parsing end_time.");
                cJSON_Delete(TaskInstanceIndexJson);
                return false;
            }

            // 转换为 Unix 时间戳
            timeStamp_end = std::mktime(&tm);

            if (timeStamp_end == -1)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Error converting end_time to timestamp.");
                cJSON_Delete(TaskInstanceIndexJson);
                return false;
            }
        }

        std::tuple<robot_msgs::msg::InspectTaskInstance, std::time_t, std::time_t> task_instance_array_temp(task_instance_temp, timeStamp_start, timeStamp_end);

        task_instance_array.push_back(task_instance_array_temp);
    }

    cJSON_Delete(TaskInstanceIndexJson);

    std::string start_time_query = req->start_time;
    std::string end_time_query = req->end_time;
    std::vector<std::string> status_query;
    for (size_t i = 0; i < req->status.size(); i++)
    {
        status_query.push_back(req->status[i]);
    }
    size_t page_size_query = req->page_size;

    std::time_t timeStamp_start_query;
    std::time_t timeStamp_end_query;

    {
        std::tm tm = {};

        // 将字符串转换为 std::tm 结构体
        std::istringstream ss(start_time_query);
        ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

        if (ss.fail())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Error parsing start_time_query.");
            return false;
        }

        // 转换为 Unix 时间戳
        timeStamp_start_query = std::mktime(&tm);

        if (timeStamp_start_query == -1)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Error converting start_time_query to timestamp.");
            return false;
        }
    }

    {
        std::tm tm = {};

        // 将字符串转换为 std::tm 结构体
        std::istringstream ss(end_time_query);
        ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

        if (ss.fail())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Error parsing end_time_query.");
            return false;
        }

        // 转换为 Unix 时间戳
        timeStamp_end_query = std::mktime(&tm);

        if (timeStamp_end_query == -1)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "[" << getCurrentTimeStr() << "] TaskInstanceQuery(): Error converting end_time_query to timestamp.");
            return false;
        }
    }

    std::vector<robot_msgs::msg::InspectTaskInstance> task_instance_query_array;
    for (size_t i = 0; i < task_instance_array.size(); i++)
    {
        bool status_match = false;
        for (size_t j = 0; j < status_query.size(); j++)
        {
            if (std::get<0>(task_instance_array[i]).status == status_query[j])
            {
                status_match = true;
            }
        }

        if (status_match)
        {
            if (std::get<1>(task_instance_array[i]) >= timeStamp_start_query && std::get<1>(task_instance_array[i]) <= timeStamp_end_query)
            {
                task_instance_query_array.push_back(std::get<0>(task_instance_array[i]));
            }
        }
    }

    for (size_t i = 0; i < std::min(page_size_query, task_instance_query_array.size()); i++)
    {
        res->task_instance_list.push_back(task_instance_query_array[i]);
    }

    return true;
}
