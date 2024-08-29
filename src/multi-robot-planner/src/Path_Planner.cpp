/*利用a*生成路径，不做简化，直接corridor，检查连通性。如果不连通报错，如果联通则进行简化*/
/*并且Node使用共享指针*/
/*采用两个膨胀地图分别进行路径规划和corridor生成*/
/*添加rviz的显示*/
#include "Path_Planner.h"

Path_Planner::Path_Planner(ros::NodeHandle &nh) : planner_(nh)
{
    double robot_radius;
    if (!nh.getParam("robot_radius", robot_radius))
    {
        ROS_ERROR("Failed to get param 'robot_radius'");
        robot_radius = 0.5;
    }

    map_sub_ = nh.subscribe("/inflated_map", 1, &Path_Planner::mapCallback, this);
    double_map_sub_ = nh.subscribe("/double_inflated_map", 1, &Path_Planner::doubleMapCallback, this);

    ros::Rate rate(10);
    while (ros::ok() && (!map_received_ || !double_map_received_))
    {
        ros::spinOnce();
        rate.sleep();
    }

    if (map_received_)
    {
        double resolution = map_data_.info.resolution;
        inflation_radius_ = static_cast<int>(std::ceil(robot_radius / resolution));
        // ROS_INFO("Calculated inflation_radius_: %d", inflation_radius_);
    }
}

void Path_Planner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_data_ = *msg;
    original_map_ = map_data_;
    map_received_ = true;
}

void Path_Planner::doubleMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    double_map_data_ = *msg;
    double_map_received_ = true;
}

void Path_Planner::planPaths()
{
    if (!map_received_ || !double_map_received_)
    {
        ROS_WARN("Maps not fully received, cannot plan paths.");
        return;
    }

    const auto &start_positions = planner_.getStartPositions();
    const auto &goal_positions = planner_.getGoalPositions();
    robot_corridors_.resize(start_positions.size());
    ROS_INFO("Get robots starts and goals.");

    int max_steps = 10000000;

    for (size_t i = 0; i < start_positions.size(); ++i)
    {
        nav_msgs::OccupancyGrid current_map = double_map_data_;
        nav_msgs::OccupancyGrid current_map2 = map_data_;


        for (size_t j = 0; j < goal_positions.size(); ++j)
        {
            if (i != j)
            {
                inflateObstacle(goal_positions[j].first, goal_positions[j].second, current_map);
                inflateObstacle(goal_positions[j].first, goal_positions[j].second, current_map2);
            }

        }

        std::vector<std::shared_ptr<Node>> path = astar(current_map,
                                                        start_positions[i].first, start_positions[i].second,
                                                        goal_positions[i].first, goal_positions[i].second, max_steps);

        if (path.empty())
        {
            ROS_WARN("No path found for robot %lu. Skipping corridor generation and connectivity check.", i + 1);
            continue;
        }

        std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> corridors = generateSafeCorridor(current_map2, path);
        bool connected = true;
        for (size_t k = 1; k < corridors.size(); ++k)
        {
            if (!areCorridorsConnected(corridors[k - 1].second, corridors[k].second))
            {
                connected = false;
                ROS_WARN("Path for Robot %lu is not connected between nodes %lu and %lu", i + 1, k, k + 1);
                break;
            }
        }

        if (connected)
        {
            // ROS_INFO("Robot %lu original corridor number is %lu", i + 1, corridors.size());

            simplifyCorridors(corridors);
            // ROS_INFO("Robot %lu simplified corridor number is %lu", i + 1, corridors.size());

            for (size_t k = 1; k < corridors.size(); ++k)
            {
                if (!areCorridorsConnected(corridors[k - 1].second, corridors[k].second))
                {
                    ROS_WARN("Path for Robot %lu is not connected after simplify between nodes %lu and %lu", i + 1, k, k + 1);
                    break;
                }
            }

            removeRedundantCorridors(corridors);
            // ROS_INFO("Robot %lu corridor size after simplification and redundancy removal: %lu", i + 1, corridors.size());

            for (size_t k = 1; k < corridors.size(); ++k)
            {
                if (!areCorridorsConnected(corridors[k - 1].second, corridors[k].second))
                {
                    ROS_WARN("Path for Robot %lu is not connected after remove between nodes %lu and %lu", i + 1, k, k + 1);
                    break;
                }
            }

            simplifyCorridors(corridors);
            // ROS_INFO("Robot %lu simplified corridor number is %lu", i + 1, corridors.size());

            for (size_t k = 1; k < corridors.size(); ++k)
            {
                if (!areCorridorsConnected(corridors[k - 1].second, corridors[k].second))
                {
                    ROS_WARN("Path for Robot %lu is not connected after second simplify between nodes %lu and %lu", i + 1, k, k + 1);
                    break;
                }
            }


        }
        robot_corridors_[i] = corridors;
    }
}

const std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> &Path_Planner::getCorridors(size_t robot_index) const
{
    return robot_corridors_[robot_index];
}

const std::vector<std::pair<int, int>> &Path_Planner::getStartPositions() const
{
    return planner_.getStartPositions();
}

const std::vector<std::pair<int, int>> &Path_Planner::getGoalPositions() const
{
    return planner_.getGoalPositions();
}

std::pair<int, int> Path_Planner::getStart(size_t robot_index) const
{
    return planner_.getStartPositions()[robot_index];
}

std::pair<int, int> Path_Planner::getGoal(size_t robot_index) const
{
    return planner_.getGoalPositions()[robot_index];
}

int Path_Planner::manhattanDistance(int x1, int y1, int x2, int y2)
{
    return abs(x1 - x2) + abs(y1 - y2);
}

std::vector<std::shared_ptr<Node>> Path_Planner::astar(const nav_msgs::OccupancyGrid& map, int start_x, int start_y, int goal_x, int goal_y, int max_steps)
{
    if (start_x < 0 || start_x >= map.info.width ||
        start_y < 0 || start_y >= map.info.height ||
        goal_x < 0 || goal_x >= map.info.width ||
        goal_y < 0 || goal_y >= map.info.height)
    {
        ROS_WARN("Start or goal position is out of map bounds.");
        return std::vector<std::shared_ptr<Node>>();
    }

    if (map.data[start_y * map.info.width + start_x] != 0 ||
        map.data[goal_y * map.info.width + goal_x] != 0)
    {
        if(map.data[start_y * map.info.width + start_x] != 0)
        {
        ROS_WARN("Start  is on an obstacle.");
        }
        if(map.data[goal_y * map.info.width + goal_x] != 0)
        {
        ROS_WARN("Goal  is on an obstacle.");
        }
        return std::vector<std::shared_ptr<Node>>();
    }

    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> open_list;
    std::vector<std::vector<bool>> closed_list(map.info.height, std::vector<bool>(map.info.width, false));

    std::shared_ptr<Node> start_node = std::make_shared<Node>(start_x, start_y, 0, manhattanDistance(start_x, start_y, goal_x, goal_y));
    open_list.push(start_node);

    int steps = 0;

    while (!open_list.empty())
    {
        steps++;

        if (steps > max_steps)
        {
            ROS_WARN("Exceeded maximum search steps (%d). Terminating A* search.", max_steps);
            return std::vector<std::shared_ptr<Node>>();
        }

        std::shared_ptr<Node> current_node = open_list.top();
        open_list.pop();

        if (current_node->x == goal_x && current_node->y == goal_y)
        {
            std::vector<std::shared_ptr<Node>> path;
            std::shared_ptr<Node> path_node = current_node;
            while (path_node)
            {
                path.push_back(path_node);
                path_node = path_node->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (closed_list[current_node->y][current_node->x])
            continue;

        closed_list[current_node->y][current_node->x] = true;

        std::vector<std::pair<int, int>> directions = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };

        for (auto &dir : directions)
        {
            int new_x = current_node->x + dir.first;
            int new_y = current_node->y + dir.second;

            if (new_x >= 0 && new_x < map.info.width &&
                new_y >= 0 && new_y < map.info.height &&
                map.data[new_y * map.info.width + new_x] == 0 &&
                !closed_list[new_y][new_x])
            {
                int new_g = current_node->g + 1;
                int new_h = manhattanDistance(new_x, new_y, goal_x, goal_y);
                std::shared_ptr<Node> new_node = std::make_shared<Node>(new_x, new_y, new_g, new_h, current_node);
                open_list.push(new_node);
            }
        }
    }

    return std::vector<std::shared_ptr<Node>>();
}

std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> Path_Planner::generateSafeCorridor(const nav_msgs::OccupancyGrid& map, const std::vector<std::shared_ptr<Node>> &path)
{
    std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> safe_corridors;
    safe_corridors.reserve(path.size());

    for (const auto &node : path)
    {
        int min_x = node->x;
        int max_x = node->x;
        int min_y = node->y;
        int max_y = node->y;

        bool expand = true;
        while (expand)
        {
            expand = false;

            if (min_x > 0)
            {
                bool obstacle_found = false;
                for (int y = min_y; y <= max_y; ++y)
                {
                    if (map.data[y * map.info.width + (min_x - 1)] != 0)
                    {
                        obstacle_found = true;
                        break;
                    }
                }
                if (!obstacle_found)
                {
                    min_x--;
                    expand = true;
                }
            }

            if (max_x < map.info.width - 1)
            {
                bool obstacle_found = false;
                for (int y = min_y; y <= max_y; ++y)
                {
                    if (map.data[y * map.info.width + (max_x + 1)] != 0)
                    {
                        obstacle_found = true;
                        break;
                    }
                }
                if (!obstacle_found)
                {
                    max_x++;
                    expand = true;
                }
            }

            if (min_y > 0)
            {
                bool obstacle_found = false;
                for (int x = min_x; x <= max_x; ++x)
                {
                    if (map.data[(min_y - 1) * map.info.width + x] != 0)
                    {
                        obstacle_found = true;
                        break;
                    }
                }
                if (!obstacle_found)
                {
                    min_y--;
                    expand = true;
                }
            }

            if (max_y < map.info.height - 1)
            {
                bool obstacle_found = false;
                for (int x = min_x; x <= max_x; ++x)
                {
                    if (map.data[(max_y + 1) * map.info.width + x] != 0)
                    {
                        obstacle_found = true;
                        break;
                    }
                }
                if (!obstacle_found)
                {
                    max_y++;
                    expand = true;
                }
            }
        }

        min_x = std::max(min_x, 0);
        max_x = std::min(max_x, static_cast<int>(map.info.width - 1));
        min_y = std::max(min_y, 0);
        max_y = std::min(max_y, static_cast<int>(map.info.height - 1));

        safe_corridors.emplace_back(node, std::vector<int>{min_x, max_x, min_y, max_y});
    }

    return safe_corridors;
}

bool Path_Planner::areCorridorsConnected(const std::vector<int> &corridor1, const std::vector<int> &corridor2)
{
    return !(corridor1[1] < corridor2[0] || corridor2[1] < corridor1[0] || corridor1[3] < corridor2[2] || corridor2[3] < corridor1[2]);
}

void Path_Planner::simplifyCorridors(std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> &corridors)
{
    if (corridors.empty()) return;

    std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> simplified_corridors;
    simplified_corridors.reserve(corridors.size());
    simplified_corridors.emplace_back(corridors[0]);

    for (size_t i = 1; i < corridors.size(); ++i)
    {
        auto &prev_corridor = simplified_corridors.back().second;
        auto &curr_corridor = corridors[i].second;

        if (curr_corridor[0] >= prev_corridor[0] && curr_corridor[1] <= prev_corridor[1] &&
            curr_corridor[2] >= prev_corridor[2] && curr_corridor[3] <= prev_corridor[3])
        {
            continue;
        }

        if (curr_corridor == prev_corridor)
        {
            continue;
        }

        // curr_corridor[0] = std::max(curr_corridor[0], 0);
        // curr_corridor[1] = std::min(curr_corridor[1], static_cast<int>(map_data_.info.width - 1));
        // curr_corridor[2] = std::max(curr_corridor[2], 0);
        // curr_corridor[3] = std::min(curr_corridor[3], static_cast<int>(map_data_.info.height - 1));

        simplified_corridors.emplace_back(corridors[i]);
    }

    corridors = simplified_corridors;
}

void Path_Planner::removeRedundantCorridors(std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>& corridors) {
    if (corridors.size() <= 2)
    {
        return;
    }

    std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> further_simplified_corridors;
    further_simplified_corridors.reserve(corridors.size());
    further_simplified_corridors.push_back(corridors[0]);
    size_t i = 0;
    bool arr_end = false;
    while (i < corridors.size()) {
        auto current_corridor = corridors[i];

        // if (i == corridors.size() - 1)
        // {
        //     arr_end = true;
        //     break;
        // }

        size_t j = i + 1;

        while (j < corridors.size())
        {
            if (j == corridors.size() - 1)
            {
                if (!areCorridorsConnected(current_corridor.second, corridors[j].second))
                {
                    arr_end = true;
                    further_simplified_corridors.push_back(corridors[j-1]);
                    further_simplified_corridors.push_back(corridors[j]);
                    break;
                }
                else
                {
                    arr_end = true;
                    further_simplified_corridors.push_back(corridors[j]);
                    break;
                }
            }

            if (!areCorridorsConnected(current_corridor.second, corridors[j].second))
            {
                break;
            }

            j++;
        }

        if (!arr_end)
        {
            further_simplified_corridors.push_back(corridors[j-1]);
            i = j-1;
        }
        else
        {
            break;
        }
    }

    corridors = std::move(further_simplified_corridors);
}

void Path_Planner::inflateObstacle(int goal_x, int goal_y, nav_msgs::OccupancyGrid &map)
{
    for (int dx = -2 * inflation_radius_; dx <= 2 * inflation_radius_; ++dx)
    {
        for (int dy = -2 * inflation_radius_; dy <= 2 * inflation_radius_; ++dy)
        {
            int x = goal_x + dx;
            int y = goal_y + dy;
            if (x >= 0 && x < map.info.width && y >= 0 && y < map.info.height)
            {
                map.data[y * map.info.width + x] = 100;
            }
        }
    }
}


void Path_Planner::publishPathVisualization(size_t robot_index, ros::Publisher& marker_pub) {
    visualization_msgs::MarkerArray marker_array;

    // 创建并发布起点Marker
    visualization_msgs::Marker start_marker;
    start_marker.header.frame_id = "map"; // 根据您的tf配置修改
    start_marker.header.stamp = ros::Time::now();
    start_marker.ns = "start_point";
    start_marker.id = 0;
    start_marker.type = visualization_msgs::Marker::SPHERE;
    start_marker.action = visualization_msgs::Marker::ADD;
    start_marker.pose.position.x = getStart(robot_index).first*map_data_.info.resolution;
    start_marker.pose.position.y = getStart(robot_index).second*map_data_.info.resolution;
    start_marker.pose.position.z = 0.0; // 提升高度以便在rviz中更容易看到
    start_marker.scale.x = 0.5; // 设置点的大小
    start_marker.scale.y = 0.5;
    start_marker.scale.z = 0.5;
    start_marker.color.r = 1.0;
    start_marker.color.g = 0.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0; // 透明度

    // 初始化 orientation 为单位四元数
    start_marker.pose.orientation.x = 0.0;
    start_marker.pose.orientation.y = 0.0;
    start_marker.pose.orientation.z = 0.0;
    start_marker.pose.orientation.w = 1.0;

    marker_array.markers.push_back(start_marker);

    // 创建并发布终点Marker
    visualization_msgs::Marker goal_marker = start_marker;
    goal_marker.ns = "goal_point";
    goal_marker.id = 1;
    goal_marker.pose.position.x = getGoal(robot_index).first*map_data_.info.resolution;
    goal_marker.pose.position.y = getGoal(robot_index).second*map_data_.info.resolution;
    goal_marker.color.r = 0.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 1.0;

    // 初始化 orientation 为单位四元数
    goal_marker.pose.orientation.x = 0.0;
    goal_marker.pose.orientation.y = 0.0;
    goal_marker.pose.orientation.z = 0.0;
    goal_marker.pose.orientation.w = 1.0;

    marker_array.markers.push_back(goal_marker);

    // 创建并发布走廊Marker
    const auto& corridors = getCorridors(robot_index);
    // for (const auto &corridor : corridors)
    //     {
    //         ROS_INFO("Node (%d, %d): Corridor [%d, %d] -> [%d, %d]",
    //                  corridor.first->x, corridor.first->y, // node x, y
    //                  corridor.second[0],  corridor.second[1], // min_x, max_x
    //                  corridor.second[2], corridor.second[3]); // max_y, max_y
    //     }
    for (size_t i = 0; i < corridors.size(); ++i) {
        visualization_msgs::Marker corridor_marker;
        corridor_marker.header.frame_id = "map";
        corridor_marker.header.stamp = ros::Time::now();
        corridor_marker.ns = "corridor";
        corridor_marker.id = i + 2; // 起点和终点的ID为0和1
        corridor_marker.type = visualization_msgs::Marker::CUBE;
        corridor_marker.action = visualization_msgs::Marker::ADD;
        corridor_marker.pose.position.x = (corridors[i].second[0] + corridors[i].second[1]) / 2.0*map_data_.info.resolution;
        corridor_marker.pose.position.y = (corridors[i].second[2] + corridors[i].second[3]) / 2.0*map_data_.info.resolution;
        corridor_marker.pose.position.z = 0.5;
        corridor_marker.scale.x = (corridors[i].second[1] - corridors[i].second[0])*map_data_.info.resolution;
        corridor_marker.scale.y = (corridors[i].second[3] - corridors[i].second[2])*map_data_.info.resolution;
        corridor_marker.scale.z = 0.5;
        corridor_marker.color.r = 0.0;
        corridor_marker.color.g = 1.0;
        corridor_marker.color.b = 0.0;
        corridor_marker.color.a = 0.5;

        // 初始化 orientation 为单位四元数
        corridor_marker.pose.orientation.x = 0.0;
        corridor_marker.pose.orientation.y = 0.0;
        corridor_marker.pose.orientation.z = 0.0;
        corridor_marker.pose.orientation.w = 1.0;

        marker_array.markers.push_back(corridor_marker);
    }

    marker_pub.publish(marker_array);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    auto start_time = std::chrono::high_resolution_clock::now();

    std::shared_ptr<Path_Planner> path_planner = std::make_shared<Path_Planner>(nh);

    // 选择一个机器人，比如第一个机器人，发布其路径可视化
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);



    ros::Rate rate(10);
    while (ros::ok() && (!path_planner->mapReceived() || !path_planner->doubleMapReceived()))
    {
        ros::spinOnce();
        rate.sleep();
    }

    path_planner->planPaths();


    const auto &start_positions = path_planner->getStartPositions();
    const auto &goal_positions = path_planner->getGoalPositions();

    for (size_t i = 0; i < start_positions.size(); ++i)
    {
        // ROS_INFO("Robot %lu Start: (%d, %d), Goal: (%d, %d)", i + 1, start_positions[i].first, start_positions[i].second, goal_positions[i].first, goal_positions[i].second);
        std::pair<int, int> start = path_planner->getStart(i);
        std::pair<int, int> goal = path_planner->getGoal(i);
        const auto &corridors = path_planner->getCorridors(i);

        // // 这里可以添加对corridors的使用代码，例如发布到ROS话题或其他处理
        // ROS_INFO("Robot %lu Start: (%d, %d), Goal: (%d, %d)", i + 1, start.first, start.second, goal.first, goal.second);
        // for (const auto &corridor : corridors)
        // {
        //     ROS_INFO("Node (%d, %d): Corridor [%d, %d] -> [%d, %d]",
        //              corridor.first->x, corridor.first->y, // node x, y
        //              corridor.second[0],  corridor.second[1], // min_x, max_x
        //              corridor.second[2], corridor.second[3]); // max_y, max_y
        // }
    }


    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;

    ROS_INFO("Execution time: %.6f seconds", elapsed_time.count());


    // 选择一个机器人，比如第一个机器人，发布其路径可视化
    while (ros::ok())
    {
        size_t robot_index = 0;
        path_planner->publishPathVisualization(robot_index, marker_pub);
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}








































































// /*利用a*生成路径，不做简化，直接corridor，检查连通性。如果不连通报错，如果联通则进行简化*/
// /*并且Node使用共享指针*/
// /*采用两个膨胀地图分别进行路径规划和corridor生成*/

// #include "Path_Planner.h"

// Path_Planner::Path_Planner(ros::NodeHandle &nh) : planner_(nh)
// {
//     // 从参数服务器获取机器人半径
//     double robot_radius;
//     if (!nh.getParam("robot_radius", robot_radius))
//     {
//         ROS_ERROR("Failed to get param 'robot_radius'");
//         robot_radius = 0.5; // 设置默认值
//     }

//     // 订阅两个地图话题
//     map_sub_ = nh.subscribe("/inflated_map", 1, &Path_Planner::mapCallback, this);
//     double_map_sub_ = nh.subscribe("/double_inflated_map", 1, &Path_Planner::doubleMapCallback, this); // 新增

//     // 等待两个地图消息，获取 resolution 并计算 inflation_radius_
//     ros::Rate rate(10);
//     while (ros::ok() && (!map_received_ || !double_map_received_))
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     if (map_received_)
//     {
//         double resolution = map_data_.info.resolution;
//         inflation_radius_ = static_cast<int>(std::ceil(robot_radius / resolution));
//         ROS_INFO("Calculated inflation_radius_: %d", inflation_radius_);
//     }
// }

// void Path_Planner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
// {
//     map_data_ = *msg;
//     original_map_ = map_data_;
//     map_received_ = true;
// }

// void Path_Planner::doubleMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// {
//     double_map_data_ = *msg;
//     double_map_received_ = true;
// }

// void Path_Planner::planPaths()
// {
//     if (!map_received_ || !double_map_received_)
//     {
//         ROS_WARN("Maps not fully received, cannot plan paths.");
//         return;
//     }

//     const auto &start_positions = planner_.getStartPositions();
//     const auto &goal_positions = planner_.getGoalPositions();
//     robot_corridors_.resize(start_positions.size());
//     ROS_INFO("Get robots starts and goals.");

//     int max_steps = 10000000; // 设置最大搜索步数，例如 10000000 步

//     for (size_t i = 0; i < start_positions.size(); ++i)
//     {
//         // 使用 double_map_data_ 作为A*的地图
//         nav_msgs::OccupancyGrid current_map = double_map_data_; // 复制一份

//         // Inflate obstacles for other goals
//         for (size_t j = 0; j < goal_positions.size(); ++j)
//         {
//             if (i != j)
//             {
//                 inflateObstacle(goal_positions[j].first, goal_positions[j].second, current_map);
//             }
//         }

//         // Perform A* on current_map
//         std::vector<std::shared_ptr<Node>> path = astar(current_map,
//                                                         start_positions[i].first, start_positions[i].second,
//                                                         goal_positions[i].first, goal_positions[i].second, max_steps);

//         if (path.empty())
//         {
//             ROS_WARN("No path found for robot %lu. Skipping corridor generation and connectivity check.", i + 1);
//             continue;
//         }

//         // 使用 map_data_ 生成安全走廊
//         std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> corridors = generateSafeCorridor(path);
//         bool connected = true;
//         for (size_t k = 1; k < corridors.size(); ++k)
//         {
//             if (!areCorridorsConnected(corridors[k - 1].second, corridors[k].second))
//             {
//                 connected = false;
//                 ROS_WARN("Path for Robot %lu is not connected between nodes %lu and %lu", i + 1, k, k + 1);
//                 break;
//             }
//         }

//         if (connected)
//         {
//             ROS_INFO("Robot %lu original corridor number is %lu", i + 1, corridors.size());
//             simplifyCorridors(corridors); // 如果连通性存在，则简化通道
//             ROS_INFO("Robot %lu simplified corridor number is %lu", i + 1, corridors.size());
//             removeRedundantCorridors(corridors); // 移除冗余通道
//             ROS_INFO("Robot %lu corridor size after simplification and redundancy removal: %lu", i + 1, corridors.size());
//         }
//         robot_corridors_[i] = corridors; // 保存通道
//     }
// }

// const std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> &Path_Planner::getCorridors(size_t robot_index) const
// {
//     return robot_corridors_[robot_index];
// }

// const std::vector<std::pair<int, int>> &Path_Planner::getStartPositions() const
// {
//     return planner_.getStartPositions();
// }

// const std::vector<std::pair<int, int>> &Path_Planner::getGoalPositions() const
// {
//     return planner_.getGoalPositions();
// }

// std::pair<int, int> Path_Planner::getStart(size_t robot_index) const
// {
//     return planner_.getStartPositions()[robot_index];
// }

// std::pair<int, int> Path_Planner::getGoal(size_t robot_index) const
// {
//     return planner_.getGoalPositions()[robot_index];
// }

// int Path_Planner::manhattanDistance(int x1, int y1, int x2, int y2)
// {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

// std::vector<std::shared_ptr<Node>> Path_Planner::astar(const nav_msgs::OccupancyGrid& map, int start_x, int start_y, int goal_x, int goal_y, int max_steps)
// {
//     // 检查起点和终点是否在地图范围内
//     if (start_x < 0 || start_x >= map.info.width ||
//         start_y < 0 || start_y >= map.info.height ||
//         goal_x < 0 || goal_x >= map.info.width ||
//         goal_y < 0 || goal_y >= map.info.height)
//     {
//         ROS_WARN("Start or goal position is out of map bounds.");
//         return std::vector<std::shared_ptr<Node>>(); // 返回空路径表示无效的起点或终点
//     }

//     // 检查起点和终点是否为障碍物
//     if (map.data[start_y * map.info.width + start_x] != 0 ||
//         map.data[goal_y * map.info.width + goal_x] != 0)
//     {
//         ROS_WARN("Start or goal position is on an obstacle.");
//         return std::vector<std::shared_ptr<Node>>(); // 返回空路径表示无效的起点或终点
//     }

//     std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> open_list;
//     std::vector<std::vector<bool>> closed_list(map.info.height, std::vector<bool>(map.info.width, false));

//     std::shared_ptr<Node> start_node = std::make_shared<Node>(start_x, start_y, 0, manhattanDistance(start_x, start_y, goal_x, goal_y));
//     open_list.push(start_node);

//     int steps = 0;  // 初始化搜索步数计数器

//     while (!open_list.empty())
//     {
//         // 增加步数计数器
//         steps++;

//         // 检查是否超过最大步数
//         if (steps > max_steps)
//         {
//             ROS_WARN("Exceeded maximum search steps (%d). Terminating A* search.", max_steps);
//             return std::vector<std::shared_ptr<Node>>(); // 返回空路径表示搜索失败
//         }

//         std::shared_ptr<Node> current_node = open_list.top();
//         open_list.pop();

//         if (current_node->x == goal_x && current_node->y == goal_y)
//         {
//             std::vector<std::shared_ptr<Node>> path;
//             std::shared_ptr<Node> path_node = current_node;
//             while (path_node)
//             {
//                 path.push_back(path_node);
//                 path_node = path_node->parent;
//             }
//             std::reverse(path.begin(), path.end());
//             return path; // 返回路径
//         }

//         if (closed_list[current_node->y][current_node->x])
//             continue; // 避免多次处理同一个节点

//         closed_list[current_node->y][current_node->x] = true;

//         std::vector<std::pair<int, int>> directions = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };
//         // 如果需要八方向扩展，可以取消注释下面这行
//         // std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

//         for (auto &dir : directions)
//         {
//             int new_x = current_node->x + dir.first;
//             int new_y = current_node->y + dir.second;

//             if (new_x >= 0 && new_x < map.info.width &&
//                 new_y >= 0 && new_y < map.info.height &&
//                 map.data[new_y * map.info.width + new_x] == 0 &&
//                 !closed_list[new_y][new_x])
//             {
//                 int new_g = current_node->g + 1;
//                 int new_h = manhattanDistance(new_x, new_y, goal_x, goal_y);
//                 std::shared_ptr<Node> new_node = std::make_shared<Node>(new_x, new_y, new_g, new_h, current_node);
//                 open_list.push(new_node);
//             }
//         }
//     }

//     return std::vector<std::shared_ptr<Node>>(); // 返回空路径表示未找到路径
// }

// std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> Path_Planner::generateSafeCorridor(const std::vector<std::shared_ptr<Node>> &path)
// {
//     std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> safe_corridors;
//     safe_corridors.reserve(path.size());

//     for (const auto &node : path)
//     {
//         int min_x = node->x;
//         int max_x = node->x;
//         int min_y = node->y;
//         int max_y = node->y;

//         bool expand = true;
//         while (expand)
//         {
//             expand = false;

//             // 向左扩展
//             if (min_x > 0)
//             {
//                 bool obstacle_found = false;
//                 for (int y = min_y; y <= max_y; ++y)
//                 {
//                     if (map_data_.data[y * map_data_.info.width + (min_x - 1)] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     min_x--;
//                     expand = true;
//                 }
//             }

//             // 向右扩展
//             if (max_x < map_data_.info.width - 1)
//             {
//                 bool obstacle_found = false;
//                 for (int y = min_y; y <= max_y; ++y)
//                 {
//                     if (map_data_.data[y * map_data_.info.width + (max_x + 1)] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     max_x++;
//                     expand = true;
//                 }
//             }

//             // 向上扩展
//             if (min_y > 0)
//             {
//                 bool obstacle_found = false;
//                 for (int x = min_x; x <= max_x; ++x)
//                 {
//                     if (map_data_.data[(min_y - 1) * map_data_.info.width + x] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     min_y--;
//                     expand = true;
//                 }
//             }

//             // 向下扩展
//             if (max_y < map_data_.info.height - 1)
//             {
//                 bool obstacle_found = false;
//                 for (int x = min_x; x <= max_x; ++x)
//                 {
//                     if (map_data_.data[(max_y + 1) * map_data_.info.width + x] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     max_y++;
//                     expand = true;
//                 }
//             }
//         }

//         // 再次检查所有方向的边界，确保安全
//         min_x = std::max(min_x, 0);
//         max_x = std::min(max_x, static_cast<int>(map_data_.info.width - 1));
//         min_y = std::max(min_y, 0);
//         max_y = std::min(max_y, static_cast<int>(map_data_.info.height - 1));

//         safe_corridors.emplace_back(node, std::vector<int>{min_x, max_x, min_y, max_y});
//     }

//     return safe_corridors;
// }



// bool Path_Planner::areCorridorsConnected(const std::vector<int> &corridor1, const std::vector<int> &corridor2)
// {
//     return !(corridor1[1] < corridor2[0] || corridor2[1] < corridor1[0] || corridor1[3] < corridor2[2] || corridor2[3] < corridor1[2]);
// }

// void Path_Planner::simplifyCorridors(std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> &corridors)
// {
//     if (corridors.empty()) return;

//     std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> simplified_corridors;
//     simplified_corridors.reserve(corridors.size());
//     simplified_corridors.emplace_back(corridors[0]);

//     for (size_t i = 1; i < corridors.size(); ++i)
//     {
//         auto &prev_corridor = simplified_corridors.back().second;
//         auto &curr_corridor = corridors[i].second;

//         if (curr_corridor[0] >= prev_corridor[0] && curr_corridor[1] <= prev_corridor[1] &&
//             curr_corridor[2] >= prev_corridor[2] && curr_corridor[3] <= prev_corridor[3])
//         {
//             continue;
//         }

//         if (curr_corridor == prev_corridor)
//         {
//             continue;
//         }

//         // 保持在地图范围内
//         curr_corridor[0] = std::max(curr_corridor[0], 0);
//         curr_corridor[1] = std::min(curr_corridor[1], static_cast<int>(map_data_.info.width - 1));
//         curr_corridor[2] = std::max(curr_corridor[2], 0);
//         curr_corridor[3] = std::min(curr_corridor[3], static_cast<int>(map_data_.info.height - 1));

//         simplified_corridors.emplace_back(corridors[i]);
//     }

//     corridors = simplified_corridors;
// }



// void Path_Planner::removeRedundantCorridors(std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>& corridors) {
//     if (corridors.size() <= 2) {
//         return; // 如果走廊数量小于等于2，则无需进一步简化
//     }

//     std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> further_simplified_corridors;
//     further_simplified_corridors.reserve(corridors.size());
//     further_simplified_corridors.push_back(corridors[0]);
//     size_t i = 0;
//     bool arr_end = false;
//     while (i < corridors.size()) {
//         // 将当前走廊作为初始走廊
//         auto current_corridor = corridors[i];

//         if (i == corridors.size() - 1)
//         {
//             arr_end = true;

//             break;
//         }

//         size_t j = i + 1;

//         // 尝试找到第一个可以与current_corridor直接相交的走廊
//         while (j < corridors.size()) {

//             if (j == corridors.size() - 1)
//             {
//                 arr_end = true;

//                 // ROS_INFO(" GET THE GOAL !! Node (%d, %d): Corridor [%d, %d] -> [%d, %d]",
//                 //      corridors[j].first->x, corridors[j].first->y, // node x, y
//                 //      corridors[j].second[0], corridors[j].second[2],  // min_x, min_y
//                 //      corridors[j].second[1], corridors[j].second[3]); // max_x, max_y)

//                 further_simplified_corridors.push_back(corridors[j]);
//                 break;
//             }

//             if (!areCorridorsConnected(current_corridor.second, corridors[j].second)) 
//             {
//                 break; // 一旦无法直接相交，停止扩展
//             }
//             j++;
//         }

//         if (!arr_end) {
//             // 将扩展后的走廊加入结果列表
//             further_simplified_corridors.push_back(corridors[j-1]);
//             // 更新i为j，继续检查下一个未处理的走廊
//             i = j-1;
//         }
//         else
//         {
//             break;
//         }


//     }

//     corridors = std::move(further_simplified_corridors); // 更新走廊列表
// }



// void Path_Planner::inflateObstacle(int goal_x, int goal_y, nav_msgs::OccupancyGrid &map)
// {
//     for (int dx = -2 * inflation_radius_; dx <= 2 * inflation_radius_; ++dx)
//     {
//         for (int dy = -2 * inflation_radius_; dy <= 2 * inflation_radius_; ++dy)
//         {
//             int x = goal_x + dx;
//             int y = goal_y + dy;
//             if (x >= 0 && x < map.info.width && y >= 0 && y < map.info.height)
//             {
//                 map.data[y * map.info.width + x] = 100;
//             }
//         }
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "path_planner_example");
//     ros::NodeHandle nh;

//     auto start_time = std::chrono::high_resolution_clock::now();

//     // 使用shared_ptr在堆中创建Path_Planner对象
//     std::shared_ptr<Path_Planner> path_planner = std::make_shared<Path_Planner>(nh);

//     // 等待两个地图都接收完毕
//     ros::Rate rate(10);
//     while (ros::ok() && (!path_planner->mapReceived() || !path_planner->doubleMapReceived()))
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     path_planner->planPaths();

//     const auto &start_positions = path_planner->getStartPositions();
//     const auto &goal_positions = path_planner->getGoalPositions();




//     for (size_t i = 0; i < start_positions.size(); ++i)
//     {
//         std::pair<int, int> start = path_planner->getStart(i);
//         std::pair<int, int> goal = path_planner->getGoal(i);
//         const auto &corridors = path_planner->getCorridors(i);

//         // 这里可以添加对corridors的使用代码，例如发布到ROS话题或其他处理
//         ROS_INFO("Robot %lu Start: (%d, %d), Goal: (%d, %d)", i + 1, start.first, start.second, goal.first, goal.second);
//         for (const auto &corridor : corridors)
//         {
//             ROS_INFO("Node (%d, %d): Corridor [%d, %d] -> [%d, %d]",
//                      corridor.first->x, corridor.first->y, // node x, y
//                      corridor.second[0],  corridor.second[1], // min_x, max_x
//                      corridor.second[2], corridor.second[3]); // max_y, max_y
//         }
//     }



//     // 记录结束时间
//     auto end_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed_time = end_time - start_time;

//     // 输出执行时间
//     ROS_INFO("Execution time: %.6f seconds", elapsed_time.count());

//     return 0;
// }









































// /*利用a*生成路径，不做简化，直接corridor，检查连通性。如果不连通报错，如果联通则进行简化*/
// /*并且Node使用共享指针*/

// #include "Path_Planner.h"

// Path_Planner::Path_Planner(ros::NodeHandle &nh) : planner_(nh)
// {
//     // 从参数服务器获取机器人半径
//     double robot_radius;
//     if (!nh.getParam("robot_radius", robot_radius))
//     {
//         ROS_ERROR("Failed to get param 'robot_radius'");
//         robot_radius = 0.5; // 设置默认值
//     }

//     map_sub_ = nh.subscribe("/inflated_map", 1, &Path_Planner::mapCallback, this);

//     // 等待地图消息，获取 resolution 并计算 inflation_radius_
//     ros::Rate rate(10);
//     while (ros::ok() && !map_received_)
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     if (map_received_)
//     {
//         double resolution = map_data_.info.resolution;
//         inflation_radius_ = static_cast<int>(std::ceil(robot_radius / resolution));
//         ROS_INFO("Calculated inflation_radius_: %d", inflation_radius_);
//     }
// }

// void Path_Planner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
// {
//     map_data_ = *msg;
//     original_map_ = map_data_;
//     map_received_ = true;
// }

// void Path_Planner::planPaths()
// {
//     if (!map_received_)
//     {
//         ROS_WARN("Map not received, cannot plan paths.");
//         return;
//     }

//     const auto &start_positions = planner_.getStartPositions();
//     const auto &goal_positions = planner_.getGoalPositions();
//     robot_corridors_.resize(start_positions.size());
//     ROS_INFO("Get robots statrs and goals. ");

//     int max_steps = 100000000; // 设置最大搜索步数，例如 100000000 步

//     for (size_t i = 0; i < start_positions.size(); ++i)
//     {
//         map_data_ = original_map_;

//         for (size_t j = 0; j < goal_positions.size(); ++j)
//         {
//             if (i != j)
//             {
//                 inflateObstacle(goal_positions[j].first, goal_positions[j].second, map_data_);
//             }
//         }

//         std::vector<std::shared_ptr<Node>> path = astar(start_positions[i].first, start_positions[i].second,
//                                                         goal_positions[i].first, goal_positions[i].second, max_steps);

//         if (path.empty())
//         {
//             ROS_WARN("No path found for robot %lu. Skipping corridor generation and connectivity check.", i + 1);
//             continue;
//         }

//         std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> corridors = generateSafeCorridor(path);
//         bool connected = true;
//         for (size_t k = 1; k < corridors.size(); ++k)
//         {
//             if (!areCorridorsConnected(corridors[k - 1].second, corridors[k].second))
//             {
//                 connected = false;
//                 ROS_WARN("Path for Robot %lu is not connected between nodes %lu and %lu", i + 1, k, k + 1);
//                 break;
//             }
//         }

//         if (connected)
//         {   ROS_INFO("Robot %lu Corridor is %lu", i + 1, corridors.size());
//             simplifyCorridors(corridors); // 如果连通性存在，则简化通道
//             ROS_INFO("Robot %lu simplified Corridor is %lu", i + 1, corridors.size());
//         }
//         robot_corridors_[i] = corridors; // 保存通道
//     }
// }

// const std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> &Path_Planner::getCorridors(size_t robot_index) const
// {
//     return robot_corridors_[robot_index];
// }

// const std::vector<std::pair<int, int>> &Path_Planner::getStartPositions() const
// {
//     return planner_.getStartPositions();
// }

// const std::vector<std::pair<int, int>> &Path_Planner::getGoalPositions() const
// {
//     return planner_.getGoalPositions();
// }

// std::pair<int, int> Path_Planner::getStart(size_t robot_index) const
// {
//     return planner_.getStartPositions()[robot_index];
// }

// std::pair<int, int> Path_Planner::getGoal(size_t robot_index) const
// {
//     return planner_.getGoalPositions()[robot_index];
// }

// int Path_Planner::manhattanDistance(int x1, int y1, int x2, int y2)
// {
//     return abs(x1 - x2) + abs(y1 - y2);
// }




// std::vector<std::shared_ptr<Node>> Path_Planner::astar(int start_x, int start_y, int goal_x, int goal_y, int max_steps)
// {
//     // 检查起点和终点是否在地图范围内
//     if (start_x < 0 || start_x >= map_data_.info.width ||
//         start_y < 0 || start_y >= map_data_.info.height ||
//         goal_x < 0 || goal_x >= map_data_.info.width ||
//         goal_y < 0 || goal_y >= map_data_.info.height)
//     {
//         ROS_WARN("Start or goal position is out of map bounds.");
//         return std::vector<std::shared_ptr<Node>>(); // 返回空路径表示无效的起点或终点
//     }

//     // 检查起点和终点是否为障碍物
//     if (map_data_.data[start_y * map_data_.info.width + start_x] != 0 ||
//         map_data_.data[goal_y * map_data_.info.width + goal_x] != 0)
//     {
//         ROS_WARN("Start or goal position is on an obstacle.");
//         return std::vector<std::shared_ptr<Node>>(); // 返回空路径表示无效的起点或终点
//     }

//     std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> open_list;
//     std::vector<std::vector<bool>> closed_list(map_data_.info.height, std::vector<bool>(map_data_.info.width, false));

//     std::shared_ptr<Node> start_node = std::make_shared<Node>(start_x, start_y, 0, manhattanDistance(start_x, start_y, goal_x, goal_y));
//     open_list.push(start_node);

//     int steps = 0;  // 初始化搜索步数计数器

//     while (!open_list.empty())
//     {
//         // 增加步数计数器
//         steps++;
        
//         // 检查是否超过最大步数
//         if (steps > max_steps)
//         {
//             ROS_WARN("Exceeded maximum search steps (%d). Terminating A* search.", max_steps);
//             return std::vector<std::shared_ptr<Node>>(); // 返回空路径表示搜索失败
//         }

//         std::shared_ptr<Node> current_node = open_list.top();
//         open_list.pop();

//         if (current_node->x == goal_x && current_node->y == goal_y)
//         {
//             std::vector<std::shared_ptr<Node>> path;
//             std::shared_ptr<Node> path_node = current_node;
//             while (path_node)
//             {
//                 path.push_back(path_node);
//                 path_node = path_node->parent;
//             }
//             std::reverse(path.begin(), path.end());
//             return path; // 返回路径
//         }

//         closed_list[current_node->y][current_node->x] = true;

//         std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
//         // std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};


//         for (auto &dir : directions)
//         {
//             int new_x = current_node->x + dir.first;
//             int new_y = current_node->y + dir.second;

//             if (new_x >= 0 && new_x < map_data_.info.width &&
//                 new_y >= 0 && new_y < map_data_.info.height &&
//                 map_data_.data[new_y * map_data_.info.width + new_x] == 0 &&
//                 !closed_list[new_y][new_x])
//             {
//                 int new_g = current_node->g + 1;
//                 int new_h = manhattanDistance(new_x, new_y, goal_x, goal_y);
//                 std::shared_ptr<Node> new_node = std::make_shared<Node>(new_x, new_y, new_g, new_h, current_node);
//                 open_list.push(new_node);
//             }
//         }
//     }

//     return std::vector<std::shared_ptr<Node>>(); // 返回空路径表示未找到路径
// }



// std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> Path_Planner::generateSafeCorridor(const std::vector<std::shared_ptr<Node>> &path)
// {
//     std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> safe_corridors;
//     safe_corridors.reserve(path.size());

//     for (const auto &node : path)
//     {
//         int min_x = node->x;
//         int max_x = node->x;
//         int min_y = node->y;
//         int max_y = node->y;

//         bool expand = true;
//         while (expand)
//         {
//             expand = false;

//             // 向左扩展
//             if (min_x > 0)
//             {
//                 bool obstacle_found = false;
//                 for (int y = min_y; y <= max_y; ++y)
//                 {
//                     if (map_data_.data[y * map_data_.info.width + (min_x - 1)] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     min_x--;
//                     expand = true;
//                 }
//             }

//             // 向右扩展
//             if (max_x < map_data_.info.width - 1)
//             {
//                 bool obstacle_found = false;
//                 for (int y = min_y; y <= max_y; ++y)
//                 {
//                     if (map_data_.data[y * map_data_.info.width + (max_x + 1)] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     max_x++;
//                     expand = true;
//                 }
//             }

//             // 向上扩展
//             if (min_y > 0)
//             {
//                 bool obstacle_found = false;
//                 for (int x = min_x; x <= max_x; ++x)
//                 {
//                     if (map_data_.data[(min_y - 1) * map_data_.info.width + x] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     min_y--;
//                     expand = true;
//                 }
//             }

//             // 向下扩展
//             if (max_y < map_data_.info.height - 1)
//             {
//                 bool obstacle_found = false;
//                 for (int x = min_x; x <= max_x; ++x)
//                 {
//                     if (map_data_.data[(max_y + 1) * map_data_.info.width + x] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     max_y++;
//                     expand = true;
//                 }
//             }
//         }

//         // 再次检查所有方向的边界，确保安全
//         min_x = std::max(min_x, 0);
//         max_x = std::min(max_x, static_cast<int>(map_data_.info.width - 1));
//         min_y = std::max(min_y, 0);
//         max_y = std::min(max_y, static_cast<int>(map_data_.info.height - 1));

//         safe_corridors.push_back({node, {min_x, max_x, min_y, max_y}});
//     }

//     return safe_corridors;
// }

// bool Path_Planner::areCorridorsConnected(const std::vector<int> &corridor1, const std::vector<int> &corridor2)
// {
//     return !(corridor1[1] < corridor2[0] || corridor2[1] < corridor1[0] || corridor1[3] < corridor2[2] || corridor2[3] < corridor1[2]);
// }

// void Path_Planner::simplifyCorridors(std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> &corridors)
// {
//     std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> simplified_corridors;
//     simplified_corridors.reserve(corridors.size());
//     simplified_corridors.push_back(corridors[0]);

//     for (size_t i = 1; i < corridors.size(); ++i)
//     {
//         auto &prev_corridor = simplified_corridors.back().second;
//         auto &curr_corridor = corridors[i].second;

//         if (curr_corridor[0] >= prev_corridor[0] && curr_corridor[1] <= prev_corridor[1] &&
//             curr_corridor[2] >= prev_corridor[2] && curr_corridor[3] <= prev_corridor[3])
//         {
//             continue;
//         }

//         if (curr_corridor == prev_corridor)
//         {
//             continue;
//         }

//         curr_corridor[0] = std::max(curr_corridor[0], 0);
//         curr_corridor[1] = std::min(curr_corridor[1], static_cast<int>(map_data_.info.width - 1));
//         curr_corridor[2] = std::max(curr_corridor[2], 0);
//         curr_corridor[3] = std::min(curr_corridor[3], static_cast<int>(map_data_.info.height - 1));

//         simplified_corridors.push_back(corridors[i]);
//     }

//     corridors = simplified_corridors;
// }

// void Path_Planner::inflateObstacle(int goal_x, int goal_y, nav_msgs::OccupancyGrid &map)
// {
//     for (int dx = -2.0 * inflation_radius_; dx <= 2.0 * inflation_radius_; ++dx)
//     {
//         for (int dy = -2.0 * inflation_radius_; dy <= 2.0 * inflation_radius_; ++dy)
//         {
//             int x = goal_x + dx;
//             int y = goal_y + dy;
//             if (x >= 0 && x < map.info.width && y >= 0 && y < map.info.height)
//             {
//                 map.data[y * map.info.width + x] = 100;
//             }
//         }
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "path_planner_example");
//     ros::NodeHandle nh;

//     auto start_time = std::chrono::high_resolution_clock::now();

//     // 使用shared_ptr在堆中创建Path_Planner对象
//     std::shared_ptr<Path_Planner> path_planner = std::make_shared<Path_Planner>(nh);

//     while (ros::ok() && !path_planner->mapReceived())
//     {
//         ros::spinOnce();
//     }

//     path_planner->planPaths();

//     const auto &start_positions = path_planner->getStartPositions();
//     const auto &goal_positions = path_planner->getGoalPositions();

//     for (size_t i = 0; i < start_positions.size(); ++i)
//     {
//         std::pair<int, int> start = path_planner->getStart(i);
//         std::pair<int, int> goal = path_planner->getGoal(i);
//         const auto &corridors = path_planner->getCorridors(i);
//     }

//     // 记录结束时间
//     auto end_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed_time = end_time - start_time;

//     // 输出执行时间
//     ROS_INFO("Execution time: %.6f seconds", elapsed_time.count());

//     return 0;
// }



































// /*在生成了路径之后，做简化路径，并且在两个corridor不连通时采用中点插值的方法确保联通性*/
// /*但是会出现中点插值收敛到一个点的现象*/
// #include "Path_Planner.h"

// Path_Planner::Path_Planner(ros::NodeHandle &nh) : planner_(nh)
// {
//     // 从参数服务器获取机器人半径
//     double robot_radius;
//     if (!nh.getParam("robot_radius", robot_radius))
//     {
//         ROS_ERROR("Failed to get param 'robot_radius'");
//         robot_radius = 0.5; // 设置默认值
//     }

//     map_sub_ = nh.subscribe("/inflated_map", 1, &Path_Planner::mapCallback, this);

//     // 等待地图消息，获取 resolution 并计算 inflation_radius_
//     ros::Rate rate(10);
//     while (ros::ok() && !map_received_)
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     if (map_received_)
//     {
//         double resolution = map_data_.info.resolution;
//         inflation_radius_ = static_cast<int>(std::ceil(robot_radius / resolution));
//         ROS_INFO("Calculated inflation_radius_: %d", inflation_radius_);
//     }
// }

// void Path_Planner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
// {
//     map_data_ = *msg;
//     original_map_ = map_data_;
//     map_received_ = true;
// }

// void Path_Planner::planPaths()
// {
//     if (!map_received_)
//     {
//         ROS_WARN("Map not received, cannot plan paths.");
//         return;
//     }

//     const auto &start_positions = planner_.getStartPositions();
//     const auto &goal_positions = planner_.getGoalPositions();
//     robot_corridors_.resize(start_positions.size());

//     for (size_t i = 0; i < start_positions.size(); ++i)
//     {
//         // ROS_INFO("Robot %lu: Start (%d, %d), Goal (%d, %d)", i + 1,
//         //          start_positions[i].first, start_positions[i].second,
//         //          goal_positions[i].first, goal_positions[i].second);

//         // 恢复地图为原始状态
//         map_data_ = original_map_;

//         // 将其他机器人的终点及其膨胀区域标记为障碍物
//         for (size_t j = 0; j < goal_positions.size(); ++j)
//         {
//             if (i != j)
//             {
//                 inflateObstacle(goal_positions[j].first, goal_positions[j].second, map_data_);
//             }
//         }

//         std::vector<Node> path = astar(start_positions[i].first, start_positions[i].second,
//                                        goal_positions[i].first, goal_positions[i].second);

//         if (!path.empty())
//         {
//             std::vector<std::pair<Node, std::vector<int>>> corridors = generateSafeCorridor(path);
//             ROS_INFO("Corridors for Robot %lu are get", i + 1);
//             ensureCorridorsConnected(path, corridors); // 确保 corridor 连通
//             ROS_INFO("Robot %lu Corridor is connected", i + 1);
//             simplifyCorridors(corridors); // 对 corridors 进行进一步简化
//             ROS_INFO("Robot %lu Corridor is simplified", i + 1);
//             robot_corridors_[i] = corridors; // 保存corridors

//             // ROS_INFO("Corridors for Robot %lu:", i + 1);
//             // for (const auto& corridor : corridors) {
//             //     ROS_INFO("Node (%d, %d): Corridor [%d, %d] -> [%d, %d]",
//             //              corridor.first.x, corridor.first.y,
//             //              corridor.second[0], corridor.second[2],  // min_x, min_y
//             //              corridor.second[1], corridor.second[3]); // max_x, max_y
//             // }
//         }
//         else
//         {
//             ROS_WARN("No path found for robot %lu.", i + 1);
//         }
//     }
// }

// const std::vector<std::pair<Node, std::vector<int>>> &Path_Planner::getCorridors(size_t robot_index) const
// {
//     return robot_corridors_[robot_index];
// }

// const std::vector<std::pair<int, int>> &Path_Planner::getStartPositions() const
// {
//     return planner_.getStartPositions();
// }

// const std::vector<std::pair<int, int>> &Path_Planner::getGoalPositions() const
// {
//     return planner_.getGoalPositions();
// }

// std::pair<int, int> Path_Planner::getStart(size_t robot_index) const
// {
//     return planner_.getStartPositions()[robot_index];
// }

// std::pair<int, int> Path_Planner::getGoal(size_t robot_index) const
// {
//     return planner_.getGoalPositions()[robot_index];
// }

// int Path_Planner::manhattanDistance(int x1, int y1, int x2, int y2)
// {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

// bool Path_Planner::isLineFree(int x1, int y1, int x2, int y2)
// {
//     int dx = abs(x2 - x1);
//     int dy = abs(y2 - y1);
//     int sx = (x1 < x2) ? 1 : -1;
//     int sy = (y1 < y2) ? 1 : -1;
//     int err = dx - dy;

//     while (true)
//     {
//         if (map_data_.data[y1 * map_data_.info.width + x1] != 0)
//         {
//             return false; // 如果线段经过障碍物，返回false
//         }
//         if (x1 == x2 && y1 == y2)
//         {
//             break; // 到达终点
//         }
//         int e2 = 2 * err;
//         if (e2 > -dy)
//         {
//             err -= dy;
//             x1 += sx;
//         }
//         if (e2 < dx)
//         {
//             err += dx;
//             y1 += sy;
//         }
//     }
//     return true;
// }

// std::vector<Node> Path_Planner::simplifyPath(const std::vector<Node> &path)
// {
//     if (path.size() < 3)
//     {
//         return path; // 如果路径点少于3个，则无需简化
//     }

//     std::vector<Node> simplified_path;
//     simplified_path.reserve(path.size()); // 预留内存以提高性能
//     simplified_path.push_back(path.front());

//     for (size_t i = 2; i < path.size(); ++i)
//     {
//         if (!isLineFree(simplified_path.back().x, simplified_path.back().y, path[i].x, path[i].y))
//         {
//             simplified_path.push_back(path[i - 1]);
//         }
//     }

//     simplified_path.push_back(path.back());
//     ROS_INFO("Path simplified from %lu to %lu nodes", path.size(), simplified_path.size());
//     ROS_INFO("Simplified_path are:");
//     for (const auto &node : simplified_path)
//     {
//         ROS_INFO("Node (%d, %d)", node.x, node.y);
//     }
//     return simplified_path;
// }

// std::vector<Node> Path_Planner::astar(int start_x, int start_y, int goal_x, int goal_y)
// {
//     std::priority_queue<Node *, std::vector<Node *>, CompareNode> open_list;
//     std::vector<std::vector<bool>> closed_list(map_data_.info.height, std::vector<bool>(map_data_.info.width, false));

//     Node *start_node = new Node(start_x, start_y, 0, manhattanDistance(start_x, start_y, goal_x, goal_y));
//     open_list.push(start_node);

//     while (!open_list.empty())
//     {
//         Node *current_node = open_list.top();
//         open_list.pop();

//         if (current_node->x == goal_x && current_node->y == goal_y)
//         {
//             std::vector<Node> path;
//             Node *path_node = current_node;
//             while (path_node)
//             {
//                 path.push_back(*path_node);
//                 path_node = path_node->parent;
//             }
//             std::reverse(path.begin(), path.end());
//             return simplifyPath(path); // 返回简化后的路径
//         }

//         closed_list[current_node->y][current_node->x] = true;

//         std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
//         // std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

//         for (auto &dir : directions)
//         {
//             int new_x = current_node->x + dir.first;
//             int new_y = current_node->y + dir.second;

//             if (new_x >= 0 && new_x < map_data_.info.width &&
//                 new_y >= 0 && new_y < map_data_.info.height &&
//                 map_data_.data[new_y * map_data_.info.width + new_x] == 0 &&
//                 !closed_list[new_y][new_x])
//             {

//                 int new_g = current_node->g + 1;
//                 int new_h = manhattanDistance(new_x, new_y, goal_x, goal_y);
//                 Node *new_node = new Node(new_x, new_y, new_g, new_h, current_node);
//                 open_list.push(new_node);
//             }
//         }
//     }

//     return std::vector<Node>(); // 返回空路径表示未找到路径
// }

// std::vector<std::pair<Node, std::vector<int>>> Path_Planner::generateSafeCorridor(const std::vector<Node> &path)
// {
//     std::vector<std::pair<Node, std::vector<int>>> safe_corridors;
//     safe_corridors.reserve(path.size()); // 预留内存以提高性能

//     for (const auto &node : path)
//     {
//         int min_x = node.x;
//         int max_x = node.x;
//         int min_y = node.y;
//         int max_y = node.y;

//         bool expand = true;
//         while (expand)
//         {
//             expand = false;

//             // 向左扩展
//             if (min_x > 0)
//             {
//                 bool obstacle_found = false;
//                 for (int y = min_y; y <= max_y; ++y)
//                 {
//                     if (map_data_.data[y * map_data_.info.width + (min_x - 1)] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     min_x--;
//                     expand = true;
//                 }
//             }

//             // 向右扩展
//             if (max_x < map_data_.info.width - 1)
//             {
//                 bool obstacle_found = false;
//                 for (int y = min_y; y <= max_y; ++y)
//                 {
//                     if (map_data_.data[y * map_data_.info.width + (max_x + 1)] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     max_x++;
//                     expand = true;
//                 }
//             }

//             // 向上扩展
//             if (min_y > 0)
//             {
//                 bool obstacle_found = false;
//                 for (int x = min_x; x <= max_x; ++x)
//                 {
//                     if (map_data_.data[(min_y - 1) * map_data_.info.width + x] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     min_y--;
//                     expand = true;
//                 }
//             }

//             // 向下扩展
//             if (max_y < map_data_.info.height - 1)
//             {
//                 bool obstacle_found = false;
//                 for (int x = min_x; x <= max_x; ++x)
//                 {
//                     if (map_data_.data[(max_y + 1) * map_data_.info.width + x] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     max_y++;
//                     expand = true;
//                 }
//             }
//         }

//         // 再次检查所有方向的边界，确保安全
//         min_x = std::max(min_x, 0);
//         max_x = std::min(max_x, static_cast<int>(map_data_.info.width - 1));
//         min_y = std::max(min_y, 0);
//         max_y = std::min(max_y, static_cast<int>(map_data_.info.height - 1));

//         safe_corridors.push_back({node, {min_x, max_x, min_y, max_y}});
//     }

//     return safe_corridors;
// }

// bool Path_Planner::areCorridorsConnected(const std::vector<int> &corridor1, const std::vector<int> &corridor2)
// {
//     return !(corridor1[1] < corridor2[0] || corridor2[1] < corridor1[0] || corridor1[3] < corridor2[2] || corridor2[3] < corridor1[2]);
// }

// void Path_Planner::ensureCorridorsConnected(std::vector<Node> &path, std::vector<std::pair<Node, std::vector<int>>> &corridors)
// {
//     for (size_t i = 1; i < corridors.size(); ++i)
//     {
//         if (!areCorridorsConnected(corridors[i - 1].second, corridors[i].second))
//         {
//             ROS_INFO("Corridors are not connected, inserting a new node between (%d, %d) and (%d, %d)",
//                      path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
//             int mid_x = (path[i - 1].x + path[i].x) / 2;
//             int mid_y = (path[i - 1].y + path[i].y) / 2;
//             Node mid_node(mid_x, mid_y, 0, 0);
//             ROS_INFO("New node: (%d, %d)", mid_node.x, mid_node.y);

//             path.insert(path.begin() + i, mid_node);

//             corridors = generateSafeCorridor(path);

//             ensureCorridorsConnected(path, corridors);
//             break;
//         }
//     }
// }

// void Path_Planner::simplifyCorridors(std::vector<std::pair<Node, std::vector<int>>> &corridors)
// {
//     std::vector<std::pair<Node, std::vector<int>>> simplified_corridors;
//     simplified_corridors.reserve(corridors.size());
//     simplified_corridors.push_back(corridors[0]);

//     for (size_t i = 1; i < corridors.size(); ++i)
//     {
//         auto &prev_corridor = simplified_corridors.back().second;
//         auto &curr_corridor = corridors[i].second;

//         if (curr_corridor[0] >= prev_corridor[0] && curr_corridor[1] <= prev_corridor[1] &&
//             curr_corridor[2] >= prev_corridor[2] && curr_corridor[3] <= prev_corridor[3])
//         {
//             continue;
//         }

//         if (curr_corridor == prev_corridor)
//         {
//             continue;
//         }

//         curr_corridor[0] = std::max(curr_corridor[0], 0);
//         curr_corridor[1] = std::min(curr_corridor[1], static_cast<int>(map_data_.info.width - 1));
//         curr_corridor[2] = std::max(curr_corridor[2], 0);
//         curr_corridor[3] = std::min(curr_corridor[3], static_cast<int>(map_data_.info.height - 1));

//         simplified_corridors.push_back(corridors[i]);
//     }

//     corridors = simplified_corridors;
// }

// void Path_Planner::inflateObstacle(int goal_x, int goal_y, nav_msgs::OccupancyGrid &map)
// {
//     for (int dx = -2.0 * inflation_radius_; dx <= 2.0 * inflation_radius_; ++dx)
//     {
//         for (int dy = -2.0 * inflation_radius_; dy <= 2.0 * inflation_radius_; ++dy)
//         {
//             int x = goal_x + dx;
//             int y = goal_y + dy;
//             if (x >= 0 && x < map.info.width && y >= 0 && y < map.info.height)
//             {
//                 map.data[y * map.info.width + x] = 100;
//             }
//         }
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "path_planner_example");
//     ros::NodeHandle nh;

//     auto start_time = std::chrono::high_resolution_clock::now();

//     // 使用shared_ptr在堆中创建Path_Planner对象
//     std::shared_ptr<Path_Planner> path_planner = std::make_shared<Path_Planner>(nh);

//     while (ros::ok() && !path_planner->mapReceived())
//     {
//         ros::spinOnce();
//     }

//     path_planner->planPaths();

//     const auto &start_positions = path_planner->getStartPositions();
//     const auto &goal_positions = path_planner->getGoalPositions();

//     for (size_t i = 0; i < start_positions.size(); ++i)
//     {
//         std::pair<int, int> start = path_planner->getStart(i);
//         std::pair<int, int> goal = path_planner->getGoal(i);
//         // ROS_INFO("Robot %lu start and goal are get", i + 1);
//         const auto &corridors = path_planner->getCorridors(i);
//         // ROS_INFO("Robot %lu Corridor is get", i + 1);

//         // ROS_INFO("Robot %lu Start: (%d, %d), Goal: (%d, %d)", i + 1, start.first, start.second, goal.first, goal.second);
//         // for (const auto &corridor : corridors)
//         // {
//         //     ROS_INFO("Node (%d, %d): Corridor [%d, %d] -> [%d, %d]",
//         //              corridor.first.x, corridor.first.y,
//         //              corridor.second[0], corridor.second[2],  // min_x, min_y
//         //              corridor.second[1], corridor.second[3]); // max_x, max_y
//         // }

//     }

//     // 记录结束时间
//     auto end_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed_time = end_time - start_time;

//     // 输出执行时间
//     ROS_INFO("Execution time: %.6f seconds", elapsed_time.count());

//     return 0;
// }
























































// /*在生成了路径之后，做简化路径，并且在两个corridor不连通时采用采用先插值后局部路径规划的方法确保联通性*/
// /*但是会出现递归不停止的现象*/
// #include "Path_Planner.h"

// Path_Planner::Path_Planner(ros::NodeHandle &nh) : planner_(nh)
// {
//     double robot_radius;
//     if (!nh.getParam("robot_radius", robot_radius))
//     {
//         ROS_ERROR("Failed to get param 'robot_radius'");
//         robot_radius = 0.5;
//     }

//     map_sub_ = nh.subscribe("/inflated_map", 1, &Path_Planner::mapCallback, this);

//     ros::Rate rate(10);
//     while (ros::ok() && !map_received_)
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     if (map_received_)
//     {
//         double resolution = map_data_.info.resolution;
//         inflation_radius_ = static_cast<int>(std::ceil(robot_radius / resolution));
//         ROS_INFO("Calculated inflation_radius_: %d", inflation_radius_);
//     }
// }

// void Path_Planner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
// {
//     map_data_ = *msg;
//     original_map_ = map_data_;
//     map_received_ = true;
// }

// void Path_Planner::planPaths()
// {
//     if (!map_received_)
//     {
//         ROS_WARN("Map not received, cannot plan paths.");
//         return;
//     }

//     const auto &start_positions = planner_.getStartPositions();
//     const auto &goal_positions = planner_.getGoalPositions();
//     robot_corridors_.resize(start_positions.size());

//     for (size_t i = 0; i < start_positions.size(); ++i)
//     {
//         map_data_ = original_map_;

//         for (size_t j = 0; j < goal_positions.size(); ++j)
//         {
//             if (i != j)
//             {
//                 inflateObstacle(goal_positions[j].first, goal_positions[j].second, map_data_);
//             }
//         }

//         std::vector<Node> path = astar(start_positions[i].first, start_positions[i].second,
//                                        goal_positions[i].first, goal_positions[i].second);

//         if (!path.empty())
//         {
//             std::vector<std::pair<Node, std::vector<int>>> corridors = generateSafeCorridor(path);
//             ensureCorridorsConnected(path, corridors);
//             simplifyCorridors(corridors);
//             robot_corridors_[i] = corridors;
//         }
//         else
//         {
//             ROS_WARN("No path found for robot %lu.", i + 1);
//         }
//     }
// }

// const std::vector<std::pair<Node, std::vector<int>>> &Path_Planner::getCorridors(size_t robot_index) const
// {
//     return robot_corridors_[robot_index];
// }

// const std::vector<std::pair<int, int>> &Path_Planner::getStartPositions() const
// {
//     return planner_.getStartPositions();
// }

// const std::vector<std::pair<int, int>> &Path_Planner::getGoalPositions() const
// {
//     return planner_.getGoalPositions();
// }

// std::pair<int, int> Path_Planner::getStart(size_t robot_index) const
// {
//     return planner_.getStartPositions()[robot_index];
// }

// std::pair<int, int> Path_Planner::getGoal(size_t robot_index) const
// {
//     return planner_.getGoalPositions()[robot_index];
// }

// int Path_Planner::manhattanDistance(int x1, int y1, int x2, int y2)
// {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

// bool Path_Planner::isLineFree(int x1, int y1, int x2, int y2)
// {
//     int dx = abs(x2 - x1);
//     int dy = abs(y2 - y1);
//     int sx = (x1 < x2) ? 1 : -1;
//     int sy = (y1 < y2) ? 1 : -1;
//     int err = dx - dy;

//     while (true)
//     {
//         if (map_data_.data[y1 * map_data_.info.width + x1] != 0)
//         {
//             return false;
//         }
//         if (x1 == x2 && y1 == y2)
//         {
//             break;
//         }
//         int e2 = 2 * err;
//         if (e2 > -dy)
//         {
//             err -= dy;
//             x1 += sx;
//         }
//         if (e2 < dx)
//         {
//             err += dx;
//             y1 += sy;
//         }
//     }
//     return true;
// }

// std::vector<Node> Path_Planner::simplifyPath(const std::vector<Node> &path)
// {
//     if (path.size() < 3)
//     {
//         return path;
//     }

//     std::vector<Node> simplified_path;
//     simplified_path.reserve(path.size());
//     simplified_path.push_back(path.front());

//     for (size_t i = 2; i < path.size(); ++i)
//     {
//         if (!isLineFree(simplified_path.back().x, simplified_path.back().y, path[i].x, path[i].y))
//         {
//             simplified_path.push_back(path[i - 1]);
//         }
//     }

//     simplified_path.push_back(path.back());
//     // ROS_INFO("Path simplified from %lu to %lu nodes", path.size(), simplified_path.size());
//     return simplified_path;
// }

// std::vector<Node> Path_Planner::astar(int start_x, int start_y, int goal_x, int goal_y)
// {
//     std::priority_queue<Node *, std::vector<Node *>, CompareNode> open_list;
//     std::vector<std::vector<bool>> closed_list(map_data_.info.height, std::vector<bool>(map_data_.info.width, false));

//     Node *start_node = new Node(start_x, start_y, 0, manhattanDistance(start_x, start_y, goal_x, goal_y));
//     open_list.push(start_node);

//     while (!open_list.empty())
//     {
//         Node *current_node = open_list.top();
//         open_list.pop();

//         if (current_node->x == goal_x && current_node->y == goal_y)
//         {
//             std::vector<Node> path;
//             Node *path_node = current_node;
//             while (path_node)
//             {
//                 path.push_back(*path_node);
//                 path_node = path_node->parent;
//             }
//             std::reverse(path.begin(), path.end());
//             return simplifyPath(path);
//         }

//         closed_list[current_node->y][current_node->x] = true;

//         std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

//         for (auto &dir : directions)
//         {
//             int new_x = current_node->x + dir.first;
//             int new_y = current_node->y + dir.second;

//             if (new_x >= 0 && new_x < map_data_.info.width &&
//                 new_y >= 0 && new_y < map_data_.info.height &&
//                 map_data_.data[new_y * map_data_.info.width + new_x] == 0 &&
//                 !closed_list[new_y][new_x])
//             {

//                 int new_g = current_node->g + 1;
//                 int new_h = manhattanDistance(new_x, new_y, goal_x, goal_y);
//                 Node *new_node = new Node(new_x, new_y, new_g, new_h, current_node);
//                 open_list.push(new_node);
//             }
//         }
//     }

//     return std::vector<Node>(); // 返回空路径表示未找到路径
// }

// std::vector<std::pair<Node, std::vector<int>>> Path_Planner::generateSafeCorridor(const std::vector<Node> &path)
// {
//     std::vector<std::pair<Node, std::vector<int>>> safe_corridors;
//     safe_corridors.reserve(path.size());

//     for (const auto &node : path)
//     {
//         int min_x = node.x;
//         int max_x = node.x;
//         int min_y = node.y;
//         int max_y = node.y;

//         bool expand = true;
//         while (expand)
//         {
//             expand = false;

//             if (min_x > 0)
//             {
//                 bool obstacle_found = false;
//                 for (int y = min_y; y <= max_y; ++y)
//                 {
//                     if (map_data_.data[y * map_data_.info.width + (min_x - 1)] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     min_x--;
//                     expand = true;
//                 }
//             }

//             if (max_x < map_data_.info.width - 1)
//             {
//                 bool obstacle_found = false;
//                 for (int y = min_y; y <= max_y; ++y)
//                 {
//                     if (map_data_.data[y * map_data_.info.width + (max_x + 1)] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     max_x++;
//                     expand = true;
//                 }
//             }

//             if (min_y > 0)
//             {
//                 bool obstacle_found = false;
//                 for (int x = min_x; x <= max_x; ++x)
//                 {
//                     if (map_data_.data[(min_y - 1) * map_data_.info.width + x] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     min_y--;
//                     expand = true;
//                 }
//             }

//             if (max_y < map_data_.info.height - 1)
//             {
//                 bool obstacle_found = false;
//                 for (int x = min_x; x <= max_x; ++x)
//                 {
//                     if (map_data_.data[(max_y + 1) * map_data_.info.width + x] != 0)
//                     {
//                         obstacle_found = true;
//                         break;
//                     }
//                 }
//                 if (!obstacle_found)
//                 {
//                     max_y++;
//                     expand = true;
//                 }
//             }
//         }

//         min_x = std::max(min_x, 0);
//         max_x = std::min(max_x, static_cast<int>(map_data_.info.width - 1));
//         min_y = std::max(min_y, 0);
//         max_y = std::min(max_y, static_cast<int>(map_data_.info.height - 1));

//         safe_corridors.push_back({node, {min_x, max_x, min_y, max_y}});
//     }

//     return safe_corridors;
// }

// bool Path_Planner::areCorridorsConnected(const std::vector<int> &corridor1, const std::vector<int> &corridor2)
// {
//     return !(corridor1[1] < corridor2[0] || corridor2[1] < corridor1[0] || corridor1[3] < corridor2[2] || corridor2[3] < corridor1[2]);
// }

// bool Path_Planner::tryInsertMidpoints(std::vector<Node> &path, size_t start_idx, size_t end_idx, int depth)
// {
//     const int MAX_DEPTH = 5; // 设定递归深度限制

//     if (depth > MAX_DEPTH)
//     {
//         ROS_WARN("Max recursion depth reached in tryInsertMidpoints");
//         return false; // 达到最大深度，停止递归
//     }

//     // 创建一个副本以避免直接修改原始 path
//     std::vector<Node> temp_path = path;

//     // 计算中点
//     int mid_x = (temp_path[start_idx].x + temp_path[end_idx].x) / 2;
//     int mid_y = (temp_path[start_idx].y + temp_path[end_idx].y) / 2;
//     Node mid_node(mid_x, mid_y, 0, 0);

//     // 检查中点是否已经存在
//     for (const auto &node : temp_path)
//     {
//         if (node.x == mid_node.x && node.y == mid_node.y)
//         {
//             ROS_WARN("Same mid_point in tryInsertMidpoints");
//             return false; // 插入失败，节点重复
//         }
//     }

//     // 插入中点
//     temp_path.insert(temp_path.begin() + end_idx, mid_node);

//     // 生成新的 corridors 并检查连通性
//     std::vector<std::pair<Node, std::vector<int>>> temp_corridors = generateSafeCorridor(temp_path);
//     if (!areCorridorsConnected(temp_corridors[start_idx].second, temp_corridors[end_idx].second))
//     {
//         // 如果插入的中点无法成功连接 corridors，则继续递归尝试插入更多中点
//         return tryInsertMidpoints(temp_path, start_idx, end_idx, depth + 1);
//     }

//     // 如果成功连接，则更新原始路径
//     path = temp_path;
//     return true;
// }

// std::vector<Node> Path_Planner::findLocalPath(int start_x, int start_y, int goal_x, int goal_y)
// {
//     return astar(start_x, start_y, goal_x, goal_y);
// }

// void Path_Planner::ensureCorridorsConnected(std::vector<Node> &path, std::vector<std::pair<Node, std::vector<int>>> &corridors, int depth)
// {
//     const int MAX_DEPTH = 10; // 设定递归深度限制

//     if (depth > MAX_DEPTH)
//     {
//         ROS_WARN("Max recursion depth reached in ensureCorridorsConnected");
//         return; // 达到最大深度，停止递归
//     }

//     for (size_t i = 1; i < corridors.size(); ++i)
//     {
//         if (!areCorridorsConnected(corridors[i - 1].second, corridors[i].second))
//         {
//             // 如果corridors不连通，尝试插入中点
//             bool connected = tryInsertMidpoints(path, i - 1, i, depth);

//             if (!connected)
//             {
//                 // 如果插入中点仍未成功连通，使用局部路径搜索作为备用方案
//                 std::vector<Node> local_path = findLocalPath(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
//                 path.insert(path.begin() + i, local_path.begin(), local_path.end());

//                 // 重置递归深度
//                 // depth = 0;
//             }

//             // 重新生成整个 corridors
//             corridors = generateSafeCorridor(path);

//             // 递归调用以确保所有corridors连通
//             ensureCorridorsConnected(path, corridors, depth + 1);
//             break; // 跳出循环，因为path已经被更新，需要重新检查
//         }
//     }
// }

// void Path_Planner::simplifyCorridors(std::vector<std::pair<Node, std::vector<int>>> &corridors)
// {
//     std::vector<std::pair<Node, std::vector<int>>> simplified_corridors;
//     simplified_corridors.reserve(corridors.size());
//     simplified_corridors.push_back(corridors[0]);

//     for (size_t i = 1; i < corridors.size(); ++i)
//     {
//         auto &prev_corridor = simplified_corridors.back().second;
//         auto &curr_corridor = corridors[i].second;

//         if (curr_corridor[0] >= prev_corridor[0] && curr_corridor[1] <= prev_corridor[1] &&
//             curr_corridor[2] >= prev_corridor[2] && curr_corridor[3] <= prev_corridor[3])
//         {
//             continue;
//         }

//         if (curr_corridor == prev_corridor)
//         {
//             continue;
//         }

//         curr_corridor[0] = std::max(curr_corridor[0], 0);
//         curr_corridor[1] = std::min(curr_corridor[1], static_cast<int>(map_data_.info.width - 1));
//         curr_corridor[2] = std::max(curr_corridor[2], 0);
//         curr_corridor[3] = std::min(curr_corridor[3], static_cast<int>(map_data_.info.height - 1));

//         simplified_corridors.push_back(corridors[i]);
//     }

//     corridors = simplified_corridors;
// }

// void Path_Planner::inflateObstacle(int goal_x, int goal_y, nav_msgs::OccupancyGrid &map)
// {
//     for (int dx = -2.0 * inflation_radius_; dx <= 2.0 * inflation_radius_; ++dx)
//     {
//         for (int dy = -2.0 * inflation_radius_; dy <= 2.0 * inflation_radius_; ++dy)
//         {
//             int x = goal_x + dx;
//             int y = goal_y + dy;
//             if (x >= 0 && x < map.info.width && y >= 0 && y < map.info.height)
//             {
//                 map.data[y * map.info.width + x] = 100;
//             }
//         }
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "path_planner_example");
//     ros::NodeHandle nh;

//     auto start_time = std::chrono::high_resolution_clock::now();

//     std::shared_ptr<Path_Planner> path_planner = std::make_shared<Path_Planner>(nh);

//     while (ros::ok() && !path_planner->mapReceived())
//     {
//         ros::spinOnce();
//     }

//     path_planner->planPaths();

//     // 显示每个机器人的起点、终点以及简化后的 corridor
//     const auto &start_positions = path_planner->getStartPositions();
//     const auto &goal_positions = path_planner->getGoalPositions();

//     for (size_t i = 0; i < start_positions.size(); ++i)
//     {
//         std::pair<int, int> start = path_planner->getStart(i);
//         std::pair<int, int> goal = path_planner->getGoal(i);

//         // 打印机器人的起点和终点
//         ROS_INFO("Robot %lu Start: (%d, %d)", i + 1, start.first, start.second);
//         ROS_INFO("Robot %lu Goal: (%d, %d)", i + 1, goal.first, goal.second);

//         // 获取并打印简化后的 corridors
//         const auto &corridors = path_planner->getCorridors(i);

//         ROS_INFO("Robot %lu Simplified Corridors:", i + 1);
//         for (const auto &corridor : corridors)
//         {
//             const Node &node = corridor.first;
//             const std::vector<int> &bounds = corridor.second;

//             ROS_INFO("Node (%d, %d): Corridor bounds [%d, %d] -> [%d, %d]",
//                      node.x, node.y,
//                      bounds[0], bounds[2],  // min_x, min_y
//                      bounds[1], bounds[3]); // max_x, max_y
//         }
//     }

//     auto end_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed_time = end_time - start_time;

//     ROS_INFO("Execution time: %.6f seconds", elapsed_time.count());

//     return 0;
// }