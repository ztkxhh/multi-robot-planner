/*利用a*生成路径，不做简化，直接corridor，检查连通性。如果不连通报错，如果联通则进行简化*/
/*并且Node使用共享指针*/
/*采用两个膨胀地图分别进行路径规划和corridor生成*/
/*添加rviz的显示*/
#include "Path_Planner.h"
using namespace std;

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

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

    // Generate multi curve segments
    if (!GenerationCurves(nh))
    {
        ROS_ERROR("Failed to generate curves.");
    }
    else
    {
        Mergecurve();
    }
}

void Path_Planner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_data_ = *msg;
    original_map_ = map_data_;
    map_received_ = true;
}

void Path_Planner::doubleMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    double_map_data_ = *msg;
    double_map_received_ = true;
}

bool Path_Planner::planPaths()
{
    bool flag = true;
    if (!map_received_ || !double_map_received_)
    {
        ROS_WARN("Maps not fully received, cannot plan paths.");
        flag = false;
        return flag;
    }

    const std::vector<std::pair<int, int>> &start_positions = planner_.getStartPositions();
    const std::vector<std::pair<int, int>> &goal_positions = planner_.getGoalPositions();

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
            flag = false;
            continue;
        }

        std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> corridors = generateSafeCorridor(current_map2, path);
        bool connected = true;
        for (size_t k = 1; k < corridors.size(); ++k)
        {
            if (!areCorridorsConnected(corridors[k - 1].second, corridors[k].second))
            {
                connected = false;
                flag = false;
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
                    flag = false;
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
                    flag = false;
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
                    flag = false;
                    ROS_WARN("Path for Robot %lu is not connected after second simplify between nodes %lu and %lu", i + 1, k, k + 1);
                    break;
                }
            }
        }
        robot_corridors_[i] = corridors;
    }
    return flag;
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

std::vector<std::shared_ptr<Node>> Path_Planner::astar(const nav_msgs::OccupancyGrid &map, int start_x, int start_y, int goal_x, int goal_y, int max_steps)
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
        if (map.data[start_y * map.info.width + start_x] != 0)
        {
            ROS_WARN("Start is on an obstacle.");
        }
        if (map.data[goal_y * map.info.width + goal_x] != 0)
        {
            ROS_WARN("Goal is on an obstacle.");
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

        std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

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

std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> Path_Planner::generateSafeCorridor(const nav_msgs::OccupancyGrid &map, const std::vector<std::shared_ptr<Node>> &path)
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
    if (corridors.empty())
        return;

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

        curr_corridor[0] = std::max(curr_corridor[0], 0);
        curr_corridor[1] = std::min(curr_corridor[1], static_cast<int>(map_data_.info.width - 1));
        curr_corridor[2] = std::max(curr_corridor[2], 0);
        curr_corridor[3] = std::min(curr_corridor[3], static_cast<int>(map_data_.info.height - 1));

        simplified_corridors.emplace_back(corridors[i]);
    }

    corridors = simplified_corridors;
}

void Path_Planner::removeRedundantCorridors(std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> &corridors)
{
    if (corridors.size() <= 2)
    {
        return;
    }

    std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> further_simplified_corridors;
    further_simplified_corridors.reserve(corridors.size());
    further_simplified_corridors.push_back(corridors[0]);
    size_t i = 0;
    bool arr_end = false;
    while (i < corridors.size())
    {
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
                    further_simplified_corridors.push_back(corridors[j - 1]);
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
            further_simplified_corridors.push_back(corridors[j - 1]);
            i = j - 1;
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

void Path_Planner::publishPathVisualization(size_t robot_index, ros::Publisher &marker_pub, ros::Publisher &path_pub)
{
    visualization_msgs::MarkerArray marker_array;

    // 创建并发布起点Marker
    visualization_msgs::Marker start_marker;
    start_marker.header.frame_id = "map"; // 根据您的tf配置修改
    start_marker.header.stamp = ros::Time::now();
    start_marker.ns = "start_point";
    start_marker.id = 0;
    start_marker.type = visualization_msgs::Marker::SPHERE;
    start_marker.action = visualization_msgs::Marker::ADD;
    start_marker.pose.position.x = getStart(robot_index).first * map_data_.info.resolution;
    start_marker.pose.position.y = getStart(robot_index).second * map_data_.info.resolution;
    start_marker.pose.position.z = 0.5; // 提升高度以便在rviz中更容易看到
    start_marker.scale.x = 0.5;         // 设置点的大小
    start_marker.scale.y = 0.5;
    start_marker.scale.z = 0.5;
    start_marker.color.r = 1.0; // 颜色设置为红色
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
    goal_marker.pose.position.x = getGoal(robot_index).first * map_data_.info.resolution;
    goal_marker.pose.position.y = getGoal(robot_index).second * map_data_.info.resolution;
    start_marker.pose.position.z = 0.5; // 提升高度以便在rviz中更容易看到

    goal_marker.color.r = 0.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 1.0; // 颜色设置为蓝色

    // 初始化 orientation 为单位四元数
    goal_marker.pose.orientation.x = 0.0;
    goal_marker.pose.orientation.y = 0.0;
    goal_marker.pose.orientation.z = 0.0;
    goal_marker.pose.orientation.w = 1.0;

    marker_array.markers.push_back(goal_marker);

    // 创建并发布控制点Marker
    const auto &control_points = all_control_points[robot_index];

    for (size_t i = 0; i < control_points.size(); ++i)
    {
        for (size_t j = 0; j < control_points[i].size(); ++j)
        {
            visualization_msgs::Marker control_point_marker;
            control_point_marker.header.frame_id = "map";
            control_point_marker.header.stamp = ros::Time::now();
            control_point_marker.ns = "control_point";
            control_point_marker.id = i * control_points[i].size() + j + 2; // 起点和终点的ID为0和1
            control_point_marker.type = visualization_msgs::Marker::SPHERE;
            control_point_marker.action = visualization_msgs::Marker::ADD;
            control_point_marker.pose.position.x = control_points[i][j].first * map_data_.info.resolution;
            control_point_marker.pose.position.y = control_points[i][j].second * map_data_.info.resolution;
            control_point_marker.pose.position.z = 0.3;
            control_point_marker.scale.x = 0.5;
            control_point_marker.scale.y = 0.5;
            control_point_marker.scale.z = 0.5;

            // Change color every 6 points
            if ((control_point_marker.id - 2) % 6 == 0)
            {
                control_point_marker.color.r = 0.0;
                control_point_marker.color.g = 0.0;
                control_point_marker.color.b = 0.0;
                control_point_marker.color.a = 1.0;
            }
            else
            {
                control_point_marker.color.r = 0.0;
                control_point_marker.color.g = 1.0;
                control_point_marker.color.b = 0.0;
                control_point_marker.color.a = 1.0;
            }

            // 初始化 orientation 为单位四元数
            control_point_marker.pose.orientation.x = 0.0;
            control_point_marker.pose.orientation.y = 0.0;
            control_point_marker.pose.orientation.z = 0.0;
            control_point_marker.pose.orientation.w = 1.0;

            marker_array.markers.push_back(control_point_marker);
        }
    }

    // 创建并发布走廊Marker
    const auto &corridors = getCorridors(robot_index);
    // for (const auto &corridor : corridors)
    //     {
    //         ROS_INFO("Node (%d, %d): Corridor [%d, %d] -> [%d, %d]",
    //                  corridor.first->x, corridor.first->y, // node x, y
    //                  corridor.second[0],  corridor.second[1], // min_x, max_x
    //                  corridor.second[2], corridor.second[3]); // max_y, max_y
    //     }
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        visualization_msgs::Marker corridor_marker;
        corridor_marker.header.frame_id = "map";
        corridor_marker.header.stamp = ros::Time::now();
        corridor_marker.ns = "corridor";
        corridor_marker.id = i + 2; // 起点和终点的ID为0和1
        corridor_marker.type = visualization_msgs::Marker::CUBE;
        corridor_marker.action = visualization_msgs::Marker::ADD;
        corridor_marker.pose.position.x = (corridors[i].second[0] + corridors[i].second[1]) / 2.0 * map_data_.info.resolution;
        corridor_marker.pose.position.y = (corridors[i].second[2] + corridors[i].second[3]) / 2.0 * map_data_.info.resolution;
        corridor_marker.pose.position.z = 0.5;
        corridor_marker.scale.x = (corridors[i].second[1] - corridors[i].second[0]) * map_data_.info.resolution;
        corridor_marker.scale.y = (corridors[i].second[3] - corridors[i].second[2]) * map_data_.info.resolution;
        corridor_marker.scale.z = 0.1;
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

    // 发布其multiple_curves,即生成的曲线，nav_msgs::Path
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    int pose_num = 0;

    for (size_t i = 0; i < multiple_curves[robot_index].size(); i++)
    {
        pose_num += multiple_curves[robot_index][i]->_total + 1;
    }

    path.poses.resize(pose_num);

    int pose_idx = 0;
    for (size_t i = 0; i < multiple_curves[robot_index].size(); i++)
    {
        for (size_t j = 0; j <= multiple_curves[robot_index][i]->_total; j++)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = multiple_curves[robot_index][i]->_points[j].first * map_data_.info.resolution;
            pose.pose.position.y = multiple_curves[robot_index][i]->_points[j].second * map_data_.info.resolution;
            pose.pose.position.z = 1.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path.poses[pose_idx] = pose;

            pose_idx++;
        }
    }
    path_pub.publish(path);
}

int Path_Planner::MultiRobotTraGen(
    const std::vector<std::vector<std::vector<int>>> &corridors, // 每个corridor：std::vector<int>{min_x, max_x, min_y, max_y}
    const MatrixXd &MQM_jerk,
    const MatrixXd &MQM_length,
    double w_1, double w_2,
    const std::vector<std::pair<int, int>> &start_positions,
    const std::vector<std::pair<int, int>> &goal_positions,
    const int &curve_order)
{

    int n = corridors.size(); // 机器人数量

    vector<int> segments_nums(n);

    int n_poly = curve_order + 1; // 每段的控制点数量（6个）

    for (int i = 0; i < n; i++)
    {
        segments_nums[i] = corridors[i].size(); // 每个机器人的段数量
    }

    try
    {

        // Create an Gurobi environment
        GRBEnv env = GRBEnv(true);

        // 禁止打印输出信息
         env.set("OutputFlag", "0");

        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);

        // // // Set time limit and MIP gap
        // // model.getEnv().set(GRB_DoubleParam_TimeLimit, 60);
        // model.getEnv().set(GRB_DoubleParam_MIPGap, 0.000001);
        // model.getEnv().set(GRB_DoubleParam_MIPGapAbs, 0.000001);

        // model.getEnv().set(GRB_IntParam_Presolve, 0);

        model.getEnv().set(GRB_IntParam_Threads, 12);

        int total_cp_num = 0; // 总的控制点数量

        for (int i = 0; i < n; i++)
        {
            int robot_cp_num = segments_nums[i] * n_poly * 2; // 每个机器人有 m_i 段，每段有 n_poly 控制点，每个控制点有 x 和 y 两个维度
            total_cp_num += robot_cp_num;
        }

        // Create variables
        std::vector<GRBVar> vars(total_cp_num); // 控制点变量

        int var_idx = 0; // 变量索引
        // 设置控制点变量的上下界
        for (int i = 0; i < n; i++)
        {
            const auto &robot_corridors = corridors[i];
            for (int seg = 0; seg < segments_nums[i]; seg++)
            {
                const auto &corridor = robot_corridors[seg];
                for (int j = 0; j < n_poly; j++)
                {
                    // x 方向的控制点上下界
                    //检查x方向的边界是否相等
                    if (corridor[0] == corridor[1])
                    {
                        ROS_WARN("Robot %d, Segment %d, Control Point %d, x is same range: [%d, %d]", i, seg, j, corridor[0], corridor[1]);
                    }
                    vars[var_idx] = model.addVar(corridor[0], corridor[1], 0.0, GRB_CONTINUOUS);
                    // ROS_INFO("var_idx: %d, corridor[0]: %d, corridor[1]: %d", var_idx, corridor[0], corridor[1]);
                    var_idx++;

                    // y 方向的控制点上下界
                    //检查y方向的边界是否相等
                    if (corridor[2] == corridor[3])
                    {
                        ROS_WARN("Robot %d, Segment %d, Control Point %d, y is same range: [%d, %d]", i, seg, j, corridor[2], corridor[3]);
                    }
                    vars[var_idx] = model.addVar(corridor[2], corridor[3], 0.0, GRB_CONTINUOUS);
                    // ROS_INFO("var_idx: %d, corridor[2]: %d, corridor[3]: %d", var_idx, corridor[2], corridor[3]);
                    var_idx++;
                }
            }
        }

        // set cost funtion,遍历每个机器人，遍历矩阵的每个元素，设置目标函数
        GRBQuadExpr obj = 0.0;
        for (int i = 0; i < n; i++)
        { // 遍历每个机器人
            int offset = 0;
            for (int j = 0; j < i; j++)
            {
                offset += segments_nums[j] * n_poly * 2; // 索引偏移量，取决于之前机器人的所有控制点数量
            }

            for (int seg = 0; seg < segments_nums[i]; seg++)
            {                                             // 遍历每个段
                int base_idx = offset + seg * n_poly * 2; // 当前段的控制点起始索引

                for (int p = 0; p < n_poly; p++)
                { // 遍历当前段的所有控制点
                    for (int q = 0; q < n_poly; q++)
                    { // 遍历矩阵的所有元素
                        for (int dim = 0; dim < 2; dim++)
                        {                                       // x 和 y 方向
                            int var_p = base_idx + p * 2 + dim; // 变量 p 的索引
                            int var_q = base_idx + q * 2 + dim; // 变量 q 的索引

                            // obj += w_1 * MQM_jerk(p, q) * vars[var_p] * vars[var_q];
                            // obj += w_1 * MQM_length(p, q) * vars[var_p] * vars[var_q];

                            obj += w_1 * MQM_jerk(p, q) * vars[var_p] * vars[var_q] + w_2 * MQM_length(p, q) * vars[var_p] * vars[var_q];
                        }
                    }
                }
            }
        }

        model.setObjective(obj, GRB_MINIMIZE);

        // 添加起始位置和终止位置约束
        for (int i = 0; i < n; i++)
        {
            // 计算当前机器人的偏移量（即它的变量在整个优化变量中的起始位置）
            int robot_offset = 0;
            for (int k = 0; k < i; k++)
            {
                robot_offset += segments_nums[k] * n_poly * 2; // 计算当前机器人起始变量索引的偏移量
            }

            // 起始位置约束：设置第一个段的第一个控制点
            int start_idx_x = robot_offset;     // 第一个控制点的x坐标索引
            int start_idx_y = robot_offset + 1; // 第一个控制点的y坐标索引

            model.addConstr(vars[start_idx_x] == start_positions[i].first);
            model.addConstr(vars[start_idx_y] == start_positions[i].second);

            // 终止位置约束：设置最后一个段的最后一个控制点
            int last_seg_start_idx = robot_offset + (segments_nums[i] - 1) * n_poly * 2; // 计算最后一个段的起始变量索引
            int end_idx_x = last_seg_start_idx + (n_poly - 1) * 2;                       // 最后一个控制点的x坐标索引
            int end_idx_y = last_seg_start_idx + (n_poly - 1) * 2 + 1;                   // 最后一个控制点的y坐标索引

            model.addConstr(vars[end_idx_x] == goal_positions[i].first);
            model.addConstr(vars[end_idx_y] == goal_positions[i].second);
        }

        // 添加轨迹段间的连续性约束
        for (int i = 0; i < n; i++)
        {                   // 遍历每个机器人
            int offset = 0; // 记录当前机器人的控制点起始索引
            for (int j = 0; j < i; j++)
            {
                offset += segments_nums[j] * n_poly * 2; // 计算偏移量
            }

            for (int seg = 0; seg < segments_nums[i] - 1; seg++)
            {                                                        // 遍历每个段
                int base_idx_current = offset + seg * n_poly * 2;    // 当前段的起始索引
                int base_idx_next = offset + (seg + 1) * n_poly * 2; // 下一段的起始索引

                for (int dim = 0; dim < 2; dim++)
                { // x 和 y 方向
                    // 位置连续性：P_6^i = P_1^{i+1}
                    model.addConstr(vars[base_idx_current + 5 * 2 + dim] == vars[base_idx_next + dim]);

                    // 速度连续性：(P_6^i - P_5^i) * 5 = (P_2^{i+1} - P_1^{i+1}) * 5
                    model.addConstr(vars[base_idx_current + 5 * 2 + dim] - vars[base_idx_current + 4 * 2 + dim] == vars[base_idx_next + 1 * 2 + dim] - vars[base_idx_next + 0 * 2 + dim]);

                    // 加速度连续性：(P_6^i - 2P_5^i + P_4^i) * 20 = (P_3^{i+1} - 2P_2^{i+1} + P_1^{i+1}) * 20
                    model.addConstr(vars[base_idx_current + 5 * 2 + dim] - 2 * vars[base_idx_current + 4 * 2 + dim] + vars[base_idx_current + 3 * 2 + dim] == vars[base_idx_next + 2 * 2 + dim] - 2 * vars[base_idx_next + 1 * 2 + dim] + vars[base_idx_next + 0 * 2 + dim]);

                    // 加加速度连续性：(P_6^i - 3P_5^i + 3P_4^i - P_3^i) * 60 = (P_4^{i+1} - 3P_3^{i+1} + 3P_2^{i+1} - P_1^{i+1}) * 60
                    model.addConstr(vars[base_idx_current + 5 * 2 + dim] - 3 * vars[base_idx_current + 4 * 2 + dim] + 3 * vars[base_idx_current + 3 * 2 + dim] - vars[base_idx_current + 2 * 2 + dim] == vars[base_idx_next + 3 * 2 + dim] - 3 * vars[base_idx_next + 2 * 2 + dim] + 3 * vars[base_idx_next + 1 * 2 + dim] - vars[base_idx_next + 0 * 2 + dim]);
                }
            }
        }

        // 添加起始状态与终止状态的速度和加速度约束，即起始状态x,y的速度和加速度为0，终止状态的速度和加速度为0
        // 起始状态的速度和加速度约束分别为：P_2^i - P_1^i = 0, P_3^i - 2P_2^i + P_1^i = 0
        // 终止状态的速度和加速度约束分别为：P_6^i - P_5^i = 0, P_6^i - 2P_5^i + P_4^i = 0
        for (int i = 0; i < n; i++)
        {
            int offset = 0;
            for (int j = 0; j < i; j++)
            {
                offset += segments_nums[j] * n_poly * 2;
            }
            // x 方向
            int base_idx = offset;
            model.addConstr(vars[base_idx + 1 * 2] == vars[base_idx + 0 * 2]);
            model.addConstr(vars[base_idx + 2 * 2] == 2 * vars[base_idx + 1 * 2] - vars[base_idx + 0 * 2]);

            int last_seg_start_idx = offset + (segments_nums[i] - 1) * n_poly * 2;
            model.addConstr(vars[last_seg_start_idx + 5 * 2] == vars[last_seg_start_idx + 4 * 2]);
            model.addConstr(vars[last_seg_start_idx + 5 * 2] == 2 * vars[last_seg_start_idx + 4 * 2] - vars[last_seg_start_idx + 3 * 2]);

            // y 方向
            base_idx = offset + 1;
            model.addConstr(vars[base_idx + 1 * 2] == vars[base_idx + 0 * 2]);
            model.addConstr(vars[base_idx + 2 * 2] == 2 * vars[base_idx + 1 * 2] - vars[base_idx + 0 * 2]);

            last_seg_start_idx = offset + (segments_nums[i] - 1) * n_poly * 2 + 1;
            model.addConstr(vars[last_seg_start_idx + 5 * 2] == vars[last_seg_start_idx + 4 * 2]);
            model.addConstr(vars[last_seg_start_idx + 5 * 2] == 2 * vars[last_seg_start_idx + 4 * 2] - vars[last_seg_start_idx + 3 * 2]);
        }

        // 添加机器人之间的距离约束(只限于首段)
        // 对于每一对机器人，如果二者的起点相距小于3.0 * inflation_radius_，则计算它们的首段相同控制点（P_1^i - P_1^j, P_2^i - P_2^j, P_3^i - P_3^j,P_4^i - P_4^j,  P_5^i - P_5^j, P_6^i - P_6^j）之间的距离，并添加约束，使得这个距离的平方大于某个阈值
        double min_threshold = 4.0 * inflation_radius_ * inflation_radius_; // 距离的平方
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                if (std::hypot(start_positions[i].first - start_positions[j].first, start_positions[i].second - start_positions[j].second) <= 2.5 * inflation_radius_)
                {

                    ROS_INFO("Robot %d and Robot %d are too close !", i, j);

                    int offset_i = 0;
                    for (int k = 0; k < i; k++)
                    {
                        offset_i += segments_nums[k] * n_poly * 2;
                    }

                    int offset_j = 0;
                    for (int k = 0; k < j; k++)
                    {
                        offset_j += segments_nums[k] * n_poly * 2;
                    }

                    int base_idx_i = offset_i;
                    int base_idx_j = offset_j;

                    for (int p = 0; p < n_poly; p++)
                    {
                        int var_idx_i = base_idx_i + p * 2;
                        int var_idx_j = base_idx_j + p * 2;

                        model.addQConstr(vars[var_idx_i] * vars[var_idx_i] - 2 * vars[var_idx_i] * vars[var_idx_j] + vars[var_idx_j] * vars[var_idx_j] + vars[var_idx_i + 1] * vars[var_idx_i + 1] - 2 * vars[var_idx_i + 1] * vars[var_idx_j + 1] + vars[var_idx_j + 1] * vars[var_idx_j + 1] >= min_threshold);
                    }
                }
            }
        }

        // Optimize model
        model.optimize();

        // ROS_INFO("Model status: %d", model.get(GRB_IntAttr_Status));

        if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
            ROS_INFO( "The model is infeasible, calculating IIS");

            // 计算 IIS
            model.computeIIS();

            // 输出 IIS
            model.write("/home/zt/multi-robot-planner/src/multi-robot-planner/launch/model.ilp");
        }

        // Get the optimization result
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
        {
            // Get the optimal value of the objective function
            double obj_val = model.get(GRB_DoubleAttr_ObjVal);
            // ROS_INFO("Optimal objective: %.4f", obj_val);

            // Get the optimal value of the decision variables
            all_control_points.resize(n);
            for (int i = 0; i < n; i++)
            {
                all_control_points[i].resize(segments_nums[i]);
                int offset = 0;
                for (int j = 0; j < i; j++)
                {
                    offset += segments_nums[j] * n_poly * 2;
                }
                for (int seg = 0; seg < segments_nums[i]; seg++)
                {
                    all_control_points[i][seg].resize(n_poly);
                    for (int p = 0; p < n_poly; p++)
                    {
                        all_control_points[i][seg][p].first = vars[offset + seg * n_poly * 2 + p * 2].get(GRB_DoubleAttr_X);
                        all_control_points[i][seg][p].second = vars[offset + seg * n_poly * 2 + p * 2 + 1].get(GRB_DoubleAttr_X);
                    }
                }
            }

            return 1; // 成功
        }
    }

    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1; // 失败

    }

}

bool Path_Planner::GenerationControlPoints(ros::NodeHandle &nh)
{
    if (planPaths())
    {
        ROS_INFO("Success to plan initial path-points.");
    }
    else
    {
        ROS_ERROR("Failed to  plan initial path-points.");
        return false;
    }

    const std::vector<std::pair<int, int>> &start_positions = getStartPositions();
    const std::vector<std::pair<int, int>> &goal_positions = getGoalPositions();

    std::vector<std::vector<std::vector<int>>> allcorridors;
    allcorridors.resize(start_positions.size());
    for (size_t i = 0; i < start_positions.size(); ++i)
    {
        const auto &corridors = getCorridors(i);
        allcorridors[i].resize(corridors.size());
        for (size_t j = 0; j < corridors.size(); ++j)
        {
            allcorridors[i][j] = corridors[j].second;
        }
    }

    double minimum_order;
    int bezier_order;
    nh.param("/path_planning/minimum_order", minimum_order, 3.0); // 最小阶数
    nh.param("/path_planning/bezier_order", bezier_order, 5);     // 最小阶数

    int _poly_order_min = 3;
    int _poly_order_max = 10;
    Bernstein _bernstein;
    if (_bernstein.setParam(_poly_order_min, _poly_order_max, minimum_order) == -1)
    {
        ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set ");
    }
    vector<MatrixXd> MQM = _bernstein.getMQM(); // minimum jerk
    MatrixXd Q_jerk = MQM[bezier_order];

    // // 显示Q_jerk
    // for (int i = 0; i < Q_jerk.rows(); i++)
    // {
    //     for (int j = 0; j < Q_jerk.cols(); j++)
    //     {
    //         ROS_INFO("Q_jerk(%d, %d): %.2f", i, j, Q_jerk(i, j));
    //     }
    // }

    Bernstein _bernstein2;
    if (_bernstein2.setParam(_poly_order_min, _poly_order_max, 1.0) == -1)
    {
        ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set ");
    }
    vector<MatrixXd> MQM2 = _bernstein2.getMQM(); // minimum length
    MatrixXd Q_length = MQM2[bezier_order];

    // // 显示Q_length
    // for (int i = 0; i < Q_length.rows(); i++)
    // {
    //     for (int j = 0; j < Q_length.cols(); j++)
    //     {
    //         ROS_INFO("Q_length(%d, %d): %.2f", i, j, Q_length(i, j));
    //     }
    // }

    double w_1;
    double w_2;
    nh.param("/path_planning/w_1", w_1, 1.0); // 平滑项系数
    nh.param("/path_planning/w_2", w_2, 1.0); // 长度项系数

    if (MultiRobotTraGen(allcorridors, Q_jerk, Q_length, w_1, w_2, start_positions, goal_positions, bezier_order) == 1)
    {
        ROS_INFO("Success to optimize the path by getting control points.");

        // // // 显示优化得到的控制点
        // for (size_t i = 0; i < all_control_points.size(); ++i)
        // {
        //     ROS_INFO("Robot %lu", i + 1);
        //     for (size_t j = 0; j < all_control_points[i].size(); ++j)
        //     {
        //         ROS_INFO("Segment %lu", j + 1);
        //         for (size_t k = 0; k < all_control_points[i][j].size(); ++k)
        //         {
        //             ROS_INFO("Control Point %lu: (%.2f, %.2f)", k + 1, all_control_points[i][j][k].first, all_control_points[i][j][k].second);
        //         }
        //     }
        // }

        return true;
    }
    else
    {
        // ROS_ERROR("Failed to optimize control points.");
        return false;
    }
}

bool Path_Planner::GenerationCurves(ros::NodeHandle &nh)
{
    if (!GenerationControlPoints(nh))
    {
        ROS_ERROR("Failed to optimize the path by getting control points.");
        return false;
    }

    int points_num;
    nh.param("points_num", points_num, 100);

    int frequence;
    nh.param("frequence", frequence, 30);
    double T_s = 1.0 / frequence; // time for each segement(1/frequence)

    // 根据控制点初始化multiple_curves
    multiple_curves.resize(all_control_points.size());
    for (size_t i = 0; i < all_control_points.size(); ++i)
    {
        multiple_curves[i].resize(all_control_points[i].size());
    }

    // 生成每个segment的曲线，并存储到multiple_curves
    for (size_t i = 0; i < all_control_points.size(); ++i)
    {
        for (size_t j = 0; j < all_control_points[i].size(); ++j)
        {
            std::vector<std::pair<double, double>> control_points;
            for (size_t k = 0; k < all_control_points[i][j].size(); ++k)
            {
                control_points.emplace_back(all_control_points[i][j][k]);
            }

            multiple_curves[i][j] = std::make_shared<Beziercurve>(points_num);
            multiple_curves[i][j]->get_params(control_points, T_s);
        }
    }

    // 将multiple_curves中的曲线进行合并，并存储到成员变量merged_curves
    // 在合并曲线时，需要相邻曲线

    return true;
}



void Path_Planner::Mergecurve()
{

        merged_curves.resize(multiple_curves.size());

        for (size_t i = 0; i < multiple_curves.size(); ++i)
        {
            int total = 0;
            int seg_num = multiple_curves[i].size();
            for (size_t j = 0; j < seg_num; ++j)
            {
                total += multiple_curves[i][j]->_total;
            }
            total -= seg_num - 1;

            merged_curves[i] = std::make_shared<Beziercurve>(total);
            // 将multiple_curves中的曲线进行合并
            for (size_t k = 0; k < seg_num; ++k)
            {
                if (k == 0)
                {
                    for (size_t m = 0; m <= multiple_curves[i][k]->_total; ++m)
                    {
                        merged_curves[i]->_points.push_back(multiple_curves[i][k]->_points[m]);
                        merged_curves[i]->_v_s.push_back(multiple_curves[i][k]->_v_s[m]);
                        merged_curves[i]->_w_s.push_back(multiple_curves[i][k]->_w_s[m]);
                        merged_curves[i]->_a_ws.push_back(multiple_curves[i][k]->_a_ws[m]);
                        merged_curves[i]->_a_ts.push_back(multiple_curves[i][k]->_a_ts[m]);
                        merged_curves[i]->_a_rs.push_back(multiple_curves[i][k]->_a_rs[m]);
                        merged_curves[i]->_K_s.push_back(multiple_curves[i][k]->_K_s[m]);
                        merged_curves[i]->_arc_length.push_back(multiple_curves[i][k]->_arc_length[m]);
                        merged_curves[i]->_theta.push_back(multiple_curves[i][k]->_theta[m]);
                        merged_curves[i]->_x_fir.push_back(multiple_curves[i][k]->_x_fir[m]);
                        merged_curves[i]->_y_fir.push_back(multiple_curves[i][k]->_y_fir[m]);
                    }
                }
                else
                {
                    for (size_t m = 1; m <= multiple_curves[i][k]->_total; ++m)
                    {
                        merged_curves[i]->_points.push_back(multiple_curves[i][k]->_points[m]);
                        merged_curves[i]->_v_s.push_back(multiple_curves[i][k]->_v_s[m]);
                        merged_curves[i]->_w_s.push_back(multiple_curves[i][k]->_w_s[m]);
                        merged_curves[i]->_a_ws.push_back(multiple_curves[i][k]->_a_ws[m]);
                        merged_curves[i]->_a_ts.push_back(multiple_curves[i][k]->_a_ts[m]);
                        merged_curves[i]->_a_rs.push_back(multiple_curves[i][k]->_a_rs[m]);
                        merged_curves[i]->_K_s.push_back(multiple_curves[i][k]->_K_s[m]);
                        merged_curves[i]->_arc_length.push_back(multiple_curves[i][k]->_arc_length[m]);
                        merged_curves[i]->_theta.push_back(multiple_curves[i][k]->_theta[m]);
                        merged_curves[i]->_x_fir.push_back(multiple_curves[i][k]->_x_fir[m]);
                        merged_curves[i]->_y_fir.push_back(multiple_curves[i][k]->_y_fir[m]);

                    }
                }
            }
        }

}






void Path_Planner::plotting()
{
    // 通过画图的形式显示合并后的曲线特性

    // // path
    // for (size_t i = 0; i < merged_curves.size(); ++i)
    // {
    //     plt::figure(i);
    //     // 获取曲线的所有点
    //     const auto &points = merged_curves[i]->_points;
    //     // 提取起点和终点
    //     const auto &start = points.front();
    //     const auto &end = points.back();
    //     // 提取x和y坐标
    //     std::vector<double> x, y;
    //     for (const auto &point : points)
    //     {
    //         x.push_back(point.first);
    //         y.push_back(point.second);
    //     }
    //     // 绘制曲线
    //     plt::plot(x, y);
    //     // 标记起点和终点
    //     plt::plot({start.first}, {start.second}, "r*"); // 红色星表示起点
    //     plt::plot({end.first}, {end.second}, "k*");     // 黑色星表示终点
    //     plt::title("Robot " + std::to_string(i + 1) + " Trajectory");
    //     plt::xlabel("x");
    //     plt::ylabel("y"); 
    // }




    // // _v_s
    // for (size_t i = 0; i < merged_curves.size(); ++i)
    // {
    //     plt::figure(i);
    //     // 获取曲线的所有点
    //     const auto &v_s = merged_curves[i]->_v_s;
    //     // 提取起点和终点
    //     const auto &start = v_s.front();
    //     const auto &end = v_s.back();
    //     // 绘制曲线
    //     plt::plot(v_s);
    //     // 标记起点和终点
    //     plt::plot({0}, {start}, "r*"); // 红色星表示起点
    //     plt::plot({v_s.size() - 1}, {end}, "k*"); // 黑色星表示终点
    //     plt::title("Robot " + std::to_string(i + 1) + " v_s");
    //     plt::xlabel("x");
    //     plt::ylabel("y");
    // }



    // // _w_s
    // for (size_t i = 0; i < merged_curves.size(); ++i)
    // {
    //     plt::figure(i);
    //     // 获取曲线的所有点
    //     const auto &w_s = merged_curves[i]->_w_s;
    //     // 提取起点和终点
    //     const auto &start = w_s.front();
    //     const auto &end = w_s.back();
    //     // 绘制曲线
    //     plt::plot(w_s);
    //     // 标记起点和终点
    //     plt::plot({0}, {start}, "r*"); // 红色星表示起点
    //     plt::plot({w_s.size() - 1}, {end}, "k*"); // 黑色星表示终点

    //     plt::title("Robot " + std::to_string(i + 1) + " w_s");
    //     plt::xlabel("x");
    //     plt::ylabel("y");
    // }


    // // _a_ws
    // for (size_t i = 0; i < merged_curves.size(); ++i)
    // {
    //     plt::figure(i);
    //     // 获取曲线的所有点
    //     const auto &a_ws = merged_curves[i]->_a_ws;
    //     // 提取起点和终点
    //     const auto &start = a_ws.front();
    //     const auto &end = a_ws.back();
    //     // 绘制曲线
    //     plt::plot(a_ws);
    //     // 标记起点和终点
    //     plt::plot({0}, {start}, "r*"); // 红色星表示起点
    //     plt::plot({a_ws.size() - 1}, {end}, "k*"); // 黑色星表示终点

    //     plt::title("Robot " + std::to_string(i + 1) + " a_ws");
    //     plt::xlabel("x");
    //     plt::ylabel("y");
    // }

    // // _a_ts
    // for (size_t i = 0; i < merged_curves.size(); ++i)
    // {
    //     plt::figure(i);
    //     // 获取曲线的所有点
    //     const auto &a_ts = merged_curves[i]->_a_ts;
    //     // 提取起点和终点
    //     const auto &start = a_ts.front();
    //     const auto &end = a_ts.back();
    //     // 绘制曲线
    //     plt::plot(a_ts);
    //     // 标记起点和终点
    //     plt::plot({0}, {start}, "r*"); // 红色星表示起点
    //     plt::plot({a_ts.size() - 1}, {end}, "k*"); // 黑色星表示终点

    //     plt::title("Robot " + std::to_string(i + 1) + " a_ts");
    //     plt::xlabel("x");
    //     plt::ylabel("y");
    // }

    // _a_rs
    for (size_t i = 0; i < merged_curves.size(); ++i)
    {
        plt::figure(i);
        // 获取曲线的所有点
        const auto &a_rs = merged_curves[i]->_a_rs;
        // 提取起点和终点
        const auto &start = a_rs.front();
        const auto &end = a_rs.back();
        // 绘制曲线
        plt::plot(a_rs);
        // 标记起点和终点
        plt::plot({0}, {start}, "r*"); // 红色星表示起点
        plt::plot({a_rs.size() - 1}, {end}, "k*"); // 黑色星表示终点

        plt::title("Robot " + std::to_string(i + 1) + " a_rs");
        plt::xlabel("x");
        plt::ylabel("y");
    }

    plt::show();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    auto start_time = std::chrono::high_resolution_clock::now();

    std::shared_ptr<Path_Planner> path_planner = std::make_shared<Path_Planner>(nh);

    // 选择一个机器人，比如第一个机器人，发布其起止点以及走廊可视化
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    // 选择一个机器人，比如第一个机器人，发布其multiple_curves,即生成的曲线，nav_msgs::Path
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("visualization_path", 1);

    ros::Rate rate(10);
    while (ros::ok() && (!path_planner->mapReceived() || !path_planner->doubleMapReceived()))
    {
        ros::spinOnce();
        rate.sleep();
    }


    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;
    ROS_INFO("Execution time: %.6f seconds", elapsed_time.count());

    // 通过画图的形式显示合并后的曲线特性
    // path_planner->plotting();

    // 选择一个机器人，比如第一个机器人，发布其路径可视化
    while (ros::ok())
    {
        size_t robot_index = 0;
        path_planner->publishPathVisualization(robot_index, marker_pub, path_pub);
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