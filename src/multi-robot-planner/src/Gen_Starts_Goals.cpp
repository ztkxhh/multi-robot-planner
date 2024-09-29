#include "Gen_Starts_Goals.h"
#include <random>
#include <cmath>

std::chrono::high_resolution_clock::time_point start_time_gene_s_and_goal;
std::chrono::high_resolution_clock::time_point end_time_gene_s_and_goal;

std::chrono::duration<double>   elapsed_time_gene_s_and_goal;

Gen_Starts_Goals::Gen_Starts_Goals(ros::NodeHandle& nh) 
    : nh(nh), map_received(false) {

    start_time_gene_s_and_goal = std::chrono::high_resolution_clock::now();

    // 获取机器人数量
    if (!nh.getParam("robot_count", robot_count)) {
        ROS_WARN("Failed to get robot_count, using default value 1");
        robot_count = 1; // 默认一个机器人
    }

    // 获取每个机器人的半径
    if (!nh.getParam("robot_radius", robot_radius)) {
        ROS_WARN("Failed to get robot_radius, using default value 0.5");
        robot_radius = 0.5; // 默认半径0.5米
    }

    // 订阅膨胀地图
    map_sub = nh.subscribe("double_inflated_map", 1, &Gen_Starts_Goals::mapCallback, this);



}


bool Gen_Starts_Goals::checkStartsandGoals()
{
    // 检查起点和终点是否在地图范围内
    for (int i = 0; i < robot_count; ++i) {
        if (start_positions[i].first < 0 || start_positions[i].first >= width ||
            start_positions[i].second < 0 || start_positions[i].second >= height) {
            ROS_ERROR("Start position for robot %d is out of map bounds", i + 1);
            return false;
        }

        if (goal_positions[i].first < 0 || goal_positions[i].first >= width ||
            goal_positions[i].second < 0 || goal_positions[i].second >= height) {
            ROS_ERROR("Goal position for robot %d is out of map bounds", i + 1);
            return false;
        }
    }

    // 检查起点和终点是否在障碍物上
    for (int i = 0; i < robot_count; ++i) {
        if (last_received_map.data[start_positions[i].first + start_positions[i].second * width] != 0) {
            ROS_ERROR("Start position for robot %d is on an obstacle", i + 1);
            return false;
        }

        if (last_received_map.data[goal_positions[i].first + goal_positions[i].second * width] != 0) {
            ROS_ERROR("Goal position for robot %d is on an obstacle", i + 1);
            return false;
        }
    }

    // 检查起点和终点是否重合
    for (int i = 0; i < robot_count; ++i) {
        if (start_positions[i].first == goal_positions[i].first && start_positions[i].second == goal_positions[i].second) {
            ROS_ERROR("Start and goal positions for robot %d are the same", i + 1);
            return false;
        }
    }

    // 检查起点之间是否间隔大于2倍机器人半径
    for (int i = 0; i < robot_count; ++i) {
        for (int j = i + 1; j < robot_count; ++j) {
            if (std::hypot(start_positions[i].first - start_positions[j].first, start_positions[i].second - start_positions[j].second) <= 2 * radius_in_cells) {
                ROS_ERROR("Start positions for robots %d and %d are too close", i + 1, j + 1);
                return false;
            }
        }
    }

    // 检查终点之间是否间隔大于2倍机器人半径
    for (int i = 0; i < robot_count; ++i) {
        for (int j = i + 1; j < robot_count; ++j) {
            if (std::hypot(goal_positions[i].first - goal_positions[j].first, goal_positions[i].second - goal_positions[j].second) <= 2 * radius_in_cells) {
                ROS_ERROR("Goal positions for robots %d and %d are too close", i + 1, j + 1);
                return false;
            }
        }
    }

    // 检查不同机器人的起点和终点之间是否间隔大于2倍机器人半径
    for (int i = 0; i < robot_count; ++i) {
        for (int j = 0; j < robot_count; ++j) {
            if (i == j) {
                continue;
            }

            if (std::hypot(start_positions[i].first - goal_positions[j].first, start_positions[i].second - goal_positions[j].second) <= 2 * radius_in_cells) {
                ROS_ERROR("Start position for robot %d and goal position for robot %d are too close", i + 1, j + 1);
                return false;
            }
        }
    }
    return true;
}


void Gen_Starts_Goals::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (map_received) {
        return;  // 已经接收到一次地图，不再处理后续地图消息
    }

    width = msg->info.width;
    height = msg->info.height;
    resolution = msg->info.resolution;
    last_received_map = *msg;

    // 计算机器人的半径在栅格中的表示
    radius_in_cells = std::ceil(robot_radius / resolution);
    // ROS_INFO("Robot radius in cells: %d", radius_in_cells);

    generateStartAndGoalPositions();

    bool dasd = checkStartsandGoals();


    map_received = true;  // 标记地图已经接收
}

void Gen_Starts_Goals::generateStartAndGoalPositions() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_x(radius_in_cells, width - radius_in_cells - 1);
    std::uniform_int_distribution<> dis_y(radius_in_cells, height - radius_in_cells - 1);
    int attempts = 0;
    int max_attempts = 10000000;
    start_positions.clear();
    goal_positions.clear();

    for (int i = 0; i < robot_count; ++i) {
        bool valid_start = false;
        bool valid_goal = false;
        int start_x, start_y, goal_x, goal_y;

        // 找到有效的起点
        while (!valid_start) {
            start_x = dis_x(gen);
            start_y = dis_y(gen);

            if (isValidPosition(start_x, start_y) &&
                 !isCollisionWithOtherRobots(start_x, start_y, true)&&
                 !isCollisionWithOtherRobots(start_x, start_y, false)) {
                valid_start = true;
                start_positions.push_back({start_x, start_y});
            }
            attempts++;
            if (attempts > max_attempts) {
                ROS_ERROR("Failed to find valid start position after max_attempts ");
                return;
            }
        }
        attempts = 0;
        // 找到有效的终点
        while (!valid_goal) {
            goal_x = dis_x(gen);
            goal_y = dis_y(gen);

            if (isValidPosition(goal_x, goal_y) &&
                !isCollisionWithOtherRobots(goal_x, goal_y, false) &&
                !isCollisionWithOtherRobots(goal_x, goal_y, true) &&
                (goal_x != start_x || goal_y != start_y)) {
                valid_goal = true;
                goal_positions.push_back({goal_x, goal_y});
            }
            attempts++;
            if (attempts > max_attempts) {
                ROS_ERROR("Failed to find valid goal position after max_attempts ");
                return;
            }
        }
    }

    end_time_gene_s_and_goal = std::chrono::high_resolution_clock::now();


    elapsed_time_gene_s_and_goal = end_time_gene_s_and_goal - start_time_gene_s_and_goal;
    ROS_INFO("Execution time: %.6f seconds for generating starts and goals.", elapsed_time_gene_s_and_goal.count());
}

bool Gen_Starts_Goals::isCollisionWithOtherRobots(int x, int y, bool checkStartPositions) {
    const auto& positions_to_check = checkStartPositions ? start_positions : goal_positions;

    for (const auto& pos : positions_to_check) {
        if (std::hypot(x - pos.first, y - pos.second)  < 10 * radius_in_cells) {
            return true;
        }
    }
    return false;
}



bool Gen_Starts_Goals::isValidPosition(int x, int y) {
    // 检查是否超出地图边界，考虑机器人半径
    if (x - radius_in_cells < 0 || x + radius_in_cells >= width || y - radius_in_cells < 0 || y + radius_in_cells >= height) {
        return false;
    }

    // 检查是否与障碍物重合（地图已膨胀）
    if (last_received_map.data[x + y * width] !=0) {
        return false;
    }

    return true;
}

void Gen_Starts_Goals::outputStartAndGoalPositions() {
    ROS_INFO("Final start and goal positions for robots:");

    for (int i = 0; i < robot_count; ++i) {
        ROS_INFO("Robot %d: Start (%d, %d), Goal (%d, %d)", i + 1,
                 start_positions[i].first, start_positions[i].second,
                 goal_positions[i].first, goal_positions[i].second);
    }
}

const std::vector<std::pair<int, int>>& Gen_Starts_Goals::getStartPositions() const {
    return start_positions;
}

const std::vector<std::pair<int, int>>& Gen_Starts_Goals::getGoalPositions() const {
    return goal_positions;
}