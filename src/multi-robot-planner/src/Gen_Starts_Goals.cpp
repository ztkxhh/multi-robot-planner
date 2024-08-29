#include "Gen_Starts_Goals.h"
#include <random>
#include <cmath>

Gen_Starts_Goals::Gen_Starts_Goals(ros::NodeHandle& nh) 
    : nh(nh), map_received(false) {

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

    generateStartAndGoalPositions();

    map_received = true;  // 标记地图已经接收
}

void Gen_Starts_Goals::generateStartAndGoalPositions() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_x(radius_in_cells, width - radius_in_cells - 1);
    std::uniform_int_distribution<> dis_y(radius_in_cells, height - radius_in_cells - 1);

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

            if (isValidPosition(start_x, start_y) && !isCollisionWithOtherRobots(start_x, start_y, true)) {
                valid_start = true;
                start_positions.push_back({start_x, start_y});
            }
        }

        // 找到有效的终点
        while (!valid_goal) {
            goal_x = dis_x(gen);
            goal_y = dis_y(gen);

            if (isValidPosition(goal_x, goal_y) &&
                !isCollisionWithOtherRobots(goal_x, goal_y, false) &&
                !isCollisionWithStartPositions(goal_x, goal_y) &&
                (goal_x != start_x || goal_y != start_y)) {
                valid_goal = true;
                goal_positions.push_back({goal_x, goal_y});
            }
        }
    }
}

bool Gen_Starts_Goals::isCollisionWithOtherRobots(int x, int y, bool checkStartPositions) {
    const auto& positions_to_check = checkStartPositions ? start_positions : goal_positions;

    for (const auto& pos : positions_to_check) {
        if (std::hypot(x - pos.first, y - pos.second) < 10 * radius_in_cells) {
            return true;
        }
    }
    return false;
}

bool Gen_Starts_Goals::isCollisionWithStartPositions(int x, int y) {
    for (const auto& start_pos : start_positions) {
        if (std::hypot(x - start_pos.first, y - start_pos.second) < 10 * radius_in_cells) {
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