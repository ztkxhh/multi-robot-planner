#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>

class RobotPathPlanner {
public:
    RobotPathPlanner() :  map_received(false) {
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

        // 获取发布的频率
        double publish_frequency;
        if (!nh.getParam("publish_frequency", publish_frequency)) {
            ROS_WARN("Failed to get publish_frequency, using default value 1.0 Hz");
            publish_frequency = 1.0; // 默认发布频率为1 Hz
        }

        // 订阅膨胀地图
        map_sub = nh.subscribe("inflated_map", 1, &RobotPathPlanner::mapCallback, this);

        // 初始化定时器，用于按频率发布起点和终点
        timer = nh.createTimer(ros::Duration(1.0 / publish_frequency), &RobotPathPlanner::timerCallback, this);

        // 初始化发布器，用于发布起点和终点
        start_pub = nh.advertise<visualization_msgs::Marker>("robot_start_points", 1);
        goal_pub = nh.advertise<visualization_msgs::Marker>("robot_goal_points", 1);

        ROS_INFO("RobotPathPlanner initialized.");
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    ros::Publisher start_pub;  // 用于发布起点消息
    ros::Publisher goal_pub;   // 用于发布终点消息
    ros::Timer timer;          // 定时器，用于按频率发布起点和终点
    int robot_count;
    double robot_radius;
    int width, height;
    double resolution;
    int radius_in_cells;
    bool map_received;

    nav_msgs::OccupancyGrid last_received_map;
    std::vector<std::pair<int, int>> start_positions;
    std::vector<std::pair<int, int>> goal_positions;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        if (map_received) {
            return;  // 已经接收到一次地图，不再处理后续地图消息
        }

        ROS_INFO("Map received, generating start and goal positions.");

        width = msg->info.width;
        height = msg->info.height;
        resolution = msg->info.resolution;
        last_received_map = *msg;

        // 计算机器人的半径在栅格中的表示
        radius_in_cells = std::ceil(robot_radius / resolution);

        generateStartAndGoalPositions();

        map_received = true;  // 标记地图已经接收
    }

    void generateStartAndGoalPositions() {
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
                    ROS_INFO("Valid start position for robot %d: (%d, %d)", i, start_x, start_y);
                } else {
                    ROS_WARN("Invalid start position for robot %d at (%d, %d)", i, start_x, start_y);
                }
            }

            // 找到有效的终点
            while (!valid_goal) {
                goal_x = dis_x(gen);
                goal_y = dis_y(gen);

                if (isValidPosition(goal_x, goal_y) && !isCollisionWithOtherRobots(goal_x, goal_y, false) &&
                    (goal_x != start_x || goal_y != start_y)) {
                    valid_goal = true;
                    goal_positions.push_back({goal_x, goal_y});
                    ROS_INFO("Valid goal position for robot %d: (%d, %d)", i, goal_x, goal_y);
                } else {
                    ROS_WARN("Invalid goal position for robot %d at (%d, %d)", i, goal_x, goal_y);
                }
            }
        }
    }

    bool isCollisionWithOtherRobots(int x, int y, bool checkStartPositions) {
        const auto& positions_to_check = checkStartPositions ? start_positions : goal_positions;

        for (const auto& pos : positions_to_check) {
            if (std::hypot(x - pos.first, y - pos.second) < 2 * radius_in_cells) {
                ROS_DEBUG("Position (%d, %d) is too close to another robot at (%d, %d)", x, y, pos.first, pos.second);
                return true;
            }
        }
        return false;
    }

    bool isValidPosition(int x, int y) {
        // 检查是否超出地图边界，考虑机器人半径
        if (x - radius_in_cells < 0 || x + radius_in_cells >= width || y - radius_in_cells < 0 || y + radius_in_cells >= height) {
            ROS_DEBUG("Position (%d, %d) out of bounds", x, y);
            return false;
        }

        // 检查是否与障碍物重合（地图已膨胀）
        if (last_received_map.data[x + y * width] > 50) {
            ROS_DEBUG("Position (%d, %d) is occupied by an obstacle", x, y);
            return false;
        }

        return true;
    }

    void timerCallback(const ros::TimerEvent&) {
        if (!map_received) {
            ROS_WARN("Map not received yet, skipping publish.");
            return;  // 地图未接收，不发布
        }

        // 定期发布起点和终点
        for (int i = 0; i < robot_count; ++i) {
            // 发布起点
            visualization_msgs::Marker start_marker;
            start_marker.header.frame_id = "map"; // 假设地图坐标系为 "map"
            start_marker.header.stamp = ros::Time::now();
            start_marker.ns = "robot_start_points";
            start_marker.id = i;
            start_marker.type = visualization_msgs::Marker::SPHERE;
            start_marker.action = visualization_msgs::Marker::ADD;
            start_marker.pose.position.x = start_positions[i].first * resolution;
            start_marker.pose.position.y = start_positions[i].second * resolution;
            start_marker.pose.position.z = 0.0;
            start_marker.pose.orientation.w = 1.0;
            start_marker.scale.x = 0.1;
            start_marker.scale.y = 0.1;
            start_marker.scale.z = 0.1;
            start_marker.color.r = 1.0f;
            start_marker.color.g = 0.0f;
            start_marker.color.b = 0.0f;
            start_marker.color.a = 1.0;
            start_pub.publish(start_marker);

            // 发布终点
            visualization_msgs::Marker goal_marker;
            goal_marker.header.frame_id = "map"; // 假设地图坐标系为 "map"
            goal_marker.header.stamp = ros::Time::now();
            goal_marker.ns = "robot_goal_points";
            goal_marker.id = i;
            goal_marker.type = visualization_msgs::Marker::SPHERE;
            goal_marker.action = visualization_msgs::Marker::ADD;
            goal_marker.pose.position.x = goal_positions[i].first * resolution;
            goal_marker.pose.position.y = goal_positions[i].second * resolution;
            goal_marker.pose.position.z = 0.0;
            goal_marker.pose.orientation.w = 1.0;
            goal_marker.scale.x = 0.1;
            goal_marker.scale.y = 0.1;
            goal_marker.scale.z = 0.1;
            goal_marker.color.r = 0.0f;
            goal_marker.color.g = 1.0f;
            goal_marker.color.b = 0.0f;
            goal_marker.color.a = 1.0;
            goal_pub.publish(goal_marker);

            ROS_INFO("Published start and goal for robot %d: Start (%f, %f), Goal (%f, %f)", 
                     i + 1, start_marker.pose.position.x, start_marker.pose.position.y, 
                     goal_marker.pose.position.x, goal_marker.pose.position.y);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_path_planner");

    RobotPathPlanner planner;

    ros::spin();

    return 0;
}