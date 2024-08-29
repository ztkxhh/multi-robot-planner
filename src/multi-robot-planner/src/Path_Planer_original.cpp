/*最初始的A*代码*/
// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <visualization_msgs/Marker.h>
// #include <vector>
// #include <queue>
// #include <algorithm>

// // 定义A*节点结构
// struct Node {
//     int x, y;
//     int g, h;
//     Node* parent;

//     Node(int _x, int _y, int _g, int _h, Node* _parent = nullptr)
//         : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

//     int f() const { return g + h; }
// };

// // 比较函数，用于优先队列
// struct CompareNode {
//     bool operator()(Node* a, Node* b) {
//         return a->f() > b->f();
//     }
// };

// // 定义地图信息
// nav_msgs::OccupancyGrid map_data;
// bool map_received = false;

// void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
//     map_data = *msg;
//     map_received = true;
//     ROS_INFO("Map received.");
// }

// // 计算曼哈顿距离
// int manhattanDistance(int x1, int y1, int x2, int y2) {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

// // A*算法函数
// std::vector<Node> astar(int start_x, int start_y, int goal_x, int goal_y) {
//     std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
//     std::vector<std::vector<bool>> closed_list(map_data.info.height, std::vector<bool>(map_data.info.width, false));

//     Node* start_node = new Node(start_x, start_y, 0, manhattanDistance(start_x, start_y, goal_x, goal_y));
//     open_list.push(start_node);

//     while (!open_list.empty()) {
//         Node* current_node = open_list.top();
//         open_list.pop();

//         if (current_node->x == goal_x && current_node->y == goal_y) {
//             std::vector<Node> path;
//             Node* path_node = current_node;
//             while (path_node) {
//                 path.push_back(*path_node);
//                 path_node = path_node->parent;
//             }
//             std::reverse(path.begin(), path.end());
//             return path;
//         }

//         closed_list[current_node->y][current_node->x] = true;

//         std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
//         for (auto& dir : directions) {
//             int new_x = current_node->x + dir.first;
//             int new_y = current_node->y + dir.second;

//             if (new_x >= 0 && new_x < map_data.info.width &&
//                 new_y >= 0 && new_y < map_data.info.height &&
//                 map_data.data[new_y * map_data.info.width + new_x] == 0 &&
//                 !closed_list[new_y][new_x]) {

//                 int new_g = current_node->g + 1;
//                 int new_h = manhattanDistance(new_x, new_y, goal_x, goal_y);
//                 Node* new_node = new Node(new_x, new_y, new_g, new_h, current_node);
//                 open_list.push(new_node);
//             }
//         }
//     }

//     return std::vector<Node>(); // 返回空路径表示未找到路径
// }

// // 将路径转换为 nav_msgs::Path 并发布
// void publishPath(const std::vector<Node>& path, ros::Publisher& path_pub, ros::Publisher& marker_pub) {
//     nav_msgs::Path ros_path;
//     ros_path.header.stamp = ros::Time::now();
//     ros_path.header.frame_id = "map";

//     visualization_msgs::Marker start_marker;
//     visualization_msgs::Marker goal_marker;

//     start_marker.header.frame_id = goal_marker.header.frame_id = "map";
//     start_marker.header.stamp = goal_marker.header.stamp = ros::Time::now();
//     start_marker.ns = goal_marker.ns = "path_markers";
//     start_marker.action = goal_marker.action = visualization_msgs::Marker::ADD;
//     start_marker.pose.orientation.w = goal_marker.pose.orientation.w = 1.0;

//     start_marker.id = 0;
//     start_marker.type = visualization_msgs::Marker::SPHERE;
//     start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.2;
//     start_marker.color.r = 1.0f;
//     start_marker.color.g = 0.0f;
//     start_marker.color.b = 0.0f;
//     start_marker.color.a = 1.0;

//     goal_marker.id = 1;
//     goal_marker.type = visualization_msgs::Marker::SPHERE;
//     goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.2;
//     goal_marker.color.r = 0.0f;
//     goal_marker.color.g = 0.0f;
//     goal_marker.color.b = 1.0f;
//     goal_marker.color.a = 1.0;

//     for (size_t i = 0; i < path.size(); ++i) {
//         geometry_msgs::PoseStamped pose;
//         pose.header.stamp = ros::Time::now();
//         pose.header.frame_id = "map";
//         pose.pose.position.x = path[i].x * map_data.info.resolution + map_data.info.origin.position.x;
//         pose.pose.position.y = path[i].y * map_data.info.resolution + map_data.info.origin.position.y;
//         pose.pose.position.z = 0.0;
//         pose.pose.orientation.w = 1.0;

//         ros_path.poses.push_back(pose);

//         if (i == 0) {
//             start_marker.pose.position = pose.pose.position;
//         } else if (i == path.size() - 1) {
//             goal_marker.pose.position = pose.pose.position;
//         }
//     }

//     path_pub.publish(ros_path);
//     marker_pub.publish(start_marker);
//     marker_pub.publish(goal_marker);
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "astar_path_planning");
//     ros::NodeHandle nh;

//     ros::Subscriber map_sub = nh.subscribe("/inflated_map", 1, mapCallback);

//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);
//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("path_markers", 1);

//     ros::Rate rate(1.0); // 每秒发布一次路径

//     while (!map_received && ros::ok()) {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     int start_x = 10;  // 示例起点
//     int start_y = 10;
//     int goal_x = 480;   // 示例终点
//     int goal_y = 480;

//     while (ros::ok()) {
//         std::vector<Node> path = astar(start_x, start_y, goal_x, goal_y);

//         if (!path.empty()) {
//             ROS_INFO("Path found.");
//             publishPath(path, path_pub, marker_pub);
//         } else {
//             ROS_WARN("No path found.");
//         }

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }

// /*标记路径点的A*代码*/
// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Path.h>
// #include <visualization_msgs/Marker.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <vector>
// #include <queue>
// #include <algorithm>

// // 定义A*节点结构
// struct Node {
//     int x, y;
//     int g, h;
//     Node* parent;

//     Node(int _x, int _y, int _g, int _h, Node* _parent = nullptr)
//         : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

//     int f() const { return g + h; }
// };

// // 比较函数，用于优先队列
// struct CompareNode {
//     bool operator()(Node* a, Node* b) {
//         return a->f() > b->f();
//     }
// };

// // 定义地图信息
// nav_msgs::OccupancyGrid map_data;
// bool map_received = false;

// void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
//     map_data = *msg;
//     map_received = true;
//     ROS_INFO("Map received.");
// }

// // 计算曼哈顿距离
// int manhattanDistance(int x1, int y1, int x2, int y2) {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

// // A*算法函数
// std::vector<Node> astar(int start_x, int start_y, int goal_x, int goal_y) {
//     std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
//     std::vector<std::vector<bool>> closed_list(map_data.info.height, std::vector<bool>(map_data.info.width, false));

//     Node* start_node = new Node(start_x, start_y, 0, manhattanDistance(start_x, start_y, goal_x, goal_y));
//     open_list.push(start_node);

//     while (!open_list.empty()) {
//         Node* current_node = open_list.top();
//         open_list.pop();

//         if (current_node->x == goal_x && current_node->y == goal_y) {
//             std::vector<Node> path;
//             Node* path_node = current_node;
//             while (path_node) {
//                 path.push_back(*path_node);
//                 path_node = path_node->parent;
//             }
//             std::reverse(path.begin(), path.end());
//             return path;
//         }

//         closed_list[current_node->y][current_node->x] = true;

//         // std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
//         std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
//         for (auto& dir : directions) {
//             int new_x = current_node->x + dir.first;
//             int new_y = current_node->y + dir.second;

//             if (new_x >= 0 && new_x < map_data.info.width &&
//                 new_y >= 0 && new_y < map_data.info.height &&
//                 map_data.data[new_y * map_data.info.width + new_x] == 0 &&
//                 !closed_list[new_y][new_x]) {

//                 int new_g = current_node->g + 1;
//                 int new_h = manhattanDistance(new_x, new_y, goal_x, goal_y);
//                 Node* new_node = new Node(new_x, new_y, new_g, new_h, current_node);
//                 open_list.push(new_node);
//             }
//         }
//     }

//     return std::vector<Node>(); // 返回空路径表示未找到路径
// }

// // 将路径转换为 nav_msgs::Path 并发布，同时发布路径点标记
// void publishPathAndMarkers(const std::vector<Node>& path, ros::Publisher& path_pub, ros::Publisher& marker_pub) {
//     nav_msgs::Path ros_path;
//     ros_path.header.stamp = ros::Time::now();
//     ros_path.header.frame_id = "map";

//     int id = 0;
//     for (size_t i = 0; i < path.size(); ++i) {
//         geometry_msgs::PoseStamped pose;
//         pose.header.stamp = ros::Time::now();
//         pose.header.frame_id = "map";
//         pose.pose.position.x = path[i].x * map_data.info.resolution + map_data.info.origin.position.x;
//         pose.pose.position.y = path[i].y * map_data.info.resolution + map_data.info.origin.position.y;
//         pose.pose.position.z = 0.0;
//         pose.pose.orientation.w = 1.0;

//         ros_path.poses.push_back(pose);

//         // 创建Marker用于显示路径点
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "map";
//         marker.header.stamp = ros::Time::now();
//         marker.ns = "path_points";
//         marker.id = id++;
//         marker.type = visualization_msgs::Marker::SPHERE;
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.pose = pose.pose;
//         marker.scale.x = 0.1;
//         marker.scale.y = 0.1;
//         marker.scale.z = 0.1;

//         if (i == 0) {  // 起点
//             marker.color.r = 1.0;
//             marker.color.g = 0.0;
//             marker.color.b = 0.0;
//             marker.color.a = 1.0;
//         } else if (i == path.size() - 1) {  // 终点
//             marker.color.r = 0.0;
//             marker.color.g = 0.0;
//             marker.color.b = 1.0;
//             marker.color.a = 1.0;
//         } else {  // 其他路径点
//             marker.color.r = 0.0;
//             marker.color.g = 1.0;
//             marker.color.b = 0.0;
//             marker.color.a = 1.0;
//         }

//         marker_pub.publish(marker);
//     }

//     path_pub.publish(ros_path);
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "astar_path_planning");
//     ros::NodeHandle nh;

//     ros::Subscriber map_sub = nh.subscribe("/inflated_map", 1, mapCallback);

//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);
//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("path_markers", 1);

//     // 设置发布频率
//     ros::Rate rate(1.0); // 每秒发布一次路径

//     // 等待地图数据接收
//     while (!map_received && ros::ok()) {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     int start_x = 10;  // 示例起点
//     int start_y = 10;
//     int goal_x = 480;   // 示例终点
//     int goal_y = 480;

//     while (ros::ok()) {
//         std::vector<Node> path = astar(start_x, start_y, goal_x, goal_y);

//         if (!path.empty()) {
//             ROS_INFO("Path found.");
//             publishPathAndMarkers(path, path_pub, marker_pub);
//         } else {
//             ROS_WARN("No path found.");
//         }

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }

/*带有路径点简化的A*代码*/
// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <vector>
// #include <queue>
// #include <algorithm>
// #include <cmath>

// // 定义A*节点结构
// struct Node {
//     int x, y;
//     int g, h;
//     Node* parent;

//     Node(int _x, int _y, int _g, int _h, Node* _parent = nullptr)
//         : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

//     int f() const { return g + h; }
// };

// // 比较函数，用于优先队列
// struct CompareNode {
//     bool operator()(Node* a, Node* b) {
//         return a->f() > b->f();
//     }
// };

// // 定义地图信息
// nav_msgs::OccupancyGrid map_data;
// bool map_received = false;

// void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
//     map_data = *msg;
//     map_received = true;
//     ROS_INFO("Map received.");
// }

// // 计算曼哈顿距离
// int manhattanDistance(int x1, int y1, int x2, int y2) {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

// // 检查两个点之间的直线是否穿过障碍物
// bool isLineClear(int x1, int y1, int x2, int y2) {
//     int dx = abs(x2 - x1);
//     int dy = abs(y2 - y1);
//     int sx = (x1 < x2) ? 1 : -1;
//     int sy = (y1 < y2) ? 1 : -1;
//     int err = dx - dy;

//     while (true) {
//         int index = y1 * map_data.info.width + x1;
//         if (map_data.data[index] != 0) {
//             return false; // 碰到障碍物
//         }

//         if (x1 == x2 && y1 == y2) {
//             break;
//         }

//         int e2 = 2 * err;
//         if (e2 > -dy) {
//             err -= dy;
//             x1 += sx;
//         }
//         if (e2 < dx) {
//             err += dx;
//             y1 += sy;
//         }
//     }
//     return true; // 无障碍物
// }

// // 简化路径点
// std::vector<Node> simplifyPath(const std::vector<Node>& path) {
//     if (path.size() < 3) {
//         return path; // 无法简化
//     }

//     std::vector<Node> simplified_path;
//     simplified_path.push_back(path.front());

//     size_t i = 0;
//     for (size_t j = 2; j < path.size(); ++j) {
//         if (!isLineClear(path[i].x, path[i].y, path[j].x, path[j].y)) {
//             simplified_path.push_back(path[j - 1]);
//             i = j - 1;
//         }
//     }

//     simplified_path.push_back(path.back());
//     return simplified_path;
// }

// // A*算法函数
// std::vector<Node> astar(int start_x, int start_y, int goal_x, int goal_y) {
//     std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
//     std::vector<std::vector<bool>> closed_list(map_data.info.height, std::vector<bool>(map_data.info.width, false));

//     Node* start_node = new Node(start_x, start_y, 0, manhattanDistance(start_x, start_y, goal_x, goal_y));
//     open_list.push(start_node);

//     while (!open_list.empty()) {
//         Node* current_node = open_list.top();
//         open_list.pop();

//         if (current_node->x == goal_x && current_node->y == goal_y) {
//             std::vector<Node> path;
//             Node* path_node = current_node;
//             while (path_node) {
//                 path.push_back(*path_node);
//                 path_node = path_node->parent;
//             }
//             std::reverse(path.begin(), path.end());
//             return simplifyPath(path); // 返回简化后的路径
//         }

//         closed_list[current_node->y][current_node->x] = true;

//         std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
//         for (auto& dir : directions) {
//             int new_x = current_node->x + dir.first;
//             int new_y = current_node->y + dir.second;

//             if (new_x >= 0 && new_x < map_data.info.width &&
//                 new_y >= 0 && new_y < map_data.info.height &&
//                 map_data.data[new_y * map_data.info.width + new_x] == 0 &&
//                 !closed_list[new_y][new_x]) {

//                 int new_g = current_node->g + 1;
//                 int new_h = manhattanDistance(new_x, new_y, goal_x, goal_y);
//                 Node* new_node = new Node(new_x, new_y, new_g, new_h, current_node);
//                 open_list.push(new_node);
//             }
//         }
//     }

//     return std::vector<Node>(); // 返回空路径表示未找到路径
// }

// // 将路径转换为 nav_msgs::Path 并发布
// void publishPath(const std::vector<Node>& path, ros::Publisher& path_pub, ros::Publisher& marker_pub) {
//     nav_msgs::Path ros_path;
//     ros_path.header.stamp = ros::Time::now();
//     ros_path.header.frame_id = "map";

//     visualization_msgs::Marker start_marker;
//     visualization_msgs::Marker goal_marker;
//     visualization_msgs::MarkerArray path_markers;

//     start_marker.header.frame_id = goal_marker.header.frame_id = "map";
//     start_marker.header.stamp = goal_marker.header.stamp = ros::Time::now();
//     start_marker.ns = goal_marker.ns = "path_markers";
//     start_marker.action = goal_marker.action = visualization_msgs::Marker::ADD;
//     start_marker.pose.orientation.w = goal_marker.pose.orientation.w = 1.0;

//     start_marker.id = 0;
//     start_marker.type = visualization_msgs::Marker::SPHERE;
//     start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.2;
//     start_marker.color.r = 1.0f;
//     start_marker.color.g = 0.0f;
//     start_marker.color.b = 0.0f;
//     start_marker.color.a = 1.0;

//     goal_marker.id = 1;
//     goal_marker.type = visualization_msgs::Marker::SPHERE;
//     goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.2;
//     goal_marker.color.r = 0.0f;
//     goal_marker.color.g = 0.0f;
//     goal_marker.color.b = 1.0f;
//     goal_marker.color.a = 1.0;

//     for (size_t i = 0; i < path.size(); ++i) {
//         geometry_msgs::PoseStamped pose;
//         pose.header.stamp = ros::Time::now();
//         pose.header.frame_id = "map";
//         pose.pose.position.x = path[i].x * map_data.info.resolution + map_data.info.origin.position.x;
//         pose.pose.position.y = path[i].y * map_data.info.resolution + map_data.info.origin.position.y;
//         pose.pose.position.z = 0.0;
//         pose.pose.orientation.w = 1.0;

//         ros_path.poses.push_back(pose);

//         if (i == 0) {
//             start_marker.pose.position = pose.pose.position;
//         } else if (i == path.size() - 1) {
//             goal_marker.pose.position = pose.pose.position;
//         } else {
//             visualization_msgs::Marker marker;
//             marker.header.frame_id = "map";
//             marker.header.stamp = ros::Time::now();
//             marker.ns = "path_markers";
//             marker.id = i + 2;
//             marker.type = visualization_msgs::Marker::SPHERE;
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.pose.position = pose.pose.position;
//             marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
//             marker.color.r = 0.0f;
//             marker.color.g = 1.0f;
//             marker.color.b = 0.0f;
//             marker.color.a = 1.0;

//             path_markers.markers.push_back(marker);
//         }
//     }

//     path_pub.publish(ros_path);
//     marker_pub.publish(start_marker);
//     marker_pub.publish(goal_marker);
//     marker_pub.publish(path_markers);
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "astar_path_planning");
//     ros::NodeHandle nh;

//     ros::Subscriber map_sub = nh.subscribe("/inflated_map", 1, mapCallback);

//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);
//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("path_markers", 1);

//     ros::Rate rate(1.0); // 每秒发布一次路径

//     while (!map_received && ros::ok()) {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     int start_x = 10;  // 示例起点
//     int start_y = 10;
//     int goal_x = 400;   // 示例终点
//     int goal_y = 480;

//     while (ros::ok()) {
//         std::vector<Node> path = astar(start_x, start_y, goal_x, goal_y);

//         if (!path.empty()) {
//             ROS_INFO("Path found.");
//             publishPath(path, path_pub, marker_pub);
//         } else {
//             ROS_WARN("No path found.");
//         }

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }

// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <visualization_msgs/Marker.h>
// #include <vector>
// #include <queue>
// #include <algorithm>

// // 定义A*节点结构
// struct Node {
//     int x, y;
//     int g, h;
//     Node* parent;

//     Node(int _x, int _y, int _g, int _h, Node* _parent = nullptr)
//         : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

//     int f() const { return g + h; }
// };

// // 比较函数，用于优先队列
// struct CompareNode {
//     bool operator()(Node* a, Node* b) {
//         return a->f() > b->f();
//     }
// };

// // 定义地图信息
// nav_msgs::OccupancyGrid map_data;
// bool map_received = false;

// void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
//     map_data = *msg;
//     map_received = true;
//     ROS_INFO("Map received.");
// }

// // 计算曼哈顿距离
// int manhattanDistance(int x1, int y1, int x2, int y2) {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

// // 判断两点之间的直线是否穿过障碍物
// bool isLineClear(int x1, int y1, int x2, int y2) {
//     int dx = abs(x2 - x1);
//     int dy = abs(y2 - y1);
//     int sx = (x1 < x2) ? 1 : -1;
//     int sy = (y1 < y2) ? 1 : -1;
//     int err = dx - dy;

//     while (true) {
//         int index = y1 * map_data.info.width + x1;
//         if (map_data.data[index] != 0) {
//             return false;
//         }
//         if (x1 == x2 && y1 == y2) break;
//         int e2 = 2 * err;
//         if (e2 > -dy) {
//             err -= dy;
//             x1 += sx;
//         }
//         if (e2 < dx) {
//             err += dx;
//             y1 += sy;
//         }
//     }
//     return true;
// }

// // 简化路径，使得路径点数目最少
// std::vector<Node> simplifyPath(const std::vector<Node>& path) {
//     if (path.size() <= 2) return path;

//     std::vector<Node> simplified_path;
//     simplified_path.push_back(path.front());

//     for (size_t i = 1; i < path.size() - 1; ++i) {
//         if (!isLineClear(simplified_path.back().x, simplified_path.back().y, path[i+1].x, path[i+1].y)) {
//             simplified_path.push_back(path[i]);
//         }
//     }

//     simplified_path.push_back(path.back());
//     return simplified_path;
// }

// // A*算法函数
// std::vector<Node> astar(int start_x, int start_y, int goal_x, int goal_y) {
//     std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
//     std::vector<std::vector<bool>> closed_list(map_data.info.height, std::vector<bool>(map_data.info.width, false));

//     Node* start_node = new Node(start_x, start_y, 0, manhattanDistance(start_x, start_y, goal_x, goal_y));
//     open_list.push(start_node);

//     while (!open_list.empty()) {
//         Node* current_node = open_list.top();
//         open_list.pop();

//         if (current_node->x == goal_x && current_node->y == goal_y) {
//             std::vector<Node> path;
//             Node* path_node = current_node;
//             while (path_node) {
//                 path.push_back(*path_node);
//                 path_node = path_node->parent;
//             }
//             std::reverse(path.begin(), path.end());
//             return path;
//         }

//         closed_list[current_node->y][current_node->x] = true;

//         // std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
//         std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

//         for (auto& dir : directions) {
//             int new_x = current_node->x + dir.first;
//             int new_y = current_node->y + dir.second;

//             if (new_x >= 0 && new_x < map_data.info.width &&
//                 new_y >= 0 && new_y < map_data.info.height &&
//                 map_data.data[new_y * map_data.info.width + new_x] == 0 &&
//                 !closed_list[new_y][new_x]) {

//                 int new_g = current_node->g + 1;
//                 int new_h = manhattanDistance(new_x, new_y, goal_x, goal_y);
//                 Node* new_node = new Node(new_x, new_y, new_g, new_h, current_node);
//                 open_list.push(new_node);
//             }
//         }
//     }

//     return std::vector<Node>(); // 返回空路径表示未找到路径
// }

// // 将路径转换为 nav_msgs::Path 并发布，并标记路径点
// void publishPath(const std::vector<Node>& path, ros::Publisher& path_pub, ros::Publisher& marker_pub) {
//     nav_msgs::Path ros_path;
//     ros_path.header.stamp = ros::Time::now();
//     ros_path.header.frame_id = "map";

//     visualization_msgs::Marker start_marker;
//     visualization_msgs::Marker goal_marker;
//     visualization_msgs::Marker path_markers;

//     start_marker.header.frame_id = goal_marker.header.frame_id = path_markers.header.frame_id = "map";
//     start_marker.header.stamp = goal_marker.header.stamp = path_markers.header.stamp = ros::Time::now();
//     start_marker.ns = goal_marker.ns = path_markers.ns = "path_markers";
//     start_marker.action = goal_marker.action = path_markers.action = visualization_msgs::Marker::ADD;
//     start_marker.pose.orientation.w = goal_marker.pose.orientation.w = path_markers.pose.orientation.w = 1.0;

//     start_marker.id = 0;
//     start_marker.type = visualization_msgs::Marker::SPHERE;
//     start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.2;
//     start_marker.color.r = 1.0f;
//     start_marker.color.g = 0.0f;
//     start_marker.color.b = 0.0f;
//     start_marker.color.a = 1.0;

//     goal_marker.id = 1;
//     goal_marker.type = visualization_msgs::Marker::SPHERE;
//     goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.2;
//     goal_marker.color.r = 0.0f;
//     goal_marker.color.g = 0.0f;
//     goal_marker.color.b = 1.0f;
//     goal_marker.color.a = 1.0;

//     path_markers.id = 2;
//     path_markers.type = visualization_msgs::Marker::SPHERE_LIST;
//     path_markers.scale.x = path_markers.scale.y = path_markers.scale.z = 0.15;
//     path_markers.color.r = 0.0f;
//     path_markers.color.g = 1.0f;
//     path_markers.color.b = 0.0f;
//     path_markers.color.a = 1.0;

//     for (size_t i = 0; i < path.size(); ++i) {
//         geometry_msgs::PoseStamped pose;
//         pose.header.stamp = ros::Time::now();
//         pose.header.frame_id = "map";
//         pose.pose.position.x = path[i].x * map_data.info.resolution + map_data.info.origin.position.x;
//         pose.pose.position.y = path[i].y * map_data.info.resolution + map_data.info.origin.position.y;
//         pose.pose.position.z = 0.0;
//         pose.pose.orientation.w = 1.0;

//         ros_path.poses.push_back(pose);

//         geometry_msgs::Point p;
//         p.x = pose.pose.position.x;
//         p.y = pose.pose.position.y;
//         p.z = 0.0;

//         if (i == 0) {
//             start_marker.pose.position = pose.pose.position;
//         } else if (i == path.size() - 1) {
//             goal_marker.pose.position = pose.pose.position;
//         } else {
//             path_markers.points.push_back(p);
//         }
//     }

//     path_pub.publish(ros_path);
//     marker_pub.publish(start_marker);
//     marker_pub.publish(goal_marker);
//     marker_pub.publish(path_markers);
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "astar_path_planning");
//     ros::NodeHandle nh;

//     ros::Subscriber map_sub = nh.subscribe("/inflated_map", 1, mapCallback);

//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);
//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("path_markers", 1);

//     ros::Rate rate(1.0); // 每秒发布一次路径

//     while (!map_received && ros::ok()) {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     int start_x = 10;  // 示例起点
//     int start_y = 10;
//     int goal_x = 480;   // 示例终点
//     int goal_y = 480;

//     while (ros::ok()) {
//         std::vector<Node> path = astar(start_x, start_y, goal_x, goal_y);
//         std::vector<Node> simplified_path = simplifyPath(path);

//         if (!simplified_path.empty()) {
//             ROS_INFO("Path found and simplified.");
//             publishPath(simplified_path, path_pub, marker_pub);
//         } else {
//             ROS_WARN("No path found.");
//         }

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }

// /*带有路径点简化的A*算法，并且能够使用中点插值法生成联通的corridor，但是corridor没有简化*/
// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Path.h>
// #include <visualization_msgs/Marker.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <vector>
// #include <queue>
// #include <algorithm>

// // 定义A*节点结构
// struct Node {
//     int x, y;
//     int g, h;
//     Node* parent;

//     Node(int _x, int _y, int _g, int _h, Node* _parent = nullptr)
//         : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

//     int f() const { return g + h; }
// };

// // 比较函数，用于优先队列
// struct CompareNode {
//     bool operator()(Node* a, Node* b) {
//         return a->f() > b->f();
//     }
// };

// // 定义地图信息
// nav_msgs::OccupancyGrid map_data;
// bool map_received = false;

// void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
//     map_data = *msg;
//     map_received = true;
//     ROS_INFO("Map received.");
// }

// // 计算曼哈顿距离
// int manhattanDistance(int x1, int y1, int x2, int y2) {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

// // 检查两点之间是否有障碍物
// bool isLineFree(int x1, int y1, int x2, int y2) {
//     int dx = abs(x2 - x1);
//     int dy = abs(y2 - y1);
//     int sx = (x1 < x2) ? 1 : -1;
//     int sy = (y1 < y2) ? 1 : -1;
//     int err = dx - dy;

//     while (true) {
//         if (map_data.data[y1 * map_data.info.width + x1] != 0) {
//             return false;  // 如果线段经过障碍物，返回false
//         }
//         if (x1 == x2 && y1 == y2) {
//             break;  // 到达终点
//         }
//         int e2 = 2 * err;
//         if (e2 > -dy) {
//             err -= dy;
//             x1 += sx;
//         }
//         if (e2 < dx) {
//             err += dx;
//             y1 += sy;
//         }
//     }
//     return true;
// }

// // 简化路径
// std::vector<Node> simplifyPath(const std::vector<Node>& path) {
//     if (path.size() < 3) {
//         return path;  // 如果路径点少于3个，则无需简化
//     }

//     std::vector<Node> simplified_path;
//     simplified_path.push_back(path.front());

//     for (size_t i = 2; i < path.size(); ++i) {
//         if (!isLineFree(simplified_path.back().x, simplified_path.back().y, path[i].x, path[i].y)) {
//             simplified_path.push_back(path[i - 1]);
//         }
//     }

//     simplified_path.push_back(path.back());
//     return simplified_path;
// }

// // A*算法函数
// std::vector<Node> astar(int start_x, int start_y, int goal_x, int goal_y) {
//     std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
//     std::vector<std::vector<bool>> closed_list(map_data.info.height, std::vector<bool>(map_data.info.width, false));

//     Node* start_node = new Node(start_x, start_y, 0, manhattanDistance(start_x, start_y, goal_x, goal_y));
//     open_list.push(start_node);

//     while (!open_list.empty()) {
//         Node* current_node = open_list.top();
//         open_list.pop();

//         if (current_node->x == goal_x && current_node->y == goal_y) {
//             std::vector<Node> path;
//             Node* path_node = current_node;
//             while (path_node) {
//                 path.push_back(*path_node);
//                 path_node = path_node->parent;
//             }
//             std::reverse(path.begin(), path.end());
//             return simplifyPath(path);  // 返回简化后的路径
//         }

//         closed_list[current_node->y][current_node->x] = true;

//         std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
//         for (auto& dir : directions) {
//             int new_x = current_node->x + dir.first;
//             int new_y = current_node->y + dir.second;

//             if (new_x >= 0 && new_x < map_data.info.width &&
//                 new_y >= 0 && new_y < map_data.info.height &&
//                 map_data.data[new_y * map_data.info.width + new_x] == 0 &&
//                 !closed_list[new_y][new_x]) {

//                 int new_g = current_node->g + 1;
//                 int new_h = manhattanDistance(new_x, new_y, goal_x, goal_y);
//                 Node* new_node = new Node(new_x, new_y, new_g, new_h, current_node);
//                 open_list.push(new_node);
//             }
//         }
//     }

//     return std::vector<Node>(); // 返回空路径表示未找到路径
// }

// // 生成Safe Corridor
// std::vector<std::pair<Node, std::vector<int>>> generateSafeCorridor(const std::vector<Node>& path) {
//     std::vector<std::pair<Node, std::vector<int>>> safe_corridors;

//     for (const auto& node : path) {
//         int min_x = node.x;
//         int max_x = node.x;
//         int min_y = node.y;
//         int max_y = node.y;

//         // 向左扩展
//         while (min_x > 0) {
//             bool obstacle_found = false;
//             for (int y = min_y; y <= max_y; ++y) {
//                 if (map_data.data[y * map_data.info.width + (min_x - 1)] != 0) {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found) {
//                 break;
//             }
//             min_x--;
//         }

//         // 向右扩展
//         while (max_x < map_data.info.width - 1) {
//             bool obstacle_found = false;
//             for (int y = min_y; y <= max_y; ++y) {
//                 if (map_data.data[y * map_data.info.width + (max_x + 1)] != 0) {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found) {
//                 break;
//             }
//             max_x++;
//         }

//         // 向上扩展
//         while (min_y > 0) {
//             bool obstacle_found = false;
//             for (int x = min_x; x <= max_x; ++x) {
//                 if (map_data.data[(min_y - 1) * map_data.info.width + x] != 0) {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found) {
//                 break;
//             }
//             min_y--;
//         }

//         // 向下扩展
//         while (max_y < map_data.info.height - 1) {
//             bool obstacle_found = false;
//             for (int x = min_x; x <= max_x; ++x) {
//                 if (map_data.data[(max_y + 1) * map_data.info.width + x] != 0) {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found) {
//                 break;
//             }
//             max_y++;
//         }

//         safe_corridors.push_back({node, {min_x, max_x, min_y, max_y}});
//     }

//     return safe_corridors;
// }

// // 检查两个 corridor 是否连通
// bool areCorridorsConnected(const std::vector<int>& corridor1, const std::vector<int>& corridor2) {
//     // 检查两个 corridor 是否在 x 和 y 方向上有重叠部分
//     return !(corridor1[1] < corridor2[0] || corridor2[1] < corridor1[0] || corridor1[3] < corridor2[2] || corridor2[3] < corridor1[2]);
// }

// // 递归插值中点，确保 corridors 连通
// void ensureCorridorsConnected(std::vector<Node>& path, std::vector<std::pair<Node, std::vector<int>>>& corridors) {
//     for (size_t i = 1; i < corridors.size(); ++i) {
//         if (!areCorridorsConnected(corridors[i - 1].second, corridors[i].second)) {
//             // 找到两个不连通的corridor，计算中点并插入
//             int mid_x = (path[i - 1].x + path[i].x) / 2;
//             int mid_y = (path[i - 1].y + path[i].y) / 2;
//             Node mid_node(mid_x, mid_y, 0, 0);

//             path.insert(path.begin() + i, mid_node);
//             corridors = generateSafeCorridor(path);

//             // 递归调用，检查插入后的corridor是否连通
//             ensureCorridorsConnected(path, corridors);
//             break;  // 递归处理后重新检查
//         }
//     }
// }

// // 将路径和Safe Corridor标记发布，并打印边界
// void publishPathAndCorridors(const std::vector<Node>& path, const std::vector<std::pair<Node, std::vector<int>>>& corridors, ros::Publisher& path_pub, ros::Publisher& marker_pub) {
//     nav_msgs::Path ros_path;
//     ros_path.header.stamp = ros::Time::now();
//     ros_path.header.frame_id = "map";

//     int id = 0;
//     for (size_t i = 0; i < path.size(); ++i) {
//         geometry_msgs::PoseStamped pose;
//         pose.header.stamp = ros::Time::now();
//         pose.header.frame_id = "map";
//         pose.pose.position.x = path[i].x * map_data.info.resolution + map_data.info.origin.position.x;
//         pose.pose.position.y = path[i].y * map_data.info.resolution + map_data.info.origin.position.y;
//         pose.pose.position.z = 0.0;
//         pose.pose.orientation.w = 1.0;

//         ros_path.poses.push_back(pose);

//         // 打印corridor边界
//         int min_x = corridors[i].second[0];
//         int max_x = corridors[i].second[1];
//         int min_y = corridors[i].second[2];
//         int max_y = corridors[i].second[3];

//         ROS_INFO("Corridor for path point (%d, %d): min_x = %d, max_x = %d, min_y = %d, max_y = %d",
//                  path[i].x, path[i].y, min_x, max_x, min_y, max_y);

//         // 创建Marker用于显示路径点
//         visualization_msgs::Marker point_marker;
//         point_marker.header.frame_id = "map";
//         point_marker.header.stamp = ros::Time::now();
//         point_marker.ns = "path_points";
//         point_marker.id = id++;
//         point_marker.type = visualization_msgs::Marker::SPHERE;
//         point_marker.action = visualization_msgs::Marker::ADD;
//         point_marker.pose = pose.pose;
//         point_marker.scale.x = 0.1;
//         point_marker.scale.y = 0.1;
//         point_marker.scale.z = 0.1;

//         if (i == 0) {  // 起点
//             point_marker.color.r = 1.0;
//             point_marker.color.g = 0.0;
//             point_marker.color.b = 0.0;
//             point_marker.color.a = 1.0;
//         } else if (i == path.size() - 1) {  // 终点
//             point_marker.color.r = 0.0;
//             point_marker.color.g = 0.0;
//             point_marker.color.b = 1.0;
//             point_marker.color.a = 1.0;
//         } else {  // 其他路径点
//             point_marker.color.r = 0.0;
//             point_marker.color.g = 1.0;
//             point_marker.color.b = 0.0;
//             point_marker.color.a = 1.0;
//         }

//         marker_pub.publish(point_marker);

//         // 创建Marker用于显示Safe Corridor
//         visualization_msgs::Marker corridor_marker;
//         corridor_marker.header.frame_id = "map";
//         corridor_marker.header.stamp = ros::Time::now();
//         corridor_marker.ns = "safe_corridor";
//         corridor_marker.id = id++;
//         corridor_marker.type = visualization_msgs::Marker::CUBE;
//         corridor_marker.action = visualization_msgs::Marker::ADD;

//         corridor_marker.pose.position.x = (min_x + max_x) / 2.0 * map_data.info.resolution + map_data.info.origin.position.x;
//         corridor_marker.pose.position.y = (min_y + max_y) / 2.0 * map_data.info.resolution + map_data.info.origin.position.y;
//         corridor_marker.pose.position.z = 0.0;
//         corridor_marker.pose.orientation.w = 1.0;

//         corridor_marker.scale.x = (max_x - min_x + 1) * map_data.info.resolution;
//         corridor_marker.scale.y = (max_y - min_y + 1) * map_data.info.resolution;
//         corridor_marker.scale.z = 0.1; // 厚度可以根据需要调整

//         corridor_marker.color.r = 0.0;
//         corridor_marker.color.g = 1.0;
//         corridor_marker.color.b = 1.0;
//         corridor_marker.color.a = 0.5; // 半透明

//         marker_pub.publish(corridor_marker);
//     }

//     path_pub.publish(ros_path);
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "astar_path_planning");
//     ros::NodeHandle nh;

//     ros::Subscriber map_sub = nh.subscribe("/inflated_map", 1, mapCallback);

//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);
//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("path_markers", 1);

//     // 设置发布频率
//     ros::Rate rate(1.0); // 每秒发布一次路径

//     // 等待地图数据接收
//     while (!map_received && ros::ok()) {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     int start_x = 10;  // 示例起点
//     int start_y = 10;
//     int goal_x = 480;   // 示例终点
//     int goal_y = 480;

//     while (ros::ok()) {
//         std::vector<Node> path = astar(start_x, start_y, goal_x, goal_y);

//         if (!path.empty()) {
//             ROS_INFO("Path found.");
//             std::vector<std::pair<Node, std::vector<int>>> corridors = generateSafeCorridor(path);
//             ensureCorridorsConnected(path, corridors);  // 确保 corridor 连通
//             publishPathAndCorridors(path, corridors, path_pub, marker_pub);
//         } else {
//             ROS_WARN("No path found.");
//         }

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }









































































































// /*带有路径点简化的A*算法，并且能够使用中点插值法生成联通的corridor，corridor经过了简化*/
// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Path.h>
// #include <visualization_msgs/Marker.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <vector>
// #include <queue>
// #include <algorithm>

// // 定义A*节点结构
// struct Node
// {
//     int x, y;
//     int g, h;
//     Node *parent;

//     Node(int _x, int _y, int _g, int _h, Node *_parent = nullptr)
//         : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

//     int f() const { return g + h; }
// };

// // 比较函数，用于优先队列
// struct CompareNode
// {
//     bool operator()(Node *a, Node *b)
//     {
//         return a->f() > b->f();
//     }
// };

// // 定义地图信息
// nav_msgs::OccupancyGrid map_data;
// bool map_received = false;

// void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
// {
//     map_data = *msg;
//     map_received = true;
//     ROS_INFO("Map received.");
// }

// // 计算曼哈顿距离
// int manhattanDistance(int x1, int y1, int x2, int y2)
// {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

// // 检查两点之间是否有障碍物
// bool isLineFree(int x1, int y1, int x2, int y2)
// {
//     int dx = abs(x2 - x1);
//     int dy = abs(y2 - y1);
//     int sx = (x1 < x2) ? 1 : -1;
//     int sy = (y1 < y2) ? 1 : -1;
//     int err = dx - dy;

//     while (true)
//     {
//         if (map_data.data[y1 * map_data.info.width + x1] != 0)
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

// // 简化路径
// std::vector<Node> simplifyPath(const std::vector<Node> &path)
// {
//     if (path.size() < 3)
//     {
//         return path; // 如果路径点少于3个，则无需简化
//     }

//     std::vector<Node> simplified_path;
//     simplified_path.push_back(path.front());

//     for (size_t i = 2; i < path.size(); ++i)
//     {
//         if (!isLineFree(simplified_path.back().x, simplified_path.back().y, path[i].x, path[i].y))
//         {
//             simplified_path.push_back(path[i - 1]);
//         }
//     }

//     simplified_path.push_back(path.back());
//     return simplified_path;
// }

// // A*算法函数
// std::vector<Node> astar(int start_x, int start_y, int goal_x, int goal_y)
// {
//     std::priority_queue<Node *, std::vector<Node *>, CompareNode> open_list;
//     std::vector<std::vector<bool>> closed_list(map_data.info.height, std::vector<bool>(map_data.info.width, false));

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
//         for (auto &dir : directions)
//         {
//             int new_x = current_node->x + dir.first;
//             int new_y = current_node->y + dir.second;

//             if (new_x >= 0 && new_x < map_data.info.width &&
//                 new_y >= 0 && new_y < map_data.info.height &&
//                 map_data.data[new_y * map_data.info.width + new_x] == 0 &&
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

// // 生成Safe Corridor
// std::vector<std::pair<Node, std::vector<int>>> generateSafeCorridor(const std::vector<Node> &path)
// {
//     std::vector<std::pair<Node, std::vector<int>>> safe_corridors;

//     for (const auto &node : path)
//     {
//         int min_x = node.x;
//         int max_x = node.x;
//         int min_y = node.y;
//         int max_y = node.y;

//         // 向左扩展
//         while (min_x > 0)
//         {
//             bool obstacle_found = false;
//             for (int y = min_y; y <= max_y; ++y)
//             {
//                 if (map_data.data[y * map_data.info.width + (min_x - 1)] != 0)
//                 {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found)
//             {
//                 break;
//             }
//             min_x--;
//         }

//         // 向右扩展
//         while (max_x < map_data.info.width - 1)
//         {
//             bool obstacle_found = false;
//             for (int y = min_y; y <= max_y; ++y)
//             {
//                 if (map_data.data[y * map_data.info.width + (max_x + 1)] != 0)
//                 {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found)
//             {
//                 break;
//             }
//             max_x++;
//         }

//         // 向上扩展
//         while (min_y > 0)
//         {
//             bool obstacle_found = false;
//             for (int x = min_x; x <= max_x; ++x)
//             {
//                 if (map_data.data[(min_y - 1) * map_data.info.width + x] != 0)
//                 {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found)
//             {
//                 break;
//             }
//             min_y--;
//         }

//         // 向下扩展
//         while (max_y < map_data.info.height - 1)
//         {
//             bool obstacle_found = false;
//             for (int x = min_x; x <= max_x; ++x)
//             {
//                 if (map_data.data[(max_y + 1) * map_data.info.width + x] != 0)
//                 {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found)
//             {
//                 break;
//             }
//             max_y++;
//         }

//         // 再次检查所有方向的边界，确保安全
//         min_x = std::max(min_x, 0);
//         max_x = std::min(max_x, static_cast<int>(map_data.info.width - 1));
//         min_y = std::max(min_y, 0);
//         max_y = std::min(max_y, static_cast<int>(map_data.info.height - 1));

//         safe_corridors.push_back({node, {min_x, max_x, min_y, max_y}});
//     }

//     return safe_corridors;
// }

// // 检查两个 corridor 是否连通
// bool areCorridorsConnected(const std::vector<int> &corridor1, const std::vector<int> &corridor2)
// {
//     // 检查两个 corridor 是否在 x 和 y 方向上有重叠部分
//     return !(corridor1[1] < corridor2[0] || corridor2[1] < corridor1[0] || corridor1[3] < corridor2[2] || corridor2[3] < corridor1[2]);
// }

// // 递归插值中点，确保 corridors 连通
// void ensureCorridorsConnected(std::vector<Node> &path, std::vector<std::pair<Node, std::vector<int>>> &corridors)
// {
//     for (size_t i = 1; i < corridors.size(); ++i)
//     {
//         if (!areCorridorsConnected(corridors[i - 1].second, corridors[i].second))
//         {
//             // 找到两个不连通的corridor，计算中点并插入
//             int mid_x = (path[i - 1].x + path[i].x) / 2;
//             int mid_y = (path[i - 1].y + path[i].y) / 2;
//             Node mid_node(mid_x, mid_y, 0, 0);

//             path.insert(path.begin() + i, mid_node);
//             corridors = generateSafeCorridor(path);

//             // 递归调用，检查插入后的corridor是否连通
//             ensureCorridorsConnected(path, corridors);
//             break; // 递归处理后重新检查
//         }
//     }
// }

// // 简化corridors
// void simplifyCorridors(std::vector<std::pair<Node, std::vector<int>>> &corridors)
// {
//     std::vector<std::pair<Node, std::vector<int>>> simplified_corridors;
//     simplified_corridors.push_back(corridors[0]);

//     for (size_t i = 1; i < corridors.size(); ++i)
//     {
//         auto &prev_corridor = simplified_corridors.back().second;
//         auto &curr_corridor = corridors[i].second;

//         // 如果当前corridor完全包含在上一个corridor中
//         if (curr_corridor[0] >= prev_corridor[0] && curr_corridor[1] <= prev_corridor[1] &&
//             curr_corridor[2] >= prev_corridor[2] && curr_corridor[3] <= prev_corridor[3])
//         {
//             continue; // 忽略当前corridor，保留上一个corridor
//         }

//         // 如果两个corridor相同
//         if (curr_corridor == prev_corridor)
//         {
//             continue; // 忽略当前corridor
//         }

//         // 检查并修正corridor边界是否超过地图边界
//         curr_corridor[0] = std::max(curr_corridor[0], 0);
//         curr_corridor[1] = std::min(curr_corridor[1], static_cast<int>(map_data.info.width - 1));
//         curr_corridor[2] = std::max(curr_corridor[2], 0);
//         curr_corridor[3] = std::min(curr_corridor[3], static_cast<int>(map_data.info.height - 1));

//         simplified_corridors.push_back(corridors[i]);
//     }

//     // 打印简化后的corridors
//     ROS_INFO("Simplified Corridors:");
//     for (size_t i = 0; i < simplified_corridors.size(); ++i)
//     {
//         auto &corridor = simplified_corridors[i].second;
//         ROS_INFO("Corridor %lu: min_x = %d, max_x = %d, min_y = %d, max_y = %d", i, corridor[0], corridor[1], corridor[2], corridor[3]);
//     }

//     corridors = simplified_corridors;
// }

// // 将路径和Safe Corridor标记发布，并打印边界
// void publishPathAndCorridors(const std::vector<Node> &path, const std::vector<std::pair<Node, std::vector<int>>> &corridors, ros::Publisher &path_pub, ros::Publisher &marker_pub)
// {
//     nav_msgs::Path ros_path;
//     ros_path.header.stamp = ros::Time::now();
//     ros_path.header.frame_id = "map";

//     int id = 0;
//     for (size_t i = 0; i < path.size(); ++i)
//     {
//         geometry_msgs::PoseStamped pose;
//         pose.header.stamp = ros::Time::now();
//         pose.header.frame_id = "map";
//         pose.pose.position.x = path[i].x * map_data.info.resolution + map_data.info.origin.position.x;
//         pose.pose.position.y = path[i].y * map_data.info.resolution + map_data.info.origin.position.y;
//         pose.pose.position.z = 0.0;
//         pose.pose.orientation.w = 1.0;

//         ros_path.poses.push_back(pose);

//         // 打印corridor边界
//         int min_x = corridors[i].second[0];
//         int max_x = corridors[i].second[1];
//         int min_y = corridors[i].second[2];
//         int max_y = corridors[i].second[3];

//         ROS_INFO("Corridor for path point (%d, %d): min_x = %d, max_x = %d, min_y = %d, max_y = %d",
//                  path[i].x, path[i].y, min_x, max_x, min_y, max_y);

//         // 创建Marker用于显示路径点
//         visualization_msgs::Marker point_marker;
//         point_marker.header.frame_id = "map";
//         point_marker.header.stamp = ros::Time::now();
//         point_marker.ns = "path_points";
//         point_marker.id = id++;
//         point_marker.type = visualization_msgs::Marker::SPHERE;
//         point_marker.action = visualization_msgs::Marker::ADD;
//         point_marker.pose = pose.pose;
//         point_marker.scale.x = 0.1;
//         point_marker.scale.y = 0.1;
//         point_marker.scale.z = 0.1;

//         if (i == 0)
//         { // 起点
//             point_marker.color.r = 1.0;
//             point_marker.color.g = 0.0;
//             point_marker.color.b = 0.0;
//             point_marker.color.a = 1.0;
//         }
//         else if (i == path.size() - 1)
//         { // 终点
//             point_marker.color.r = 0.0;
//             point_marker.color.g = 0.0;
//             point_marker.color.b = 1.0;
//             point_marker.color.a = 1.0;
//         }
//         else
//         { // 其他路径点
//             point_marker.color.r = 0.0;
//             point_marker.color.g = 1.0;
//             point_marker.color.b = 0.0;
//             point_marker.color.a = 1.0;
//         }

//         marker_pub.publish(point_marker);

//         // 创建Marker用于显示Safe Corridor
//         visualization_msgs::Marker corridor_marker;
//         corridor_marker.header.frame_id = "map";
//         corridor_marker.header.stamp = ros::Time::now();
//         corridor_marker.ns = "safe_corridor";
//         corridor_marker.id = id++;
//         corridor_marker.type = visualization_msgs::Marker::CUBE;
//         corridor_marker.action = visualization_msgs::Marker::ADD;

//         corridor_marker.pose.position.x = (min_x + max_x) / 2.0 * map_data.info.resolution + map_data.info.origin.position.x;
//         corridor_marker.pose.position.y = (min_y + max_y) / 2.0 * map_data.info.resolution + map_data.info.origin.position.y;
//         corridor_marker.pose.position.z = 0.0;
//         corridor_marker.pose.orientation.w = 1.0;

//         corridor_marker.scale.x = (max_x - min_x + 1) * map_data.info.resolution;
//         corridor_marker.scale.y = (max_y - min_y + 1) * map_data.info.resolution;
//         corridor_marker.scale.z = 0.1; // 厚度可以根据需要调整

//         corridor_marker.color.r = 0.0;
//         corridor_marker.color.g = 1.0;
//         corridor_marker.color.b = 1.0;
//         corridor_marker.color.a = 0.5; // 半透明

//         marker_pub.publish(corridor_marker);
//     }

//     path_pub.publish(ros_path);
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "astar_path_planning");
//     ros::NodeHandle nh;

//     ros::Subscriber map_sub = nh.subscribe("/inflated_map", 1, mapCallback);

//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);
//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("path_markers", 1);

//     // 设置发布频率
//     ros::Rate rate(1.0); // 每秒发布一次路径

//     // 等待地图数据接收
//     while (!map_received && ros::ok())
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     int start_x = 10; // 示例起点
//     int start_y = 10;
//     int goal_x = 480; // 示例终点
//     int goal_y = 480;

//     while (ros::ok())
//     {
//         std::vector<Node> path = astar(start_x, start_y, goal_x, goal_y);

//         if (!path.empty())
//         {
//             ROS_INFO("Path found.");
//             std::vector<std::pair<Node, std::vector<int>>> corridors = generateSafeCorridor(path);
//             ensureCorridorsConnected(path, corridors); // 确保 corridor 连通
//             simplifyCorridors(corridors);              // 对 corridors 进行进一步简化
//             publishPathAndCorridors(path, corridors, path_pub, marker_pub);
//         }
//         else
//         {
//             ROS_WARN("No path found.");
//         }

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }
















































// /*带有路径点简化的A*算法，并且能够使用中点插值法生成联通的corridor，corridor经过了简化，但是执行性能未经过优化*/
// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Path.h>
// #include <visualization_msgs/Marker.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <vector>
// #include <queue>
// #include <algorithm>
// #include <chrono> // 用于测量时间

// // 定义A*节点结构
// struct Node
// {
//     int x, y;
//     int g, h;
//     Node *parent;

//     Node(int _x, int _y, int _g, int _h, Node *_parent = nullptr)
//         : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

//     int f() const { return g + h; }
// };

// // 比较函数，用于优先队列
// struct CompareNode
// {
//     bool operator()(Node *a, Node *b)
//     {
//         return a->f() > b->f();
//     }
// };

// // 定义地图信息
// nav_msgs::OccupancyGrid map_data;
// bool map_received = false;

// void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
// {
//     map_data = *msg;
//     map_received = true;
//     ROS_INFO("Map received.");
// }

// // 计算曼哈顿距离
// int manhattanDistance(int x1, int y1, int x2, int y2)
// {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

// // 检查两点之间是否有障碍物
// bool isLineFree(int x1, int y1, int x2, int y2)
// {
//     int dx = abs(x2 - x1);
//     int dy = abs(y2 - y1);
//     int sx = (x1 < x2) ? 1 : -1;
//     int sy = (y1 < y2) ? 1 : -1;
//     int err = dx - dy;

//     while (true)
//     {
//         if (map_data.data[y1 * map_data.info.width + x1] != 0)
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

// // 简化路径
// std::vector<Node> simplifyPath(const std::vector<Node> &path)
// {
//     if (path.size() < 3)
//     {
//         return path; // 如果路径点少于3个，则无需简化
//     }

//     std::vector<Node> simplified_path;
//     simplified_path.push_back(path.front());

//     for (size_t i = 2; i < path.size(); ++i)
//     {
//         if (!isLineFree(simplified_path.back().x, simplified_path.back().y, path[i].x, path[i].y))
//         {
//             simplified_path.push_back(path[i - 1]);
//         }
//     }

//     simplified_path.push_back(path.back());
//     return simplified_path;
// }

// // A*算法函数
// std::vector<Node> astar(int start_x, int start_y, int goal_x, int goal_y)
// {
//     std::priority_queue<Node *, std::vector<Node *>, CompareNode> open_list;
//     std::vector<std::vector<bool>> closed_list(map_data.info.height, std::vector<bool>(map_data.info.width, false));

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
//         for (auto &dir : directions)
//         {
//             int new_x = current_node->x + dir.first;
//             int new_y = current_node->y + dir.second;

//             if (new_x >= 0 && new_x < map_data.info.width &&
//                 new_y >= 0 && new_y < map_data.info.height &&
//                 map_data.data[new_y * map_data.info.width + new_x] == 0 &&
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

// // 生成Safe Corridor
// std::vector<std::pair<Node, std::vector<int>>> generateSafeCorridor(const std::vector<Node> &path)
// {
//     std::vector<std::pair<Node, std::vector<int>>> safe_corridors;

//     for (const auto &node : path)
//     {
//         int min_x = node.x;
//         int max_x = node.x;
//         int min_y = node.y;
//         int max_y = node.y;

//         // 向左扩展
//         while (min_x > 0)
//         {
//             bool obstacle_found = false;
//             for (int y = min_y; y <= max_y; ++y)
//             {
//                 if (map_data.data[y * map_data.info.width + (min_x - 1)] != 0)
//                 {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found)
//             {
//                 break;
//             }
//             min_x--;
//         }

//         // 向右扩展
//         while (max_x < map_data.info.width - 1)
//         {
//             bool obstacle_found = false;
//             for (int y = min_y; y <= max_y; ++y)
//             {
//                 if (map_data.data[y * map_data.info.width + (max_x + 1)] != 0)
//                 {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found)
//             {
//                 break;
//             }
//             max_x++;
//         }

//         // 向上扩展
//         while (min_y > 0)
//         {
//             bool obstacle_found = false;
//             for (int x = min_x; x <= max_x; ++x)
//             {
//                 if (map_data.data[(min_y - 1) * map_data.info.width + x] != 0)
//                 {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found)
//             {
//                 break;
//             }
//             min_y--;
//         }

//         // 向下扩展
//         while (max_y < map_data.info.height - 1)
//         {
//             bool obstacle_found = false;
//             for (int x = min_x; x <= max_x; ++x)
//             {
//                 if (map_data.data[(max_y + 1) * map_data.info.width + x] != 0)
//                 {
//                     obstacle_found = true;
//                     break;
//                 }
//             }
//             if (obstacle_found)
//             {
//                 break;
//             }
//             max_y++;
//         }

//         // 再次检查所有方向的边界，确保安全
//         min_x = std::max(min_x, 0);
//         max_x = std::min(max_x, static_cast<int>(map_data.info.width - 1));
//         min_y = std::max(min_y, 0);
//         max_y = std::min(max_y, static_cast<int>(map_data.info.height - 1));

//         safe_corridors.push_back({node, {min_x, max_x, min_y, max_y}});
//     }

//     return safe_corridors;
// }

// // 检查两个 corridor 是否连通
// bool areCorridorsConnected(const std::vector<int> &corridor1, const std::vector<int> &corridor2)
// {
//     // 检查两个 corridor 是否在 x 和 y 方向上有重叠部分
//     return !(corridor1[1] < corridor2[0] || corridor2[1] < corridor1[0] || corridor1[3] < corridor2[2] || corridor2[3] < corridor1[2]);
// }

// // 递归插值中点，确保 corridors 连通
// void ensureCorridorsConnected(std::vector<Node> &path, std::vector<std::pair<Node, std::vector<int>>> &corridors)
// {
//     for (size_t i = 1; i < corridors.size(); ++i)
//     {
//         if (!areCorridorsConnected(corridors[i - 1].second, corridors[i].second))
//         {
//             // 找到两个不连通的corridor，计算中点并插入
//             int mid_x = (path[i - 1].x + path[i].x) / 2;
//             int mid_y = (path[i - 1].y + path[i].y) / 2;
//             Node mid_node(mid_x, mid_y, 0, 0);

//             path.insert(path.begin() + i, mid_node);
//             corridors = generateSafeCorridor(path);

//             // 递归调用，检查插入后的corridor是否连通
//             ensureCorridorsConnected(path, corridors);
//             break; // 递归处理后重新检查
//         }
//     }
// }

// // 简化corridors
// void simplifyCorridors(std::vector<std::pair<Node, std::vector<int>>> &corridors)
// {
//     std::vector<std::pair<Node, std::vector<int>>> simplified_corridors;
//     simplified_corridors.push_back(corridors[0]);

//     for (size_t i = 1; i < corridors.size(); ++i)
//     {
//         auto &prev_corridor = simplified_corridors.back().second;
//         auto &curr_corridor = corridors[i].second;

//         // 如果当前corridor完全包含在上一个corridor中
//         if (curr_corridor[0] >= prev_corridor[0] && curr_corridor[1] <= prev_corridor[1] &&
//             curr_corridor[2] >= prev_corridor[2] && curr_corridor[3] <= prev_corridor[3])
//         {
//             continue; // 忽略当前corridor，保留上一个corridor
//         }

//         // 如果两个corridor相同
//         if (curr_corridor == prev_corridor)
//         {
//             continue; // 忽略当前corridor
//         }

//         // 检查并修正corridor边界是否超过地图边界
//         curr_corridor[0] = std::max(curr_corridor[0], 0);
//         curr_corridor[1] = std::min(curr_corridor[1], static_cast<int>(map_data.info.width - 1));
//         curr_corridor[2] = std::max(curr_corridor[2], 0);
//         curr_corridor[3] = std::min(curr_corridor[3], static_cast<int>(map_data.info.height - 1));

//         simplified_corridors.push_back(corridors[i]);
//     }

//     corridors = simplified_corridors;
// }

// // 将路径和Safe Corridor标记发布，并打印边界
// void publishPathAndCorridors(const std::vector<Node> &path, const std::vector<std::pair<Node, std::vector<int>>> &corridors, ros::Publisher &path_pub, ros::Publisher &marker_pub)
// {
//     nav_msgs::Path ros_path;
//     ros_path.header.stamp = ros::Time::now();
//     ros_path.header.frame_id = "map";

//     int id = 0;
//     for (size_t i = 0; i < path.size(); ++i)
//     {
//         geometry_msgs::PoseStamped pose;
//         pose.header.stamp = ros::Time::now();
//         pose.header.frame_id = "map";
//         pose.pose.position.x = path[i].x * map_data.info.resolution + map_data.info.origin.position.x;
//         pose.pose.position.y = path[i].y * map_data.info.resolution + map_data.info.origin.position.y;
//         pose.pose.position.z = 0.0;
//         pose.pose.orientation.w = 1.0;

//         ros_path.poses.push_back(pose);

//         // 添加路径点的Marker
//         visualization_msgs::Marker point_marker;
//         point_marker.header.frame_id = "map";
//         point_marker.header.stamp = ros::Time::now();
//         point_marker.ns = "path_points";
//         point_marker.id = id++;
//         point_marker.type = visualization_msgs::Marker::SPHERE;
//         point_marker.action = visualization_msgs::Marker::ADD;
//         point_marker.pose = pose.pose;
//         point_marker.scale.x = 0.2;
//         point_marker.scale.y = 0.2;
//         point_marker.scale.z = 0.2;

//         if (i == 0)
//         { // 起点
//             point_marker.color.r = 0.0;
//             point_marker.color.g = 0.0;
//             point_marker.color.b = 0.0;
//             point_marker.color.a = 1.0;
//         }
//         else if (i == path.size() - 1)
//         { // 终点
//             point_marker.color.r = 0.0;
//             point_marker.color.g = 1.0;
//             point_marker.color.b = 0.0;
//             point_marker.color.a = 1.0;
//         }
//         else
//         { // 其他路径点
//             point_marker.color.r = 1.0;
//             point_marker.color.g = 0.0;
//             point_marker.color.b = 0.0;
//             point_marker.color.a = 1.0;
//         }

//         marker_pub.publish(point_marker);
//     }

//     path_pub.publish(ros_path);

//     // 发布简化后的corridor
//     for (const auto &corridor : corridors)
//     {
//         int min_x = corridor.second[0];
//         int max_x = corridor.second[1];
//         int min_y = corridor.second[2];
//         int max_y = corridor.second[3];

//         ROS_INFO("Simplified Corridor: min_x = %d, max_x = %d, min_y = %d, max_y = %d",
//                  min_x, max_x, min_y, max_y);

//         visualization_msgs::Marker corridor_marker;
//         corridor_marker.header.frame_id = "map";
//         corridor_marker.header.stamp = ros::Time::now();
//         corridor_marker.ns = "simplified_corridor";
//         corridor_marker.id = id++;
//         corridor_marker.type = visualization_msgs::Marker::CUBE;
//         corridor_marker.action = visualization_msgs::Marker::ADD;

//         corridor_marker.pose.position.x = (min_x + max_x) / 2.0 * map_data.info.resolution + map_data.info.origin.position.x;
//         corridor_marker.pose.position.y = (min_y + max_y) / 2.0 * map_data.info.resolution + map_data.info.origin.position.y;
//         corridor_marker.pose.position.z = 0.0;
//         corridor_marker.pose.orientation.w = 1.0;

//         corridor_marker.scale.x = (max_x - min_x + 1) * map_data.info.resolution;
//         corridor_marker.scale.y = (max_y - min_y + 1) * map_data.info.resolution;
//         corridor_marker.scale.z = 0.1; // 厚度可以根据需要调整

//         corridor_marker.color.r = 0.0;
//         corridor_marker.color.g = 1.0;
//         corridor_marker.color.b = 1.0;
//         corridor_marker.color.a = 0.5; // 半透明

//         marker_pub.publish(corridor_marker);
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "astar_path_planning");
//     ros::NodeHandle nh;

//     ros::Subscriber map_sub = nh.subscribe("/inflated_map", 1, mapCallback);

//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);
//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("path_markers", 1);

//     // 设置发布频率
//     ros::Rate rate(1.0); // 每秒发布一次路径

//     // 等待地图数据接收
//     while (!map_received && ros::ok())
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     int start_x = 10; // 示例起点
//     int start_y = 10;
//     int goal_x = 480; // 示例终点
//     int goal_y = 480;

//     while (ros::ok())
//     {
//         // 记录起始时间
//         auto start_time = std::chrono::high_resolution_clock::now();

//         std::vector<Node> path = astar(start_x, start_y, goal_x, goal_y);

//         if (!path.empty())
//         {
//             ROS_INFO("Path found.");
//             std::vector<std::pair<Node, std::vector<int>>> corridors = generateSafeCorridor(path);
//             ensureCorridorsConnected(path, corridors); // 确保 corridor 连通
//             simplifyCorridors(corridors);              // 对 corridors 进行进一步简化
//             publishPathAndCorridors(path, corridors, path_pub, marker_pub);
//         }
//         else
//         {
//             ROS_WARN("No path found.");
//         }
//         // 记录结束时间
//         auto end_time = std::chrono::high_resolution_clock::now();
//         std::chrono::duration<double> elapsed_time = end_time - start_time;

//         // 输出执行时间
//         ROS_INFO("Execution time: %.6f seconds", elapsed_time.count());
//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }





























// /*带有路径点简化的A*算法，并且能够使用中点插值法生成联通的corridor，corridor经过了简化，执行性能经过了优化*/
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>
#include <algorithm>
#include <chrono>  // 用于测量时间
#include "Gen_Starts_Goals.h"

// 定义A*节点结构
struct Node {
    int x, y;
    int g, h;
    Node* parent;

    Node(int _x, int _y, int _g, int _h, Node* _parent = nullptr)
        : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

    int f() const { return g + h; }
};

// 比较函数，用于优先队列
struct CompareNode {
    bool operator()(Node* a, Node* b) {
        return a->f() > b->f();
    }
};

// 定义地图信息
nav_msgs::OccupancyGrid map_data;
bool map_received = false;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_data = *msg;
    map_received = true;
    ROS_INFO("Map received.");
}

// 计算曼哈顿距离
int manhattanDistance(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

// 检查两点之间是否有障碍物
bool isLineFree(int x1, int y1, int x2, int y2) {
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (map_data.data[y1 * map_data.info.width + x1] != 0) {
            return false;  // 如果线段经过障碍物，返回false
        }
        if (x1 == x2 && y1 == y2) {
            break;  // 到达终点
        }
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    return true;
}

// 简化路径
std::vector<Node> simplifyPath(const std::vector<Node>& path) {
    if (path.size() < 3) {
        return path;  // 如果路径点少于3个，则无需简化
    }

    std::vector<Node> simplified_path;
    simplified_path.reserve(path.size());  // 预留内存以提高性能
    simplified_path.push_back(path.front());

    for (size_t i = 2; i < path.size(); ++i) {
        if (!isLineFree(simplified_path.back().x, simplified_path.back().y, path[i].x, path[i].y)) {
            simplified_path.push_back(path[i - 1]);
        }
    }

    simplified_path.push_back(path.back());
    return simplified_path;
}

// A*算法函数
std::vector<Node> astar(int start_x, int start_y, int goal_x, int goal_y) {
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
    std::vector<std::vector<bool>> closed_list(map_data.info.height, std::vector<bool>(map_data.info.width, false));

    Node* start_node = new Node(start_x, start_y, 0, manhattanDistance(start_x, start_y, goal_x, goal_y));
    open_list.push(start_node);

    while (!open_list.empty()) {
        Node* current_node = open_list.top();
        open_list.pop();

        if (current_node->x == goal_x && current_node->y == goal_y) {
            std::vector<Node> path;
            Node* path_node = current_node;
            while (path_node) {
                path.push_back(*path_node);
                path_node = path_node->parent;
            }
            std::reverse(path.begin(), path.end());
            return simplifyPath(path);  // 返回简化后的路径
        }

        closed_list[current_node->y][current_node->x] = true;

        std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
        for (auto& dir : directions) {
            int new_x = current_node->x + dir.first;
            int new_y = current_node->y + dir.second;

            if (new_x >= 0 && new_x < map_data.info.width &&
                new_y >= 0 && new_y < map_data.info.height &&
                map_data.data[new_y * map_data.info.width + new_x] == 0 &&
                !closed_list[new_y][new_x]) {

                int new_g = current_node->g + 1;
                int new_h = manhattanDistance(new_x, new_y, goal_x, goal_y);
                Node* new_node = new Node(new_x, new_y, new_g, new_h, current_node);
                open_list.push(new_node);
            }
        }
    }

    return std::vector<Node>(); // 返回空路径表示未找到路径
}

// 生成Safe Corridor
std::vector<std::pair<Node, std::vector<int>>> generateSafeCorridor(const std::vector<Node>& path) {
    std::vector<std::pair<Node, std::vector<int>>> safe_corridors;
    safe_corridors.reserve(path.size());  // 预留内存以提高性能

    for (const auto& node : path) {
        int min_x = node.x;
        int max_x = node.x;
        int min_y = node.y;
        int max_y = node.y;

        // 向左扩展
        while (min_x > 0) {
            bool obstacle_found = false;
            for (int y = min_y; y <= max_y; ++y) {
                if (map_data.data[y * map_data.info.width + (min_x - 1)] != 0) {
                    obstacle_found = true;
                    break;
                }
            }
            if (obstacle_found) {
                break;
            }
            min_x--;
        }

        // 向右扩展
        while (max_x < map_data.info.width - 1) {
            bool obstacle_found = false;
            for (int y = min_y; y <= max_y; ++y) {
                if (map_data.data[y * map_data.info.width + (max_x + 1)] != 0) {
                    obstacle_found = true;
                    break;
                }
            }
            if (obstacle_found) {
                break;
            }
            max_x++;
        }

        // 向上扩展
        while (min_y > 0) {
            bool obstacle_found = false;
            for (int x = min_x; x <= max_x; ++x) {
                if (map_data.data[(min_y - 1) * map_data.info.width + x] != 0) {
                    obstacle_found = true;
                    break;
                }
            }
            if (obstacle_found) {
                break;
            }
            min_y--;
        }

        // 向下扩展
        while (max_y < map_data.info.height - 1) {
            bool obstacle_found = false;
            for (int x = min_x; x <= max_x; ++x) {
                if (map_data.data[(max_y + 1) * map_data.info.width + x] != 0) {
                    obstacle_found = true;
                    break;
                }
            }
            if (obstacle_found) {
                break;
            }
            max_y++;
        }

        // 再次检查所有方向的边界，确保安全
        min_x = std::max(min_x, 0);
        max_x = std::min(max_x, static_cast<int>(map_data.info.width - 1));
        min_y = std::max(min_y, 0);
        max_y = std::min(max_y, static_cast<int>(map_data.info.height - 1));

        safe_corridors.push_back({node, {min_x, max_x, min_y, max_y}});
    }

    return safe_corridors;
}

// 检查两个 corridor 是否连通
bool areCorridorsConnected(const std::vector<int>& corridor1, const std::vector<int>& corridor2) {
    // 检查两个 corridor 是否在 x 和 y 方向上有重叠部分
    return !(corridor1[1] < corridor2[0] || corridor2[1] < corridor1[0] || corridor1[3] < corridor2[2] || corridor2[3] < corridor1[2]);
}

// 递归插值中点，确保 corridors 连通
void ensureCorridorsConnected(std::vector<Node>& path, std::vector<std::pair<Node, std::vector<int>>>& corridors) {
    for (size_t i = 1; i < corridors.size(); ++i) {
        if (!areCorridorsConnected(corridors[i - 1].second, corridors[i].second)) {
            // 找到两个不连通的corridor，计算中点并插入
            int mid_x = (path[i - 1].x + path[i].x) / 2;
            int mid_y = (path[i - 1].y + path[i].y) / 2;
            Node mid_node(mid_x, mid_y, 0, 0);

            path.insert(path.begin() + i, mid_node);
            corridors = generateSafeCorridor(path);

            // 递归调用，检查插入后的corridor是否连通
            ensureCorridorsConnected(path, corridors);
            break;  // 递归处理后重新检查
        }
    }
}

// 简化corridors
void simplifyCorridors(std::vector<std::pair<Node, std::vector<int>>>& corridors) {
    std::vector<std::pair<Node, std::vector<int>>> simplified_corridors;
    simplified_corridors.reserve(corridors.size());  // 预留内存以提高性能
    simplified_corridors.push_back(corridors[0]);

    for (size_t i = 1; i < corridors.size(); ++i) {
        auto& prev_corridor = simplified_corridors.back().second;
        auto& curr_corridor = corridors[i].second;

        // 如果当前corridor完全包含在上一个corridor中
        if (curr_corridor[0] >= prev_corridor[0] && curr_corridor[1] <= prev_corridor[1] &&
            curr_corridor[2] >= prev_corridor[2] && curr_corridor[3] <= prev_corridor[3]) {
            continue;  // 忽略当前corridor，保留上一个corridor
        }

        // 如果两个corridor相同
        if (curr_corridor == prev_corridor) {
            continue;  // 忽略当前corridor
        }

        // 检查并修正corridor边界是否超过地图边界
        curr_corridor[0] = std::max(curr_corridor[0], 0);
        curr_corridor[1] = std::min(curr_corridor[1], static_cast<int>(map_data.info.width - 1));
        curr_corridor[2] = std::max(curr_corridor[2], 0);
        curr_corridor[3] = std::min(curr_corridor[3], static_cast<int>(map_data.info.height - 1));

        simplified_corridors.push_back(corridors[i]);
    }

    corridors = simplified_corridors;
}

// 将路径和Safe Corridor标记发布，并打印边界
void publishPathAndCorridors(const std::vector<Node>& path, const std::vector<std::pair<Node, std::vector<int>>>& corridors, ros::Publisher& path_pub, ros::Publisher& marker_pub) {
    nav_msgs::Path ros_path;
    ros_path.header.stamp = ros::Time::now();
    ros_path.header.frame_id = "map";

    int id = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = path[i].x * map_data.info.resolution + map_data.info.origin.position.x;
        pose.pose.position.y = path[i].y * map_data.info.resolution + map_data.info.origin.position.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        ros_path.poses.push_back(pose);

        // 添加路径点的Marker
        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = "map";
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "path_points";
        point_marker.id = id++;
        point_marker.type = visualization_msgs::Marker::SPHERE;
        point_marker.action = visualization_msgs::Marker::ADD;
        point_marker.pose = pose.pose;
        point_marker.scale.x = 0.2;
        point_marker.scale.y = 0.2;
        point_marker.scale.z = 0.2;

        if (i == 0) {  // 起点
            point_marker.color.r = 0.0;
            point_marker.color.g = 0.0;
            point_marker.color.b = 0.0;
            point_marker.color.a = 1.0;
        } else if (i == path.size() - 1) {  // 终点
            point_marker.color.r = 0.0;
            point_marker.color.g = 1.0;
            point_marker.color.b = 0.0;
            point_marker.color.a = 1.0;
        } else {  // 其他路径点
            point_marker.color.r = 1.0;
            point_marker.color.g = 0.0;
            point_marker.color.b = 0.0;
            point_marker.color.a = 1.0;
        }

        marker_pub.publish(point_marker);
    }

    path_pub.publish(ros_path);

    // 发布简化后的corridor
    for (const auto& corridor : corridors) {
        int min_x = corridor.second[0];
        int max_x = corridor.second[1];
        int min_y = corridor.second[2];
        int max_y = corridor.second[3];

        ROS_INFO("Simplified Corridor: min_x = %d, max_x = %d, min_y = %d, max_y = %d", 
                 min_x, max_x, min_y, max_y);

        visualization_msgs::Marker corridor_marker;
        corridor_marker.header.frame_id = "map";
        corridor_marker.header.stamp = ros::Time::now();
        corridor_marker.ns = "simplified_corridor";
        corridor_marker.id = id++;
        corridor_marker.type = visualization_msgs::Marker::CUBE;
        corridor_marker.action = visualization_msgs::Marker::ADD;

        corridor_marker.pose.position.x = (min_x + max_x) / 2.0 * map_data.info.resolution + map_data.info.origin.position.x;
        corridor_marker.pose.position.y = (min_y + max_y) / 2.0 * map_data.info.resolution + map_data.info.origin.position.y;
        corridor_marker.pose.position.z = 0.0;
        corridor_marker.pose.orientation.w = 1.0;

        corridor_marker.scale.x = (max_x - min_x + 1) * map_data.info.resolution;
        corridor_marker.scale.y = (max_y - min_y + 1) * map_data.info.resolution;
        corridor_marker.scale.z = 0.1; // 厚度可以根据需要调整

        corridor_marker.color.r = 0.0;
        corridor_marker.color.g = 1.0;
        corridor_marker.color.b = 1.0;
        corridor_marker.color.a = 0.5; // 半透明

        marker_pub.publish(corridor_marker);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_path_planning");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/inflated_map", 1, mapCallback);

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("path_markers", 1);


    // 创建 Gen_Starts_Goals 对象，并从参数服务器获取robot_count和robot_radius
    Gen_Starts_Goals planner(nh);
    // 获取生成的起点和终点
    const auto& start_positions = planner.getStartPositions();
    const auto& goal_positions = planner.getGoalPositions();

    for (size_t i = 0; i < start_positions.size(); ++i) {
        ROS_INFO("Robot %lu: Start (%d, %d), Goal (%d, %d)", i + 1,
                 start_positions[i].first, start_positions[i].second,
                 goal_positions[i].first, goal_positions[i].second);
    }


    // 设置发布频率
    ros::Rate rate(1.0); // 每秒发布一次路径

    // 等待地图数据接收
    while (!map_received && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    int start_x = 10;  // 示例起点
    int start_y = 10;
    int goal_x = 300;   // 示例终点
    int goal_y = 300;

    while (ros::ok()) {
        // 记录起始时间
        auto start_time = std::chrono::high_resolution_clock::now();

        std::vector<Node> path = astar(start_x, start_y, goal_x, goal_y);

        if (!path.empty()) {
            ROS_INFO("Path found.");
            std::vector<std::pair<Node, std::vector<int>>> corridors = generateSafeCorridor(path);
            ensureCorridorsConnected(path, corridors);  // 确保 corridor 连通
            simplifyCorridors(corridors);  // 对 corridors 进行进一步简化
            publishPathAndCorridors(path, corridors, path_pub, marker_pub);
        } else {
            ROS_WARN("No path found.");
        }

        // 记录结束时间
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_time = end_time - start_time;

        // 输出执行时间
        ROS_INFO("Execution time: %.6f seconds", elapsed_time.count());

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}