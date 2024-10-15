#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <queue>
#include <cmath>

class MapInflator {
public:
    MapInflator() : map_received(false) {
        // 获取机器人的半径
        double robot_radius;
        if (!nh.getParam("robot_radius", robot_radius)) {
            ROS_WARN("Failed to get robot_radius, using default value 0.5");
            robot_radius = 0.5; // 默认机器人半径0.5米
        }

        // 等待第一次地图消息以获取地图分辨率
        // ROS_INFO("Waiting for the first map message to get resolution...");
        auto msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", nh);
        if (msg) {
            resolution = msg->info.resolution;
            // ROS_INFO("Map resolution: %f", resolution);

            // 计算膨胀半径
            inflation_radius = std::ceil(robot_radius / resolution);
            double_inflation_radius = std::ceil(1.5 * robot_radius / resolution);
            // ROS_INFO("Calculated inflation_radius: %d", inflation_radius);
            // ROS_INFO("Calculated double_inflation_radius: %d", double_inflation_radius);

            // 初始化发布器
            inflated_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("inflated_map", 1);
            double_inflated_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("double_inflated_map", 1);

            // 初始化订阅器
            map_sub = nh.subscribe("map", 100, &MapInflator::mapCallback, this);

            // 设置定时器，定期调用 mapInflationAndPublish
            timer = nh.createTimer(ros::Duration(1.0), &MapInflator::mapInflationAndPublish, this);
        } else {
            ROS_WARN("Failed to receive the first map message.");
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher inflated_map_pub;
    ros::Publisher double_inflated_map_pub;
    ros::Subscriber map_sub;
    ros::Timer timer;
    int inflation_radius;
    int double_inflation_radius;
    int width, height;
    double resolution;
    bool map_received;

    std::vector<std::vector<int>> original_map;
    std::vector<std::vector<int>> inflated_map;
    std::vector<std::vector<int>> double_inflated_map;
    nav_msgs::OccupancyGrid last_received_map;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        // ROS_INFO("mapCallback has been called");

        if (width != msg->info.width || height != msg->info.height) {
            width = msg->info.width;
            height = msg->info.height;
            resolution = msg->info.resolution;

            original_map.assign(height, std::vector<int>(width, 0));
            inflated_map.assign(height, std::vector<int>(width, 0));
            double_inflated_map.assign(height, std::vector<int>(width, 0));
        }

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = x + y * width;
                original_map[y][x] = msg->data[index];
                // inflated_map[y][x] = original_map[y][x];
                // double_inflated_map[y][x] = original_map[y][x];
            }
        }

        last_received_map = *msg;  // 存储最后一次接收到的地图
        map_received = true;
    }

    void mapInflationAndPublish(const ros::TimerEvent&) {
        if (!map_received) {
            ROS_WARN("No map received yet, skipping inflation and publish.");
            return;
        }

        // 复制原始地图数据
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                inflated_map[y][x] = original_map[y][x];
                double_inflated_map[y][x] = original_map[y][x];
            }
        }

        // 使用BFS进行障碍物膨胀
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (original_map[y][x] > 50) {  // 假设大于50的值为障碍物

                    // 膨胀该障碍物 (普通膨胀)
                    for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                        for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;

                            // 检查是否在地图边界内
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                // 使用欧几里得距离判断是否在膨胀半径内
                                if (std::hypot(dx, dy) <= inflation_radius) {
                                    inflated_map[ny][nx] = 100;  // 将膨胀区域标记为障碍物
                                }
                            }
                        }
                    }

                    // 膨胀该障碍物 (双倍膨胀)
                    for (int dy = -double_inflation_radius; dy <= double_inflation_radius; ++dy) {
                        for (int dx = -double_inflation_radius; dx <= double_inflation_radius; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;

                            // 检查是否在地图边界内
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                // 使用欧几里得距离判断是否在膨胀半径内
                                if (std::hypot(dx, dy) <= double_inflation_radius) {
                                    double_inflated_map[ny][nx] = 100;  // 将双倍膨胀区域标记为障碍物
                                }
                            }
                        }
                    }
                }
            }
        }

        // 将膨胀后的地图转换为OccupancyGrid消息 (普通膨胀)
        nav_msgs::OccupancyGrid inflated_map_msg;
        inflated_map_msg.header.stamp = ros::Time::now();
        inflated_map_msg.header.frame_id = "map";
        inflated_map_msg.info = last_received_map.info;
        inflated_map_msg.data.resize(width * height);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = x + y * width;
                inflated_map_msg.data[index] = inflated_map[y][x];
            }
        }

        // 发布普通膨胀后的地图
        inflated_map_pub.publish(inflated_map_msg);
        // ROS_INFO("Published inflated map.");

        // 将双倍膨胀后的地图转换为OccupancyGrid消息 (双倍膨胀)
        nav_msgs::OccupancyGrid double_inflated_map_msg;
        double_inflated_map_msg.header.stamp = ros::Time::now();
        double_inflated_map_msg.header.frame_id = "map";
        double_inflated_map_msg.info = last_received_map.info;
        double_inflated_map_msg.data.resize(width * height);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = x + y * width;
                double_inflated_map_msg.data[index] = double_inflated_map[y][x];
            }
        }

        // 发布双倍膨胀后的地图
        double_inflated_map_pub.publish(double_inflated_map_msg);
        // ROS_INFO("Published double inflated map.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_inflator");

    MapInflator inflator;

    ros::spin();

    return 0;
}













// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <vector>
// #include <queue>
// #include <cmath>

// class MapInflator {
// public:
//     MapInflator() : map_received(false) {
//         // 获取机器人的半径
//         double robot_radius;
//         if (!nh.getParam("robot_radius", robot_radius)) {
//             ROS_WARN("Failed to get robot_radius, using default value 0.5");
//             robot_radius = 0.5; // 默认机器人半径0.5米
//         }

//         // 等待第一次地图消息以获取地图分辨率
//         ROS_INFO("Waiting for the first map message to get resolution...");
//         auto msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", nh);
//         if (msg) {
//             resolution = msg->info.resolution;
//             ROS_INFO("Map resolution: %f", resolution);

//             // 计算膨胀半径
//             inflation_radius = std::ceil(robot_radius / resolution);
//             ROS_INFO("Calculated inflation_radius: %d", inflation_radius);

//             // 初始化发布器
//             inflated_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("inflated_map", 1);

//             // 初始化订阅器
//             map_sub = nh.subscribe("map", 100, &MapInflator::mapCallback, this);

//             // 设置定时器，定期调用 mapInflationAndPublish
//             timer = nh.createTimer(ros::Duration(1.0), &MapInflator::mapInflationAndPublish, this);
//         } else {
//             ROS_WARN("Failed to receive the first map message.");
//         }
//     }

// private:
//     ros::NodeHandle nh;
//     ros::Publisher inflated_map_pub;
//     ros::Subscriber map_sub;
//     ros::Timer timer;
//     int inflation_radius;
//     int width, height;
//     double resolution;
//     bool map_received;

//     std::vector<std::vector<int>> original_map;
//     std::vector<std::vector<int>> inflated_map;
//     nav_msgs::OccupancyGrid last_received_map;

//     void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
//         ROS_INFO("mapCallback has been called");

//         if (width != msg->info.width || height != msg->info.height) {
//             width = msg->info.width;
//             height = msg->info.height;
//             resolution = msg->info.resolution;

//             original_map.assign(height, std::vector<int>(width, 0));
//             inflated_map.assign(height, std::vector<int>(width, 0));
//         }

//         for (int y = 0; y < height; ++y) {
//             for (int x = 0; x < width; ++x) {
//                 int index = x + y * width;
//                 original_map[y][x] = msg->data[index];
//                 inflated_map[y][x] = original_map[y][x];
//             }
//         }

//         last_received_map = *msg;  // 存储最后一次接收到的地图
//         map_received = true;
//     }

//     void mapInflationAndPublish(const ros::TimerEvent&) {
//         if (!map_received) {
//             ROS_WARN("No map received yet, skipping inflation and publish.");
//             return;
//         }

//         // 复制原始地图数据
//         for (int y = 0; y < height; ++y) {
//             for (int x = 0; x < width; ++x) {
//                 inflated_map[y][x] = original_map[y][x];
//             }
//         }

//         // 使用BFS进行障碍物膨胀
//         for (int y = 0; y < height; ++y) {
//             for (int x = 0; x < width; ++x) {
//                 if (original_map[y][x] > 50) {  // 假设大于50的值为障碍物
//                     // 膨胀该障碍物
//                     for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
//                         for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
//                             int nx = x + dx;
//                             int ny = y + dy;

//                             // 检查是否在地图边界内
//                             if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
//                                 // 使用欧几里得距离判断是否在膨胀半径内
//                                 if (std::hypot(dx, dy) <= inflation_radius) {
//                                     inflated_map[ny][nx] = 100;  // 将膨胀区域标记为障碍物
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }
//         }

//         // 将膨胀后的地图转换为OccupancyGrid消息
//         nav_msgs::OccupancyGrid inflated_map_msg;
//         inflated_map_msg.header.stamp = ros::Time::now();
//         inflated_map_msg.header.frame_id = "map";
//         inflated_map_msg.info = last_received_map.info;
//         inflated_map_msg.data.resize(width * height);

//         for (int y = 0; y < height; ++y) {
//             for (int x = 0; x < width; ++x) {
//                 int index = x + y * width;
//                 inflated_map_msg.data[index] = inflated_map[y][x];
//             }
//         }

//         // 发布膨胀后的地图
//         inflated_map_pub.publish(inflated_map_msg);
//         ROS_INFO("Published inflated map.");
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "map_inflator");

//     MapInflator inflator;

//     ros::spin();

//     return 0;
// }