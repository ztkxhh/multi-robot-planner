/*利用a*生成路径，不做简化，直接corridor，检查连通性。如果不连通报错，如果联通则进行简化*/
/*并且Node使用共享指针*/
/*采用两个膨胀地图分别进行路径规划和corridor生成*/
/*添加rviz的显示*/
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <queue>
#include <utility>
#include <chrono>
#include "mosek.h"
#include <memory>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "gurobi_c++.h"


#include "Gen_Starts_Goals.h"
#include "bezier_base.h"

struct Node {
    int x, y;
    int g, h;
    std::shared_ptr<Node> parent;

    Node(int _x, int _y, int _g, int _h, std::shared_ptr<Node> _parent = nullptr)
        : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

    int f() const { return g + h; }
};

struct CompareNode {
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        return a->f() > b->f();
    }
};

class Path_Planner {
public:
    Path_Planner(ros::NodeHandle& nh);
    ~Path_Planner() = default;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void doubleMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    bool planPaths();

    const std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>& getCorridors(size_t robot_index) const;
    const std::vector<std::pair<int, int>>& getStartPositions() const;
    const std::vector<std::pair<int, int>>& getGoalPositions() const;

    std::pair<int, int> getStart(size_t robot_index) const;
    std::pair<int, int> getGoal(size_t robot_index) const;

    bool mapReceived() const { return map_received_; }
    bool doubleMapReceived() const { return double_map_received_; }

    void publishPathVisualization(size_t robot_index, ros::Publisher& marker_pub);

    int MultiRobotTraGen(
    const std::vector<std::vector< std::vector<int>>>  & corridors,
    const MatrixXd & MQM_jerk,
    const MatrixXd & MQM_length,
    double w_1, double w_2,
    const  std::vector<std::pair<int, int>> & start_positions,
    const std::vector<std::pair<int, int>> & goal_positions,
    const double & minimize_order,
    const int & curve_order,
    const double & min_threshold);

    std::vector<std::vector< std::vector<std::pair<double,double>>>> all_control_points;
    std::vector<std::vector< std::vector<std::pair<double,double>>>> getControlPoints(){ return all_control_points; }

private:
    int manhattanDistance(int x1, int y1, int x2, int y2);
    std::vector<std::shared_ptr<Node>> astar(const nav_msgs::OccupancyGrid& map, int start_x, int start_y, int goal_x, int goal_y, int max_steps);
    std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> generateSafeCorridor(const nav_msgs::OccupancyGrid& map, const std::vector<std::shared_ptr<Node>>& path);
    bool areCorridorsConnected(const std::vector<int>& corridor1, const std::vector<int>& corridor2);
    void simplifyCorridors(std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>& corridors);
    void removeRedundantCorridors(std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>& corridors);
    void inflateObstacle(int goal_x, int goal_y, nav_msgs::OccupancyGrid& map);

    ros::Subscriber map_sub_;
    ros::Subscriber double_map_sub_;

    nav_msgs::OccupancyGrid map_data_;
    nav_msgs::OccupancyGrid original_map_;
    bool map_received_ = false;

    nav_msgs::OccupancyGrid double_map_data_;
    bool double_map_received_ = false;

    int inflation_radius_;

    Gen_Starts_Goals planner_;
    std::vector<std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>> robot_corridors_;
};

#endif // PATH_PLANNER_H





























// /*利用a*生成路径，不做简化，直接corridor，检查连通性。如果不连通报错，如果联通则进行简化*/
// /*并且Node使用共享指针*/
// /*采用两个膨胀地图分别进行路径规划和corridor生成*/


// #ifndef PATH_PLANNER_H
// #define PATH_PLANNER_H

// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <vector>
// #include <queue>
// #include <utility>
// #include <chrono>
// #include "Gen_Starts_Goals.h"
// #include <memory>

// struct Node {
//     int x, y;
//     int g, h;
//     std::shared_ptr<Node> parent;  // 使用 shared_ptr 代替裸指针

//     Node(int _x, int _y, int _g, int _h, std::shared_ptr<Node> _parent = nullptr)
//         : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

//     int f() const { return g + h; }
// };

// struct CompareNode {
//     bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
//         return a->f() > b->f();
//     }
// };

// class Path_Planner {
// public:
//     Path_Planner(ros::NodeHandle& nh);
//     ~Path_Planner() = default;

//     void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
//     void doubleMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg); // 新增

//     void planPaths();

//     const std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>& getCorridors(size_t robot_index) const;
//     const std::vector<std::pair<int, int>>& getStartPositions() const;
//     const std::vector<std::pair<int, int>>& getGoalPositions() const;

//     std::pair<int, int> getStart(size_t robot_index) const;
//     std::pair<int, int> getGoal(size_t robot_index) const;

//     bool mapReceived() const { return map_received_; }
//     bool doubleMapReceived() const { return double_map_received_; } // 新增获取double map的函数

// private:
//     int manhattanDistance(int x1, int y1, int x2, int y2);
//     std::vector<std::shared_ptr<Node>> astar(const nav_msgs::OccupancyGrid& map, int start_x, int start_y, int goal_x, int goal_y, int max_steps); // 修改
//     std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> generateSafeCorridor(const std::vector<std::shared_ptr<Node>>& path);
//     bool areCorridorsConnected(const std::vector<int>& corridor1, const std::vector<int>& corridor2);
//     void simplifyCorridors(std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>& corridors);
//     void inflateObstacle(int goal_x, int goal_y, nav_msgs::OccupancyGrid& map);
//     void removeRedundantCorridors(std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> &corridors);
//     ros::Subscriber map_sub_;
//     ros::Subscriber double_map_sub_; // 新增

//     nav_msgs::OccupancyGrid map_data_;
//     nav_msgs::OccupancyGrid original_map_;
//     bool map_received_ = false;

//     nav_msgs::OccupancyGrid double_map_data_; // 新增
//     bool double_map_received_ = false; // 新增

//     int inflation_radius_;

//     Gen_Starts_Goals planner_;
//     std::vector<std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>> robot_corridors_;
// };

// #endif // PATH_PLANNER_H


















// /*利用a*生成路径，不做简化，直接corridor，检查连通性。如果不连通报错，如果联通则进行简化*/
// /*并且Node使用共享指针*/

// #ifndef PATH_PLANNER_H
// #define PATH_PLANNER_H

// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <vector>
// #include <queue>
// #include <utility>
// #include <chrono>
// #include "Gen_Starts_Goals.h"
// #include <memory>

// struct Node {
//     int x, y;
//     int g, h;
//     std::shared_ptr<Node> parent;  // 使用 shared_ptr 代替裸指针

//     Node(int _x, int _y, int _g, int _h, std::shared_ptr<Node> _parent = nullptr)
//         : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

//     int f() const { return g + h; }
// };

// struct CompareNode {
//     bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
//         return a->f() > b->f();
//     }
// };

// class Path_Planner {
// public:
//     Path_Planner(ros::NodeHandle& nh);
//     ~Path_Planner() = default;

//     void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
//     void planPaths();

//     const std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>& getCorridors(size_t robot_index) const;
//     const std::vector<std::pair<int, int>>& getStartPositions() const;
//     const std::vector<std::pair<int, int>>& getGoalPositions() const;

//     std::pair<int, int> getStart(size_t robot_index) const;
//     std::pair<int, int> getGoal(size_t robot_index) const;

//     bool mapReceived() const { return map_received_; }

// private:
//     int manhattanDistance(int x1, int y1, int x2, int y2);
//     std::vector<std::shared_ptr<Node>> astar(int start_x, int start_y, int goal_x, int goal_y, int max_steps);
//     std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>> generateSafeCorridor(const std::vector<std::shared_ptr<Node>>& path);
//     bool areCorridorsConnected(const std::vector<int>& corridor1, const std::vector<int>& corridor2);
//     void simplifyCorridors(std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>& corridors);
//     void inflateObstacle(int goal_x, int goal_y, nav_msgs::OccupancyGrid& map);

//     ros::Subscriber map_sub_;
//     nav_msgs::OccupancyGrid map_data_;
//     nav_msgs::OccupancyGrid original_map_;
//     bool map_received_ = false;
//     int inflation_radius_;

//     Gen_Starts_Goals planner_;
//     std::vector<std::vector<std::pair<std::shared_ptr<Node>, std::vector<int>>>> robot_corridors_;
// };

// #endif // PATH_PLANNER_H






























// // /*在生成了路径之后，做简化路径，并且在两个corridor不连通时采用中点插值的方法确保联通性*/
// // /*但是会出现中点插值收敛到一个点的现象*/
// #ifndef PATH_PLANNER_H
// #define PATH_PLANNER_H

// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <vector>
// #include <queue>
// #include <utility>
// #include <chrono>
// #include "Gen_Starts_Goals.h"
// #include <memory>

// struct Node {
//     int x, y;
//     int g, h;
//     Node* parent;

//     Node(int _x, int _y, int _g, int _h, Node* _parent = nullptr)
//         : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

//     int f() const { return g + h; }
// };

// struct CompareNode {
//     bool operator()(Node* a, Node* b) {
//         return a->f() > b->f();
//     }
// };

// class Path_Planner {
// public:
//     Path_Planner(ros::NodeHandle& nh);
//     ~Path_Planner() = default;

//     void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
//     void planPaths();

//     const std::vector<std::pair<Node, std::vector<int>>>& getCorridors(size_t robot_index) const;
//     const std::vector<std::pair<int, int>>& getStartPositions() const;
//     const std::vector<std::pair<int, int>>& getGoalPositions() const;

//     std::pair<int, int> getStart(size_t robot_index) const;
//     std::pair<int, int> getGoal(size_t robot_index) const;

//     bool mapReceived() const { return map_received_; }

// private:
//     int manhattanDistance(int x1, int y1, int x2, int y2);
//     bool isLineFree(int x1, int y1, int x2, int y2);
//     std::vector<Node> simplifyPath(const std::vector<Node>& path);
//     std::vector<Node> astar(int start_x, int start_y, int goal_x, int goal_y);
//     std::vector<std::pair<Node, std::vector<int>>> generateSafeCorridor(const std::vector<Node>& path);
//     bool areCorridorsConnected(const std::vector<int>& corridor1, const std::vector<int>& corridor2);
//     void ensureCorridorsConnected(std::vector<Node>& path, std::vector<std::pair<Node, std::vector<int>>>& corridors);
//     void simplifyCorridors(std::vector<std::pair<Node, std::vector<int>>>& corridors);
//     void inflateObstacle(int goal_x, int goal_y, nav_msgs::OccupancyGrid& map);

//     ros::Subscriber map_sub_;
//     nav_msgs::OccupancyGrid map_data_;
//     nav_msgs::OccupancyGrid original_map_;
//     bool map_received_ = false;
//     int inflation_radius_;

//     Gen_Starts_Goals planner_;
//     std::vector<std::vector<std::pair<Node, std::vector<int>>>> robot_corridors_;
// };

// #endif // PATH_PLANNER_H








































// /*在生成了路径之后，做简化路径，并且在两个corridor不连通时采用采用先插值后局部路径规划的方法确保联通性*/
// /*但是会出现递归不停止的现象*/
// #ifndef PATH_PLANNER_H
// #define PATH_PLANNER_H

// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <vector>
// #include <queue>
// #include <utility>
// #include <chrono>
// #include "Gen_Starts_Goals.h"
// #include <memory>

// struct Node {
//     int x, y;
//     int g, h;
//     Node* parent;

//     Node(int _x, int _y, int _g, int _h, Node* _parent = nullptr)
//         : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

//     int f() const { return g + h; }
// };

// struct CompareNode {
//     bool operator()(Node* a, Node* b) {
//         return a->f() > b->f();
//     }
// };

// class Path_Planner {
// public:
//     Path_Planner(ros::NodeHandle& nh);
//     ~Path_Planner() = default;

//     void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
//     void planPaths();

//     const std::vector<std::pair<Node, std::vector<int>>>& getCorridors(size_t robot_index) const;
//     const std::vector<std::pair<int, int>>& getStartPositions() const;
//     const std::vector<std::pair<int, int>>& getGoalPositions() const;

//     std::pair<int, int> getStart(size_t robot_index) const;
//     std::pair<int, int> getGoal(size_t robot_index) const;

//     bool mapReceived() const { return map_received_; }

// private:
//     int manhattanDistance(int x1, int y1, int x2, int y2);
//     bool isLineFree(int x1, int y1, int x2, int y2);
//     std::vector<Node> simplifyPath(const std::vector<Node>& path);
//     std::vector<Node> astar(int start_x, int start_y, int goal_x, int goal_y);
//     std::vector<std::pair<Node, std::vector<int>>> generateSafeCorridor(const std::vector<Node>& path);
//     bool areCorridorsConnected(const std::vector<int>& corridor1, const std::vector<int>& corridor2);
//     void ensureCorridorsConnected(std::vector<Node>& path, std::vector<std::pair<Node, std::vector<int>>>& corridors, int depth = 0);
//     bool tryInsertMidpoints(std::vector<Node>& path, size_t start_idx, size_t end_idx, int depth = 0);
//     std::vector<Node> findLocalPath(int start_x, int start_y, int goal_x, int goal_y);
//     void simplifyCorridors(std::vector<std::pair<Node, std::vector<int>>>& corridors);
//     void inflateObstacle(int goal_x, int goal_y, nav_msgs::OccupancyGrid& map);

//     ros::Subscriber map_sub_;
//     nav_msgs::OccupancyGrid map_data_;
//     nav_msgs::OccupancyGrid original_map_;
//     bool map_received_ = false;
//     int inflation_radius_;

//     Gen_Starts_Goals planner_;
//     std::vector<std::vector<std::pair<Node, std::vector<int>>>> robot_corridors_;
// };

// #endif // PATH_PLANNER_H




