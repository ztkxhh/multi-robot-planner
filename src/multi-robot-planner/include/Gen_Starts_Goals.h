#ifndef GEN_STARTS_GOALS_H
#define GEN_STARTS_GOALS_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

class Gen_Starts_Goals {
public:
    Gen_Starts_Goals(ros::NodeHandle& nh);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void generateStartAndGoalPositions();
    const std::vector<std::pair<int, int>>& getStartPositions() const;
    const std::vector<std::pair<int, int>>& getGoalPositions() const;

private:
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    int robot_count;
    double robot_radius;
    int width, height;
    double resolution;
    int radius_in_cells;
    bool map_received;

    nav_msgs::OccupancyGrid last_received_map;
    std::vector<std::pair<int, int>> start_positions;
    std::vector<std::pair<int, int>> goal_positions;

    bool isCollisionWithOtherRobots(int x, int y, bool checkStartPositions);
    bool isValidPosition(int x, int y);
    void outputStartAndGoalPositions();
    bool checkStartsandGoals();
};

#endif // GEN_STARTS_GOALS_H