#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>

int main(int argc, char** argv)
{

    ROS_INFO("Starting multi_car_trajectory_publisher node...");
    // 初始化ROS节点
    ros::init(argc, argv, "multi_car_trajectory_publisher");
    ros::NodeHandle nh;
    
    // 创建Marker消息的发布器
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("multi_car_marker", 10);

    // 定义多个小车的轨迹数据
    std::vector<std::vector<geometry_msgs::Point>> trajectories;

    // 示例：两个小车的轨迹
    // 小车1的轨迹
    std::vector<geometry_msgs::Point> trajectory1;
    geometry_msgs::Point p;
    p.x = 0; p.y = 0; p.z = 0;
    trajectory1.push_back(p);
    p.x = 1; p.y = 1; p.z = 0;
    trajectory1.push_back(p);
    p.x = 2; p.y = 2; p.z = 0;
    trajectory1.push_back(p);

    // 小车2的轨迹
    std::vector<geometry_msgs::Point> trajectory2;
    p.x = 0; p.y = 2; p.z = 0;
    trajectory2.push_back(p);
    p.x = 1; p.y = 1; p.z = 0;
    trajectory2.push_back(p);
    p.x = 2; p.y = 0; p.z = 0;
    trajectory2.push_back(p);
    p.x = 0; p.y = 2; p.z = 0;
    trajectory2.push_back(p);
    p.x = 1; p.y = 1; p.z = 0;
    trajectory2.push_back(p);
    p.x = 2; p.y = 0; p.z = 0;
    trajectory2.push_back(p);
    // 将轨迹添加到总的轨迹列表中
    trajectories.push_back(trajectory1);
    trajectories.push_back(trajectory2);

    // 获取最大轨迹长度，以确定循环次数
    size_t max_length = 0;
    for (const auto& traj : trajectories)
    {
        if (traj.size() > max_length)
            max_length = traj.size();
    }

    // 设置发布频率
    ros::Rate r(10); // 1 Hz

    size_t step = 0;
    while (ros::ok() && step < max_length)
    {
        for (size_t car_id = 0; car_id < trajectories.size(); ++car_id)
        {
            const auto& traj = trajectories[car_id];

            // 检查当前小车是否还有未发布的轨迹点
            if (step < traj.size())
            {
                visualization_msgs::Marker marker;

                // 设置Marker的基本信息
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();

                // 使用不同的命名空间和ID来区分不同的小车
                marker.ns = "car_" + std::to_string(car_id);
                marker.id = car_id;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;

                // 设置小车的位置
                marker.pose.position.x = traj[step].x;
                marker.pose.position.y = traj[step].y;
                marker.pose.position.z = traj[step].z;

                // 设置小车的朝向
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                // 设置小车的尺寸
                marker.scale.x = 0.5;
                marker.scale.y = 0.3;
                marker.scale.z = 0.2;

                // 设置小车的颜色，给不同的小车不同的颜色
                marker.color.a = 1.0;
                if (car_id == 0)
                {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0; // 红色
                }
                else if (car_id == 1)
                {
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0; // 蓝色
                }
                // 可以为更多的小车设置不同的颜色

                // 发布Marker消息
                marker_pub.publish(marker);
            }
        }

        // 等待下一个循环
        ros::spinOnce();
        r.sleep();
        ++step;
    }

    return 0;
}