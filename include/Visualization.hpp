#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <random>
#include <set>
#include <vector>
#include <tf/LinearMath/Quaternion.h>

class ResultPublisher {
public:
    ResultPublisher(ros::NodeHandle& _nh, std::shared_ptr<MultiTra_Planner> _MultiTraPlanner_obj)
            : nh(_nh), MultiTraPlanner_obj(std::move(_MultiTraPlanner_obj))
    {
        // ROS_INFO("ResultPublisher constructor finished and ready for publishing.");

        if (!nh.getParam("robot_radius", robot_radius))
        {
            ROS_ERROR("Failed to get param 'robot_radius'");
            robot_radius = 0.5;
        }
        if (!nh.getParam("robot_count", robot_count))
        {
            ROS_WARN("Failed to get robot_count, using default value 1");
            robot_count = 1; // 默认一个机器人
        }

        c_matrix.resize(robot_count, std::vector<double>(3));
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        // 使用一个集合来检测颜色是否重复
        std::set<std::vector<double>> color_set;
        for (int i = 0; i < robot_count; ++i) {
            std::vector<double> color(3);
            do {
                // 生成随机颜色
                color[0] = dis(gen); // R
                color[1] = dis(gen); // G
                color[2] = dis(gen); // B
            } while (color_set.find(color) != color_set.end()); // 确保颜色不同

            // 将颜色添加到集合和结果中
            color_set.insert(color);
            c_matrix[i] = color;
        }

        inter.resize(robot_count, std::vector<double>(3));
        prev_quaternions.resize(robot_count);
        for (int i = 0; i < robot_count; ++i) {
            prev_quaternions[i].w = 1.0;  // 初始化为单位四元数
        }

        traj_pubs.resize(robot_count);
        msgs_traj.resize(robot_count);

        curves = MultiTraPlanner_obj->path_planner->merged_curves;

        max_duration = 0.0;
        for (int i = 0; i < robot_count; ++i)
        {
            max_duration = std::max(max_duration, curves[i]->_duration.back());
        }

        // ROS_INFO("max_duration: %f", max_duration);
        std::cout << "\033[1m\033[32m Makespan:  \033[0m" << "\033[1m\033[32m" << max_duration << " s." << "\033[0m" << std::endl;

        // trajectory
        for(int qi = 0; qi < robot_count; qi++){
            std::string rob_name = "/robot" + std::to_string(qi);
            traj_pubs[qi] = nh.advertise<nav_msgs::Path>("/trajectory" + rob_name, 1);
        }

        // robot_model
        colBox_pub = nh.advertise<visualization_msgs::MarkerArray>("/robot_model", 1);
        colBox_pub2 = nh.advertise<visualization_msgs::MarkerArray>("/robot_model2", 1);
    }
    ~ResultPublisher() = default;

    void update(double & current_time)
    {
        // update the robot position
        for(int i = 0; i < robot_count; ++i)
        {
            auto it = std::lower_bound(curves[i]->_duration.begin(), curves[i]->_duration.end(), current_time);
            if (it != curves[i]->_duration.end())
            {
                int idx = it - curves[i]->_duration.begin();
                if (idx == 0)
                {
                    inter[i][0] = curves[i]->_points[0].first;
                    inter[i][1] = curves[i]->_points[0].second;
                    inter[i][2] = curves[i]->_theta[0];
                }
                else
                {
                    double t = (current_time - curves[i]->_duration[idx - 1]) / (curves[i]->_duration[idx] - curves[i]->_duration[idx - 1]);
                    inter[i][0] = curves[i]->_points[idx - 1].first + t * (curves[i]->_points[idx].first - curves[i]->_points[idx - 1].first);
                    inter[i][1] = curves[i]->_points[idx - 1].second + t * (curves[i]->_points[idx].second - curves[i]->_points[idx - 1].second);
                    inter[i][2] = curves[i]->_theta[idx - 1] + t * (curves[i]->_theta[idx] - curves[i]->_theta[idx - 1]);
                }
            }
        }

        // update trajectory
        update_traj(current_time);
    }

    void publish()
    {
        update_Robot();

        for(int qi = 0; qi < robot_count; qi++){
            traj_pubs[qi].publish(msgs_traj[qi]);
        }

        colBox_pub.publish(msgs_colBox);
        colBox_pub2.publish(msgs_colBox2);
    }

private:
    ros::NodeHandle nh;

    double max_duration;

    std::shared_ptr<MultiTra_Planner> MultiTraPlanner_obj;

    std::vector <std::shared_ptr<Beziercurve>> curves;

    int robot_count; // number of robots
    double robot_radius; // robot radius

    // ROS publisher
    ros::Publisher colBox_pub;
    ros::Publisher colBox_pub2;
    std::vector<ros::Publisher> traj_pubs;

    // ROS messages
    visualization_msgs::MarkerArray msgs_colBox;
    visualization_msgs::MarkerArray msgs_colBox2;
    std::vector<nav_msgs::Path> msgs_traj;

    std::vector<std::vector<double>> inter;
    std::vector<std::vector<double>> c_matrix;
    std::vector<geometry_msgs::Quaternion> prev_quaternions;

    void update_traj(double& current_time)
    {
        if (current_time > max_duration)
        {
            return;
        }
        for(int qi = 0; qi < robot_count; qi++) 
        {
            msgs_traj[qi].header.frame_id = "/map";
            msgs_traj[qi].header.stamp.sec = current_time;

            geometry_msgs::PoseStamped pos_des;
            pos_des.header.frame_id = "/map";
            pos_des.pose.position.x = inter[qi][0];
            pos_des.pose.position.y = inter[qi][1];
            pos_des.pose.position.z = 0;
            pos_des.pose.orientation = Euler_to_Quat(0, 0, inter[qi][2]);
            msgs_traj[qi].poses.emplace_back(pos_des);
        }
    }

    // 将欧拉角转四元数
    geometry_msgs::Quaternion Euler_to_Quat(double roll, double pitch, double yaw)
    {
        tf::Quaternion tf_quat;
        geometry_msgs::Quaternion msg_quat;
        tf_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
        tf::quaternionTFToMsg(tf_quat, msg_quat);
        return msg_quat;
    }

    // 使用 SLERP 平滑插值四元数
    geometry_msgs::Quaternion Slerp_Quaternion(const geometry_msgs::Quaternion& prev_quat, const geometry_msgs::Quaternion& curr_quat, double t) 
    {
        tf::Quaternion q1(prev_quat.x, prev_quat.y, prev_quat.z, prev_quat.w);
        tf::Quaternion q2(curr_quat.x, curr_quat.y, curr_quat.z, curr_quat.w);
        tf::Quaternion slerped = q1.slerp(q2, t);
        geometry_msgs::Quaternion result;
        tf::quaternionTFToMsg(slerped, result);
        return result;
    }

    // 检查并修正四元数的符号以保持连续性
    geometry_msgs::Quaternion Ensure_Quaternion_Continuity(const geometry_msgs::Quaternion& prev_quat, const geometry_msgs::Quaternion& curr_quat)
    {
        // 计算前一个四元数和当前四元数的点积
        double dot_product = prev_quat.x * curr_quat.x + prev_quat.y * curr_quat.y + prev_quat.z * curr_quat.z + prev_quat.w * curr_quat.w;

        // 如果点积为负，则取相反的四元数
        geometry_msgs::Quaternion result = curr_quat;
        if (dot_product < 0.0)
        {
            result.x = -curr_quat.x;
            result.y = -curr_quat.y;
            result.z = -curr_quat.z;
            result.w = -curr_quat.w;
        }
        return result;
    }

    // Robot Model
    void update_Robot()
    {
        visualization_msgs::MarkerArray mk_array, mk_array2;

        for (int r_i = 0; r_i < robot_count; r_i++) {
            geometry_msgs::Quaternion curr_quat = Euler_to_Quat(0, 0, inter[r_i][2]);

            // 使用 Ensure_Quaternion_Continuity 确保四元数的连续性
            geometry_msgs::Quaternion adjusted_quat = Ensure_Quaternion_Continuity(prev_quaternions[r_i], curr_quat);

            // 使用 SLERP 平滑插值四元数
            adjusted_quat = Slerp_Quaternion(prev_quaternions[r_i], adjusted_quat, 0.1); // t 可以根据需要调整，通常在 0 到 1 之间

            // 更新 prev_quaternions
            prev_quaternions[r_i] = adjusted_quat;

            for (int i = 0; i < 2; i++){
                visualization_msgs::Marker mk;
                mk.header.frame_id = "map";
                mk.ns = "Robot";
                if (i == 0)
                {
                    mk.type = visualization_msgs::Marker::MESH_RESOURCE;
                    mk.mesh_resource = std::string("package://multi-robot-planner/chassis.dae");

                    mk.scale.x = robot_radius * 2.0;
                    mk.scale.y = robot_radius * 2.0;
                    mk.scale.z = robot_radius * 2.0;

                }
                else{
                    mk.type = visualization_msgs::Marker::ARROW;
                    mk.scale.x = 1.6 ;
                    mk.scale.y = 0.9 ;
                    mk.scale.z = 0.9 ;
                }
                mk.action = visualization_msgs::Marker::ADD;

                mk.id = r_i;
                mk.pose.position.x = inter[r_i][0];
                mk.pose.position.y = inter[r_i][1];
                mk.pose.position.z = 0;

                // 使用经过修正和平滑的四元数
                mk.pose.orientation = adjusted_quat;

                mk.color.a = 0.7;
                mk.color.r = c_matrix[r_i][0];
                mk.color.g = c_matrix[r_i][1];
                mk.color.b = c_matrix[r_i][2];

                if (i == 0)
                    mk_array.markers.emplace_back(mk);
                else
                    mk_array2.markers.emplace_back(mk);
            }
        }
        msgs_colBox = mk_array;
        msgs_colBox2 = mk_array2;
    }
};