#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include<com_fun.h>

class ResultPublisher {
public:
    ResultPublisher(ros::NodeHandle& _nh, std::shared_ptr<MultiTra_Planner> _MultiTraPlanner_obj)
            : nh(_nh), MultiTraPlanner_obj(std::move(_MultiTraPlanner_obj))
    {

        ROS_INFO("ResultPublisher constructor fInished and ready for publishing.");

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

        inter.resize(robot_count, std::vector<double>(3));


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


        curves = MultiTraPlanner_obj->path_planner->merged_curves;



        colBox_pub = nh.advertise<visualization_msgs::MarkerArray>("/robot_model", 1);
        colBox_pub2 = nh.advertise<visualization_msgs::MarkerArray>("/robot_model2", 1);

        // msgs_traj.resize(qn);

    }
    ~ResultPublisher() = default;

    void update(double & current_time)
    {

        for(int i = 0; i < robot_count; ++i)
        {
            auto it = std::lower_bound(curves[i]->_duration.begin(), curves[i]->_duration.end(), current_time);
            if (it == curves[i]->_duration.end())
            {
                inter[i][0] = curves[i]->_points.back().first;
                inter[i][1] = curves[i]->_points.back().second;
                inter[i][2] = curves[i]->_theta.back();
            }
            else
            {
                int idx = it - curves[i]->_duration.begin();
                inter[i][0] = curves[i]->_points[idx].first;
                inter[i][1] = curves[i]->_points[idx].second;
                inter[i][2] = curves[i]->_theta[idx];

            }
        }
    }



    void publish()
    {
        update_Robot();
        colBox_pub.publish(msgs_colBox);
        colBox_pub2.publish(msgs_colBox2);
    }

private:
    ros::NodeHandle nh;

    std::shared_ptr<MultiTra_Planner> MultiTraPlanner_obj;

    std::vector <std::shared_ptr<Beziercurve>> curves;

    int robot_count; // number of robots
    double robot_radius; // robot radius

    // ROS publisher
    ros::Publisher colBox_pub;
    ros::Publisher colBox_pub2;

    // ROS messages
    visualization_msgs::MarkerArray msgs_colBox;
    visualization_msgs::MarkerArray msgs_colBox2;


    std::vector<std::vector<double>> inter;
    std::vector<std::vector<double>> c_matrix;



    // Robot Model
    void update_Robot()
    {
        visualization_msgs::MarkerArray mk_array, mk_array2;


        for (int r_i = 0; r_i < robot_count; r_i++) {

            for (int i=0;i<2;i++){
                visualization_msgs::Marker mk;
                mk.header.frame_id = "map";
                mk.ns = "Robot";
                if (i==0)
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

                mk.pose.orientation = Euler_to_Quat (0,0,inter[r_i][2]);

                mk.color.a = 0.7;
                mk.color.r = c_matrix[r_i][0];
                mk.color.g = c_matrix[r_i][1];
                mk.color.b = c_matrix[r_i][2];

                if (i==0)
                    mk_array.markers.emplace_back(mk);
                else
                    mk_array2.markers.emplace_back(mk);

            }

        }
        msgs_colBox = mk_array;
        msgs_colBox2 = mk_array2;
    }
};
