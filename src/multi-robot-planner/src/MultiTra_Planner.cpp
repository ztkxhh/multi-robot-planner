#include "MultiTra_Planner.h"


// 计算曲线的包围盒
static void computeBoundingBox(const Beziercurve& curve, double& minX, double& maxX, double& minY, double& maxY) {
    minX = maxX = curve._points[0].first;
    minY = maxY = curve._points[0].second;
    for (const auto& point : curve._points) {
        minX = std::min(minX, point.first);
        maxX = std::max(maxX, point.first);
        minY = std::min(minY, point.second);
        maxY = std::max(maxY, point.second);
    }
}

// 检查两个包围盒是否在影响阈值内
static bool boundingBoxesClose(double& minX1, double& maxX1, double& minY1, double& maxY1,
                        double& minX2, double& maxX2, double& minY2, double& maxY2,
                        double& threshold) {
    return !(minX1 > maxX2 + threshold || maxX1 < minX2 - threshold ||
             minY1 > maxY2 + threshold || maxY1 < minY2 - threshold);
}





MultiTra_Planner::MultiTra_Planner(ros::NodeHandle& nh)
{
    path_planner = std::make_shared<Path_Planner>(nh);

     curves = path_planner->merged_curves;

    // 获取每个机器人的半径
    if (!nh.getParam("robot_radius", robot_radius)) {
        ROS_WARN("Failed to get robot_radius, using default value 0.5");
        robot_radius = 0.5; // 默认半径0.5米
    }

    // 获取影响因子
    if (!nh.getParam("influnce_factor", influnce_factor)) {
        ROS_WARN("Failed to get influence_threshold, using default value 1.0");
        influnce_factor = 1.0; // 默认阈值1.0米
    }

    InfluenceThreshold = 2.0 * influnce_factor * robot_radius;

    GuropSubstion();

    visualization_test(nh);
}


// 判断两条曲线是否在空间位置上相互影响
bool MultiTra_Planner::curvesInfluenceEachOther( const Beziercurve& a, const Beziercurve& b, double threshold)
{
    for(const std::pair<double, double>& p_a : a._points) {
        for(const std::pair<double, double>& p_b : b._points) {
            double dx = p_a.first - p_b.first;
            double dy = p_a.second - p_b.second;
            double dx2 = dx *dx;
            double dy2 = dy * dy;
            double threshold2 = threshold * threshold;
            if(dx2 + dy2 <= threshold2)
                return true;
        }
    }
    return false;
}


bool MultiTra_Planner::computeInfluenceType(const Beziercurve& a, const Beziercurve& b, int& idxA, int& idxB)
{
    double vx1 = a._x_fir[idxA];
    double vy1 = a._y_fir[idxA];
    double vx2 = b._x_fir[idxB];
    double vy2 = b._y_fir[idxB];

    if ((vx1 == 0 && vy1 == 0) || (vx2 == 0 && vy2 == 0)){
        ROS_WARN("Zero velocity vector detected, cannot compute influence type");
        return false;
    }

    double dotProduct = vx1 * vx2 + vy1 * vy2;

    if (dotProduct > 0)
    {
        return true; // acute
    }
    else
    {
        return false; // non-acute
    }
}






void MultiTra_Planner::processCurvePair(const Beziercurve& a, const Beziercurve& b, int& idxA, int& idxB, double& threshold) 
{
    InfluenceSegment seg;
    seg.curveAIndex = idxA;
    seg.curveBIndex = idxB;

    int nA = a._points.size();
    int nB = b._points.size();

    double threshold2 = threshold * threshold;

    std::vector<std::vector<bool>> influenceMatrix(nA, std::vector<bool>(nB, false));
    std::vector<std::vector<bool>> influenceMatrix2(nB, std::vector<bool>(nA, false));


    for (int i = 0; i < nA; ++i)
    {
        for (int j = 0; j < nB; ++j) {
            double dx = a._points[i].first - b._points[j].first;
            double dy = a._points[i].second - b._points[j].second;
            double dist2 = dx * dx + dy * dy;
            if (dist2 < threshold2) {
                influenceMatrix[i][j] = true;
                influenceMatrix2[j][i] = true;

            }
        }
    }


    std::vector<std::vector<InfluenceInfo>> AB = seg_processing(a, b, influenceMatrix);

    std::vector<std::vector<InfluenceInfo>> BA = seg_processing(b, a, influenceMatrix2);

    int sizeAB = AB.size();
    int sizeBA = BA.size();

    if (sizeAB != sizeBA)
    {
        ROS_WARN("Number of influence segments between curve %d and curve %d are not equal", idxA, idxB);
    }


    for (int i = 0; i < sizeAB; ++i)
    {

        bool seg_ab_type = AB[i][0].Infulencetype;
        int seg_ab_size = AB[i].size();
        int start_ab_idx_a = AB[i][0].indexA;
        int end_ab_idx_a = AB[i][seg_ab_size - 1].indexA;
        int start_ab_idx_b = AB[i][0].indexB;
        int end_ab_idx_b = AB[i][0].indexB;
        for (int j = 1; j < seg_ab_size; ++j)
        {
            start_ab_idx_b = min(start_ab_idx_b, AB[i][j].indexB);
            end_ab_idx_b = max(end_ab_idx_b, AB[i][j].indexB);
        }

        for (int k = 0; k < sizeBA; ++k)
        {
            bool seg_ba_type = BA[k][0].Infulencetype;
            int seg_ba_size = BA[k].size();
            int start_ba_idx_b = BA[k][0].indexA;
            int end_ba_idx_b = BA[k][seg_ba_size - 1].indexA;
            int start_ba_idx_a = BA[k][0].indexB;
            int end_ba_idx_a = BA[k][0].indexB;
            for (int j = 1; j < seg_ba_size; ++j)
            {
                start_ba_idx_a = min(start_ba_idx_a, BA[k][j].indexB);
                end_ba_idx_a = max(end_ba_idx_a, BA[k][j].indexB);
            }


            if (seg_ab_type == seg_ba_type )
            {
                bool overlap = (start_ab_idx_a <= end_ba_idx_a && end_ab_idx_a >= start_ba_idx_a && start_ab_idx_b <= end_ba_idx_b && end_ab_idx_b >= start_ba_idx_b);
                if (overlap)
                {
                    if (seg_ab_type == true) // acute
                    {
                        double cof_ab;
                        double cof_ba;
                        for (int m = 0; m < seg_ab_size; ++m)
                        {
                            cof_ab = 10.0;
                            // a_ahead_b
                            if (a._duration[AB[i][m].indexA] == 0)
                            {
                                ROS_WARN("Zero duration detected between curve %d and curve %d, cannot compute influence pair", idxA, idxB );
                                continue;
                            }
                            cof_ab = min(cof_ab, b._duration[AB[i][m].indexB] / a._duration[AB[i][m].indexA]);
                        }

                        for ( int n = 0; n < seg_ba_size; ++n)
                        {
                            cof_ba = 10.0;

                            if (b._duration[BA[k][n].indexB] == 0)
                            {
                                ROS_WARN("Zero duration detected between curve %d and curve %d, cannot compute influence pair", idxA, idxB );
                                continue;
                            }
                            // b_ahead_a
                            cof_ba = min(cof_ba, a._duration[BA[k][n].indexB] / b._duration[BA[k][n].indexA]);
                        }

                        influncepair pair;
                        pair.a_head_b = cof_ab;
                        pair.b_ahed_a = cof_ba;
                        seg.influencePairs.push_back(pair);
                    }
                    else // non-acute
                    {

                        double cof_ab = b._duration[AB[i][seg_ab_size-1].indexB] / a._duration[AB[i][seg_ab_size-1].indexA];

                        double cof_ba = a._duration[BA[k][seg_ba_size-1].indexB] / b._duration[BA[k][seg_ba_size-1].indexA];

                        influncepair pair;
                        pair.a_head_b = cof_ab;
                        pair.b_ahed_a = cof_ba;
                        seg.influencePairs.push_back(pair);
                    }
                }
            }
        }
    }


    influenceSegments.push_back(seg);



}




std::vector<std::vector<InfluenceInfo>> MultiTra_Planner::seg_processing(const Beziercurve& a, const Beziercurve& b, std::vector<std::vector<bool>>& influenceMatrix)
{

    int nA = a._points.size();
    int nB = b._points.size();


    // 收集a_ahead_b情况下所有相互影响的段及其影响类型,要求相互影响的分段中点在曲线a上是连续的
    std::vector<std::vector<InfluenceInfo>> influencePointsAB;
    influencePointsAB.reserve(nA);

    int i = 0;
    while (i < nA)
    {
        while (i < nA && std::none_of(influenceMatrix[i].begin(), influenceMatrix[i].end(), [](bool v) { return v; }))
        {
            ++i;
        }
        if (i >= nA)
            break;

        std::vector<int> firstidxB;

        int preidxB = -11;

        for (int j = 0; j < nB; ++j)
        {
            if (influenceMatrix[i][j])
            {
                int dif_j = j - preidxB;
                if (dif_j > 10)
                {
                    firstidxB.push_back(j);
                }
                preidxB = j;
            }
        }

        if (firstidxB.empty())
        {
            ++i;
            continue;
        }

        std::vector<InfluenceInfo> element;
        for (int k = 0; k < firstidxB.size(); ++k)
        {
            element.push_back({i, firstidxB[k], computeInfluenceType(a, b, i, firstidxB[k])});
        }
        influencePointsAB.push_back(element);


        ++i;
    }


    // 收集相互影响的段, 首先初始化一个段，然后遍历所有的影响点，如果与当前任意一个段相邻且影响类型相同，则合并，否则添加一个新的段并将当前影响点作为新段的起点
    std::vector<std::vector<InfluenceInfo>> influenceSegmentsAB;
    int sizeA = influencePointsAB.size();

    for (int i = 0 ; i< sizeA; ++i)
    {
        int sizeB = influencePointsAB[i].size();

        for (int j = 0; j < sizeB; ++j)
        {
            if (influenceSegmentsAB.empty())
            {
                std::vector<InfluenceInfo> segment;
                segment.push_back(influencePointsAB[i][j]);
                influenceSegmentsAB.push_back(segment);
                continue;
            }

            bool added = false;

            for (int k = 0; k < influenceSegmentsAB.size(); ++k)
            {

                int endA = influenceSegmentsAB[k].back().indexA;
                int endB = influenceSegmentsAB[k].back().indexB;
                bool currentType = influenceSegmentsAB[k].back().Infulencetype;

                int dis_B = std::abs(influencePointsAB[i][j].indexB - endB);
                if (influencePointsAB[i][j].Infulencetype == currentType && influencePointsAB[i][j].indexA == endA + 1 && dis_B <= 10)
                {
                    influenceSegmentsAB[k].push_back(influencePointsAB[i][j]);
                    added = true;
                    break;
                }
            }
            if(!added)
            {
                std::vector<InfluenceInfo> segment2;
                segment2.push_back(influencePointsAB[i][j]);
                influenceSegmentsAB.push_back(segment2);
            }
        }
    }


    // //输出打印influenceSegmentsAB用以调试
    // ROS_INFO("influenceSegmentsAB size: %lu", influenceSegmentsAB.size());
    // for (int i = 0; i < influenceSegmentsAB.size(); ++i)
    // {
    //     for (int j = 0; j < influenceSegmentsAB[i].size(); ++j)
    //     {
    //         std::cout << "indexA: " << influenceSegmentsAB[i][j].indexA << " indexB: " << influenceSegmentsAB[i][j].indexB << " Infulencetype: " << influenceSegmentsAB[i][j].Infulencetype << std::endl;
    //     }
    //     std::cout << "----------------" << std::endl;
    // }


    return influenceSegmentsAB;
}


void MultiTra_Planner::visualization_test(ros::NodeHandle& nh)
{



    int vis_hz = 10;
    double vis_dt = 1.0 / vis_hz;

    int num_robots = curves.size();
    double max_duration = 0.0;
    for (int i = 0; i < num_robots; ++i)
    {
        max_duration = std::max(max_duration, curves[i]->_duration.back());
    }


    std::vector<std::vector<geometry_msgs::Point>> trajectories;
    for (int i = 0; i < num_robots; ++i)
    {
        std::vector<geometry_msgs::Point> trajectory;
        double max_dur_i = curves[i]->_duration.back();
        for (double t = 0.0; t < max_duration; t += vis_dt)
        {

            geometry_msgs::Point p;
            if (t == 0.0)
            {
                p.x = curves[i]->_points[0].first;
                p.y = curves[i]->_points[0].second;
                p.z = 0;
            }

            if (t > max_dur_i)
            {
                p.x = curves[i]->_points.back().first;
                p.y = curves[i]->_points.back().second;
                p.z = 0;                break;
            }

            auto it = std::upper_bound(curves[i]->_duration.begin(), curves[i]->_duration.end(), t);
            int idx = std::distance(curves[i]->_duration.begin(), it);
            double dif = t - curves[i]->_duration[idx - 1];
            if (dif < 1e-6)
            {
                p.x = curves[i]->_points[idx - 1].first;
                p.y = curves[i]->_points[idx - 1].second;
                p.z = 0;
            }
            else
            {
                p.x = 0.5 * (curves[i]->_points[idx - 1].first + curves[i]->_points[idx].first);
                p.y = 0.5 * (curves[i]->_points[idx - 1].second + curves[i]->_points[idx].second);
                p.z = 0;
            }
            trajectory.push_back(p);
        }
        trajectories.push_back(trajectory);
    }



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
                marker.type = visualization_msgs::Marker::CUBE;
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

}




void MultiTra_Planner::GuropSubstion()
{

    int n = curves.size();

    // 计算所有曲线的包围盒
    std::vector<std::tuple<double, double, double, double>> boundingBoxes(n);
    for (int i = 0; i < n; ++i) {
        double minX, maxX, minY, maxY;
        computeBoundingBox(*curves[i], minX, maxX, minY, maxY);
        boundingBoxes[i] = std::make_tuple(minX, maxX, minY, maxY);
    }

    // 构建相互影响图
    std::vector<std::unordered_set<int>> InfluenceGraph(n);

    for (int i = 0; i < n; ++i) {
        double minX1, maxX1, minY1, maxY1;
        std::tie(minX1, maxX1, minY1, maxY1) = boundingBoxes[i];
        for (int j = i + 1; j < n; ++j) {
            double minX2, maxX2, minY2, maxY2;
            std::tie(minX2, maxX2, minY2, maxY2) = boundingBoxes[j];
            if (boundingBoxesClose(minX1, maxX1, minY1, maxY1, minX2, maxX2, minY2, maxY2, InfluenceThreshold)) {
                if (curvesInfluenceEachOther(*curves[i], *curves[j], InfluenceThreshold)) {
                    InfluenceGraph[i].insert(j);
                    InfluenceGraph[j].insert(i);
                }
            }
        }
    }

    // 使用并查集对曲线进行分组
    UnionFind uf(n);
    for (int i = 0; i < n; ++i) {
        for (int j : InfluenceGraph[i]) {
            uf.unite(i, j);
        }
    }

    // 收集分组
    std::unordered_map<int, std::vector<int>> groups;
    for (int i = 0; i < n; ++i) {
        groups[uf.find(i)].push_back(i);
    }


    // 输出打印groups用以调试
    for (const auto& groupEntry : groups) {
        const std::vector<int>& groupIndices = groupEntry.second;
        for (size_t i = 0; i < groupIndices.size(); ++i) {
            std::cout << groupIndices[i] << " ";
        }
        std::cout << std::endl;
    }


    // 处理每个分组
    for (const auto& groupEntry : groups) {
        const std::vector<int>& groupIndices = groupEntry.second;
        for (size_t i = 0; i < groupIndices.size(); ++i) {
            int idxA = groupIndices[i];
            for (size_t j = i + 1; j < groupIndices.size(); ++j) {
                int idxB = groupIndices[j];
                if (InfluenceGraph[idxA].find(idxB) != InfluenceGraph[idxA].end())
                {
                    processCurvePair(*curves[idxA], *curves[idxB], idxA, idxB, InfluenceThreshold);
                }
            }
        }
    }

    // 输出打印influenceSegments用以调试
    ROS_INFO("influenceSegments size: %lu", influenceSegments.size());
    for (int i = 0; i < influenceSegments.size(); ++i)
    {
        std::cout << "curveAIndex: " << influenceSegments[i].curveAIndex << " curveBIndex: " << influenceSegments[i].curveBIndex << std::endl;
        for (int j = 0; j < influenceSegments[i].influencePairs.size(); ++j)
        {
            std::cout << "a_head_b: " << influenceSegments[i].influencePairs[j].a_head_b << " b_ahed_a: " << influenceSegments[i].influencePairs[j].b_ahed_a << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    // 创建Marker消息的发布器
    auto start_time = std::chrono::high_resolution_clock::now();

    std::shared_ptr<MultiTra_Planner> MultiTraPlanner = std::make_shared<MultiTra_Planner>(nh);

    // 选择一个机器人，比如第一个机器人，发布其起止点以及走廊可视化
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    // 选择一个机器人，比如第一个机器人，发布其multiple_curves,即生成的曲线，nav_msgs::Path
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("visualization_path", 1);

    ros::Publisher mulity_pub = nh.advertise<visualization_msgs::Marker>("multi_car_marker", 10);


    ros::Rate rate(10);
    while (ros::ok() && (!MultiTraPlanner->path_planner->mapReceived() || !MultiTraPlanner->path_planner->doubleMapReceived()))
    {
        ros::spinOnce();
        rate.sleep();
    }


    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;
    ROS_INFO("Execution time: %.6f seconds", elapsed_time.count());

    // 通过画图的形式显示合并后的曲线特性
    // path_planner->plotting();

    // // 选择一个机器人，比如第一个机器人，发布其路径可视化
    // while (ros::ok())
    // {
    //     size_t robot_index = 0;
    //     MultiTraPlanner->path_planner->publishPathVisualization(robot_index, marker_pub, path_pub);
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    // 创建一个发布器的数组
    std::vector<ros::Publisher> path_pubs(MultiTraPlanner->path_planner->merged_curves.size());

    // 为每个曲线初始化一个发布器
    for (size_t i = 0; i <MultiTraPlanner-> path_planner->merged_curves.size(); i++)
    {
        std::stringstream pub_name;
        pub_name << "path_pub_" << i;
        path_pubs[i] = nh.advertise<nav_msgs::Path>(pub_name.str(), 1);
    }

    // 面向所有机器人，发布其路径可视化
    while (ros::ok())
    {
        MultiTraPlanner->path_planner->publishPathsVisualization(path_pubs, marker_pub);
        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}






    // for (int i = 0; i < sizeA; ++i)
    // {
    //     int sizeB = influencePointsAB[i].size();
    //     int startA = influencePointsAB[i][0].indexA;
    //     int startB = influencePointsAB[i][0].indexB;
    //     int endA = startA;
    //     int endB = startB;
    //     bool currentType = influencePointsAB[i][0].Infulencetype;

    //     for (int k = 1; k < sizeB; ++k)
    //     {
    //         int idxA = influencePointsAB[i][k].indexA;
    //         int idxB = influencePointsAB[i][k].indexB;
    //         bool type = influencePointsAB[i][k].Infulencetype;


    //         if (idxA == endA + 1 && idxB == endB + 1 && type == currentType)
    //         {
    //             endA = idxA;
    //             endB = idxB;
    //         }
    //         else
    //         {
    //             segments.push_back({idxA, idxB, startA, endA, startB, endB, currentType});
    //             startA = idxA;
    //             startB = idxB;
    //             endA = idxA;
    //             endB = idxB;
    //             currentType = type;
    //         }
    //     }
    //     segments.push_back({idxA, idxB, startA, endA, startB, endB, currentType});
    // }








// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "path_planner");
//     ros::NodeHandle nh;

//     auto start_time = std::chrono::high_resolution_clock::now();

//     std::shared_ptr<Path_Planner> path_planner = std::make_shared<Path_Planner>(nh);

//     // 选择一个机器人，比如第一个机器人，发布其起止点以及走廊可视化
//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
//     // 选择一个机器人，比如第一个机器人，发布其multiple_curves,即生成的曲线，nav_msgs::Path
//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("visualization_path", 1);

//     ros::Rate rate(10);
//     while (ros::ok() && (!path_planner->mapReceived() || !path_planner->doubleMapReceived()))
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }


//     auto end_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed_time = end_time - start_time;
//     ROS_INFO("Execution time: %.6f seconds", elapsed_time.count());

//     // 通过画图的形式显示合并后的曲线特性
//     // path_planner->plotting();

//     // 选择一个机器人，比如第一个机器人，发布其路径可视化
//     while (ros::ok())
//     {
//         size_t robot_index = 0;
//         path_planner->publishPathVisualization(robot_index, marker_pub, path_pub);
//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }
