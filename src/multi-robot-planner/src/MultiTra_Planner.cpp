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

    InfluenceThreshold = 2.0 * robot_radius;

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

    if ((vx1 == 0 && vy1 == 0) || (vx2 == 0 && vy2 == 0))
        return false;
    
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


void MultiTra_Planner::processCurvePair(const Beziercurve& a, const Beziercurve& b, int& idxA, int& idxB, double& threshold, std::vector<InfluenceSegment>& segments) 
{
    int nA = a._points.size();
    int nB = b._points.size();

    double threshold2 = threshold * threshold;

    // 收集a_ahead_b情况下所有相互影响的点对及其影响类型
    std::vector<InfluenceInfo> influencePointsAB;
    influencePointsAB.reserve(nA);
    for (int i = 0; i < nA; ++i) {
        for (int j = 0; j < nB; ++j) {
            double dx = a._points[i].first - b._points[j].first;
            double dy = a._points[i].second - b._points[j].second;
            double dist2 = dx * dx + dy * dy;
            if (dist2 < threshold2) {
                bool influenceType = computeInfluenceType(a, b, i, j);
                influencePointsAB.push_back({i, j, influenceType});
            }
        }
    }

    // // 对相互影响的点对进行分段
    // if (influencePointsAB.empty())
    //     return;

    // // 初始化第一个段
    // size_t startA = influencePointsAB[0].indexA;
    // size_t startB = influencePointsAB[0].indexB;
    // size_t endA = startA;
    // size_t endB = startB;
    // bool currentType = influencePointsAB[0].Infulencetype;

    // for (size_t k = 1; k < influencePointsAB.size(); ++k) {
    //     size_t idxA = influencePointsAB[k].indexA;
    //     size_t idxB = influencePointsAB[k].indexB;
    //     bool type = influencePointsAB[k].Infulencetype;

    //     // 判断是否与当前段连续
    //     if (idxA == endA + 1 && idxB == endB + 1 && type == currentType) {
    //         // 更新结束索引
    //         endA = idxA;
    //         endB = idxB;
    //     } else {
    //         // 保存当前段
    //         segments.push_back({idxA, idxB, startA, endA, startB, endB, currentType});
    //         // 开始新的段
    //         startA = idxA;
    //         startB = idxB;
    //         endA = idxA;
    //         endB = idxB;
    //         currentType = type;
    //     }
    // }
    // // 保存最后一个段
    // segments.push_back({idxA, idxB, startA, endA, startB, endB, currentType});
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
    std::vector<std::vector<int>> InfluenceGraph(n);

    for (int i = 0; i < n; ++i) {
        double minX1, maxX1, minY1, maxY1;
        std::tie(minX1, maxX1, minY1, maxY1) = boundingBoxes[i];
        for (int j = i + 1; j < n; ++j) {
            double minX2, maxX2, minY2, maxY2;
            std::tie(minX2, maxX2, minY2, maxY2) = boundingBoxes[j];
            if (boundingBoxesClose(minX1, maxX1, minY1, maxY1, minX2, maxX2, minY2, maxY2, InfluenceThreshold)) {
                if (curvesInfluenceEachOther(*curves[i], *curves[j], InfluenceThreshold)) {
                    InfluenceGraph[i].push_back(j);
                    InfluenceGraph[j].push_back(i);
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


    // 处理每个分组
    std::vector<InfluenceSegment> influenceSegments;
    for (const auto& groupEntry : groups) {
        const std::vector<int>& groupIndices = groupEntry.second;
        for (size_t i = 0; i < groupIndices.size(); ++i) {
            int idxA = groupIndices[i];
            for (size_t j = i + 1; j < groupIndices.size(); ++j) {
                int idxB = groupIndices[j];
                if (std::find(InfluenceGraph[idxA].begin(), InfluenceGraph[idxA].end(), idxB) != InfluenceGraph[idxA].end()) {
                    processCurvePair(*curves[idxA], *curves[idxB], idxA, idxB, InfluenceThreshold, influenceSegments);
                }
            }
        }
    }



}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    auto start_time = std::chrono::high_resolution_clock::now();

    std::shared_ptr<MultiTra_Planner> MultiTraPlanner = std::make_shared<MultiTra_Planner>(nh);

    // 选择一个机器人，比如第一个机器人，发布其起止点以及走廊可视化
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    // 选择一个机器人，比如第一个机器人，发布其multiple_curves,即生成的曲线，nav_msgs::Path
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("visualization_path", 1);

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

    // 选择一个机器人，比如第一个机器人，发布其路径可视化
    while (ros::ok())
    {
        size_t robot_index = 0;
        MultiTraPlanner->path_planner->publishPathVisualization(robot_index, marker_pub, path_pub);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



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
