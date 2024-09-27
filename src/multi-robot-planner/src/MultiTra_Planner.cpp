#include "MultiTra_Planner.h"

double inf = std::numeric_limits<double>::infinity();


void MILP_Test()
{
    try
    {
        // Create an Gurobi environment
        GRBEnv env = GRBEnv(true);
        //禁止打印输出信息
        env.set("OutputFlag", "0");

        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);

        model.getEnv().set(GRB_IntParam_Threads, 1);

        // Create variables
        GRBVar x = model.addVar(1.0, 100, 0.0, GRB_CONTINUOUS, "x");
        GRBVar y = model.addVar(1.0, 100, 0.0, GRB_CONTINUOUS, "y");
        GRBVar z1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z1");
        GRBVar z2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z2");
        // GRBVar z3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z3");
        // GRBVar z4 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z4");



        // Set cost function
        GRBLinExpr objective = x + y ;
        model.setObjective(objective, GRB_MINIMIZE);

// [ INFO] [1727423439.921922818]: idxA: 27 and 28
// [ INFO] [1727423439.921933486]: seg_ab_type: 0
// [ INFO] [1727423439.921941384]: a_head_b: 0.182790
// [ INFO] [1727423439.921948089]: b_ahed_a: 4.787765
// [ INFO] [1727423439.921954436]: a_b_starta: 283   and   a_b_enda: 283
// [ INFO] [1727423439.921960854]: a_b_startb: 20   and   a_b_endb: 20
// [ INFO] [1727423439.921967264]: b_a_startb: 0   and   b_a_endb: 33
// [ INFO] [1727423439.921973657]: b_a_starta: 283   and   b_a_enda: 285
// --------------
// --------------
// [ INFO] [1727423439.921984436]: idxA: 27 and 28
// [ INFO] [1727423439.921994388]: seg_ab_type: 0
// [ INFO] [1727423439.922002356]: a_head_b: 0.000000
// [ INFO] [1727423439.922008935]: b_ahed_a: 4.787765
// [ INFO] [1727423439.922017239]: a_b_starta: 284   and   a_b_enda: 309
// [ INFO] [1727423439.922025013]: a_b_startb: 0   and   a_b_endb: 6
// [ INFO] [1727423439.922032710]: b_a_startb: 0   and   b_a_endb: 33
// [ INFO] [1727423439.922039416]: b_a_starta: 283   and   b_a_enda: 285

        int M = 1000; // A large number for binary constraints
        double epsilon = 1e-5; // A small number for numerical stability
        double a_head_b_1 = 0.182790;
        double b_ahed_a_1 =4.787765;
        double a_head_b_2 = 0.000000;
        double b_ahed_a_2 = 4.787765;
        // double a_head_b_3 = 0.475358;
        // double b_ahed_a_3 = 1.974138;
        // double a_head_b_4 = 0.655340;
        // double b_ahed_a_4 = 1.415259;
        // Add constraints
        model.addConstr(x  <= y * a_head_b_1 + M * z1 - epsilon);
        model.addConstr(y  <= x * b_ahed_a_1 + M * (1 - z1) - epsilon);
        model.addConstr(x  <= y * a_head_b_2 + M * z2 - epsilon);
        model.addConstr(y  <= x * b_ahed_a_2 + M * (1 - z2) - epsilon);
        // model.addConstr(x  <= y * a_head_b_3 + M * z3 - epsilon);
        // model.addConstr(y  <= x * b_ahed_a_3 + M * (1 - z3) - epsilon);
        // model.addConstr(x  <= y * a_head_b_4 + M * z4 - epsilon);
        // model.addConstr(y  <= x * b_ahed_a_4 + M * (1 - z4) - epsilon);


        // Optimize model
        model.optimize();

        std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
        std::cout << x.get(GRB_DoubleAttr_X) << std::endl;
        std::cout << y.get(GRB_DoubleAttr_X) << std::endl;
        std::cout << z1.get(GRB_DoubleAttr_X) << std::endl;
        std::cout << z2.get(GRB_DoubleAttr_X) << std::endl;
        // std::cout << z3.get(GRB_DoubleAttr_X) << std::endl;
        // std::cout << z4.get(GRB_DoubleAttr_X) << std::endl;
    }
    catch (GRBException e)
    {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }
    catch (...)
    {
        std::cout << "Exception during optimization" << std::endl;
    }
}






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
    path_planner = std::make_unique<Path_Planner>(nh);

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
    double epsilon = 1e-4;
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

    // if (sizeAB != sizeBA)
    // {
    //     ROS_WARN("Number of influence segments between curve %d and curve %d are not equal", idxA, idxB);
    // }


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
                bool overlap = start_ab_idx_a <= end_ba_idx_a && end_ab_idx_a >= start_ba_idx_a && start_ab_idx_b <= end_ba_idx_b && end_ab_idx_b >= start_ba_idx_b;
                if (overlap)
                {
                    double cof_ab = 5.0;
                    double cof_ba = 5.0;

                    if (seg_ab_type == true) // acute
                    {
                        for (int m = 0; m < seg_ab_size; ++m)
                        {
                            // a_ahead_b
                            if (a._duration[AB[i][m].indexA]<= epsilon)
                            {
                                // ROS_WARN("Zero duration detected between curve %d and curve %d, cannot compute influence pair", idxA, idxB );
                                continue;
                            }
                            cof_ab = std::min(cof_ab, b._duration[AB[i][m].indexB] / a._duration[AB[i][m].indexA]);
                        }

                        for ( int n = 0; n < seg_ba_size; ++n)
                        {
                            if (b._duration[BA[k][n].indexB]  <= epsilon)
                            {
                                // ROS_WARN("Zero duration detected between curve %d and curve %d, cannot compute influence pair", idxA, idxB );
                                continue;
                            }
                            // b_ahead_a
                            cof_ba = std::min(cof_ba, a._duration[BA[k][n].indexB] / b._duration[BA[k][n].indexA]);
                        }
                    }
                    else // non-acute
                    {
                        if (a._duration[AB[i][seg_ab_size-1].indexA] <= epsilon)
                        {
                            cof_ab = 2.0;
                        }
                        else
                        {
                        cof_ab = b._duration[AB[i][seg_ab_size-1].indexB] / a._duration[AB[i][seg_ab_size-1].indexA];
                        }
                        if (b._duration[BA[k][seg_ba_size-1].indexA] <= epsilon)
                        {
                            cof_ba = 2.0;
                        }
                        else
                        {
                        cof_ba = a._duration[BA[k][seg_ba_size-1].indexB] / b._duration[BA[k][seg_ba_size-1].indexA];
                        }
                    }

                    influncepair pair;
                    pair.a_head_b = cof_ab;
                    pair.b_ahed_a = cof_ba;
                    // pair.a_b_starta = (start_ab_idx_a == 0);
                    // pair.a_b_enda = (end_ab_idx_a == nA - 1);
                    // pair.b_a_startb = (start_ba_idx_b == 0);
                    // pair.b_a_endb = (end_ba_idx_b == nB - 1);

                    seg.influencePairs.push_back(pair);


                    // // print for debug
                    // std::cout<<"--------------"<<std::endl;
                    // ROS_INFO("idxA: %d and %d", idxA, idxB);
                    // ROS_INFO("seg_ab_type: %d", seg_ab_type);
                    // ROS_INFO("a_head_b: %f", pair.a_head_b);
                    // ROS_INFO("b_ahed_a: %f", pair.b_ahed_a);
                    // ROS_INFO("a_b_starta: %d   and   a_b_enda: %d", start_ab_idx_a, end_ab_idx_a);
                    // ROS_INFO("a_b_startb: %d   and   a_b_endb: %d", start_ab_idx_b, end_ab_idx_b);
                    // ROS_INFO("b_a_startb: %d   and   b_a_endb: %d", start_ba_idx_b, end_ba_idx_b);
                    // ROS_INFO("b_a_starta: %d   and   b_a_enda: %d", start_ba_idx_a, end_ba_idx_a);
                    // std::cout<<"--------------"<<std::endl;




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


void MultiTra_Planner::visualization_test(ros::Publisher &marker_pub)
{

    int vis_hz = 50;
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
                p.z = 0;
                break;
            }

            auto it = std::upper_bound(curves[i]->_duration.begin(), curves[i]->_duration.end(), t);
            int idx = std::distance(curves[i]->_duration.begin(), it);
            double dif = t - curves[i]->_duration[idx - 1];
            if (dif < 1e-6)
            {
                p.x = curves[i]->_points[idx - 1].first;
                p.y = curves[i]->_points[idx - 1].second;
                p.z = 0.05;
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
    ros::Rate rate(vis_hz);

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
                marker.scale.x = robot_radius;
                marker.scale.y = robot_radius;
                marker.scale.z = robot_radius;

                // 设置小车的颜色，不同的小车颜色不同
                marker.color.a = 1.0;
                marker.color.r = 1.0 - 0.1 * car_id;
                marker.color.g = 0.5;
                marker.color.b = 0.1 * car_id;

                // 发布Marker消息
                marker_pub.publish(marker);
            }
        }

        // 等待下一个循环
        ros::spinOnce();
        rate.sleep();
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
        std::cout << "Group: ";
        for (size_t i = 0; i < groupIndices.size(); ++i) {
            std::cout << groupIndices[i] << " ";
        }
        std::cout << std::endl;
        std::cout << "----------------" << std::endl;

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


    // // 输出打印influenceSegments用以调试
    // ROS_INFO("influenceSegments size: %lu", influenceSegments.size());

    // for (int i = 0; i < influenceSegments.size(); ++i)
    // {
    //     std::cout << "curveAIndex: " << influenceSegments[i].curveAIndex << " curveBIndex: " << influenceSegments[i].curveBIndex << std::endl;
    //     for (int j = 0; j < influenceSegments[i].influencePairs.size(); ++j)
    //     {
    //         std::cout << "a_head_b: " << influenceSegments[i].influencePairs[j].a_head_b << " b_ahed_a: " << influenceSegments[i].influencePairs[j].b_ahed_a << std::endl;
    //         std::cout << "a_b_starta: " << influenceSegments[i].influencePairs[j].a_b_starta << " a_b_enda: " << influenceSegments[i].influencePairs[j].a_b_enda << std::endl;
    //         std::cout << "b_a_startb: " << influenceSegments[i].influencePairs[j].b_a_startb << " b_a_endb: " << influenceSegments[i].influencePairs[j].b_a_endb << std::endl;
    //         std::cout << "----------------" << std::endl;
    //     }
    // }

    MILP_Adujust();

}

















void MultiTra_Planner::MILP_Adujust()
{
    std::unordered_set<int> curves_idxs_set;
    int seg_size = influenceSegments.size();
    curves_idxs_set.reserve(seg_size * 2);

    int binary_num = 0;

    for (int i = 0; i < seg_size; ++i)
    {
        // 查看每个影响段的两个曲线索引，如果曲线索引不在curves_idxs_set中，则添加
        curves_idxs_set.insert(influenceSegments[i].curveAIndex);
        curves_idxs_set.insert(influenceSegments[i].curveBIndex);

        binary_num += influenceSegments[i].influencePairs.size();
    }

    // 如果需要，可以将 unordered_set 转换回 vector
    std::vector<int> curves_idxs(curves_idxs_set.begin(), curves_idxs_set.end());


    try
    {
        // Create an Gurobi environment
        GRBEnv env = GRBEnv(true);
        //禁止打印输出信息
        env.set("OutputFlag", "0");

        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);

        model.getEnv().set(GRB_IntParam_Threads, 10);

        int num_curves = curves_idxs.size();
        // Create variables. first curves_idxs.size() are sacling factors for all influenced curves
        // and the rest are binary variables for each influence pair
        std::vector<GRBVar> vars(num_curves+ binary_num);
        // Create scaling factors for each curve
        for (int i = 0; i < num_curves; ++i)
        {
            vars[i] = model.addVar(1.0, 100, 0.0, GRB_CONTINUOUS);
        }
        // Create binary variables for each influence pair
        for (int i = num_curves; i < num_curves + binary_num; ++i)
        {
            vars[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }

        // Set cost function
        GRBLinExpr objective = 0.0;
        for (int i = 0; i < num_curves; ++i)
        {
            objective += vars[i];
        }
        model.setObjective(objective, GRB_MINIMIZE);

        // Add constraints
        int M = 1000; // A large number for binary constraints
        double epsilon = 1e-5; // A small number for numerical stability
        int idx_binary = 0;
        for (int i = 0; i < seg_size; ++i)
        {
            int idx_A = influenceSegments[i].curveAIndex;
            int idx_B = influenceSegments[i].curveBIndex;

            auto it_A = std::find(curves_idxs.begin(), curves_idxs.end(), idx_A);
            auto it_B = std::find(curves_idxs.begin(), curves_idxs.end(), idx_B);
            int idx_A_vars;
            int idx_B_vars;

            if (it_A != curves_idxs.end())
            {
                idx_A_vars = std::distance(curves_idxs.begin(), it_A);
            }
            else
            {
                ROS_WARN("Curve index %d not found in curves_idxs", idx_A);
            }

            if (it_B != curves_idxs.end())
            {
                idx_B_vars = std::distance(curves_idxs.begin(), it_B);
            }
            else
            {
                ROS_WARN("Curve index %d not found in curves_idxs", idx_B);
            }

            for (int j = 0; j < influenceSegments[i].influencePairs.size(); ++j)
            {
                double a_head_b = influenceSegments[i].influencePairs[j].a_head_b;
                double b_ahed_a = influenceSegments[i].influencePairs[j].b_ahed_a;


                model.addConstr(vars[idx_A_vars] <= vars[idx_B_vars] * a_head_b + M * vars[num_curves + idx_binary] - epsilon);
                model.addConstr(vars[idx_B_vars] <= vars[idx_A_vars] * b_ahed_a + M * (1 - vars[num_curves + idx_binary]) - epsilon);
                idx_binary += 1;

            }

        }

        // Optimize model
        model.optimize();

        std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

        // Get  the optimal scaling factors
        std::vector<double> sf(num_curves);
        for (int i = 0; i < num_curves; ++i)
        {
            sf[i] = vars[i].get(GRB_DoubleAttr_X);
        }
        // Get the scaling factors for each curve
        scaling_factors.resize(num_curves);
        for (int i = 0; i < num_curves; ++i)
        {
            scaling_factors[i].cur_idx = curves_idxs[i];
            scaling_factors[i].scale = sf[i];
            std::cout << "Scaling factor for curve " << curves_idxs[i] << ": " << sf[i] << std::endl;
        }

        // Scaling the curves
        Scaling();

    }

    catch(const GRBException& e)
    {
        std::cerr << "Error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
    }

}

void MultiTra_Planner:: Scaling()
{
    int cur_num = scaling_factors.size();

    for (int i = 0; i < cur_num; ++i)
    {
        int cur_idx = scaling_factors[i].cur_idx;
        double scale = scaling_factors[i].scale;

        if (scale != 1.0)
        {
            for (int j = 0; j < curves[cur_idx]->_duration.size(); ++j)
            {
                curves[cur_idx]->_duration[j] *= scale;
            }
        }
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    // 创建Marker消息的发布器
    auto start_time = std::chrono::high_resolution_clock::now();

    std::unique_ptr<MultiTra_Planner> MultiTraPlanner = std::make_unique<MultiTra_Planner>(nh);

    // 选择一个机器人，比如第一个机器人，发布其起止点以及走廊可视化
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    // 选择一个机器人，比如第一个机器人，发布其multiple_curves,即生成的曲线，nav_msgs::Path
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("visualization_path", 1);

    ros::Publisher mulity_pub = nh.advertise<visualization_msgs::Marker>("multi_car_marker", 10);


    ros::Rate rate(1);
    while (ros::ok() && (!MultiTraPlanner->path_planner->mapReceived() || !MultiTraPlanner->path_planner->doubleMapReceived()))
    {
        ros::spinOnce();
        rate.sleep();
    }


    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;
    ROS_INFO("Execution time: %.6f seconds for whole code running.", elapsed_time.count());

     elapsed_time = end_time - start_time - elapsed_time_gene_s_and_goal;

    ROS_INFO("Execution time: %.6f seconds for the propsed Motion Planning Method.", elapsed_time.count());


    // // 通过画图的形式显示合并后的曲线特性
    // MultiTraPlanner->path_planner->plotting();


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
        MultiTraPlanner-> visualization_test(mulity_pub);
        ros::spinOnce();
        rate.sleep();
    }




    return 0;
}



