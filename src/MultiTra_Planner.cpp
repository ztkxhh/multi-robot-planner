#include "MultiTra_Planner.h"
#include<Visualization.hpp>
#include <fstream>

double inf = std::numeric_limits<double>::infinity();

bool PlanningSuccess = false;



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

    if (path_planner->path_planning_successful_ == false)
    {
        ROS_ERROR("Path planning failed, please check the map and parameters.");
        return;
    }
    else
    {
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
    double epsilon = 1e-3;
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
            if (dist2 <= threshold2) {
                influenceMatrix[i][j] = true;
                influenceMatrix2[j][i] = true;

            }
        }
    }

    // ROS_INFO("Influence matrix for  curve %d and curve %d computed", idxA, idxB);
    std::vector<std::vector<InfluenceInfo>> AB = seg_processing(a, b, influenceMatrix);
    // ROS_WARN("From AB to BA");
    std::vector<std::vector<InfluenceInfo>> BA = seg_processing(b, a, influenceMatrix2);
    // ROS_WARN("Influence matrix size: %d x %d", nA, nB);


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
                    double cof_ab = 100.0;
                    double cof_ba = 100.0;


                    for (int m = 0; m < seg_ab_size; ++m)
                    {
                        // a_ahead_b
                        if (a._duration[AB[i][m].indexA] == 0.0 )
                        {
                            // ROS_WARN("A ahead B detected between curve %d and curve %d, cannot compute influence pair", idxA, idxB );
                            continue;
                        }
                        cof_ab = std::min(cof_ab, b._duration[AB[i][m].indexB] / a._duration[AB[i][m].indexA]);
                    }

                    for ( int n = 0; n < seg_ba_size; ++n)
                    {
                        if (b._duration[BA[k][n].indexA]  == 0.0)
                        {
                            // ROS_WARN("B ahead A duration detected between curve %d and curve %d, cannot compute influence pair", idxA, idxB );
                            continue;
                        }
                        // b_ahead_a
                        cof_ba = std::min(cof_ba, a._duration[BA[k][n].indexB] / b._duration[BA[k][n].indexA]);
                    }

                    // if ( ! std::isfinite(cof_ab) ||  ! std::isfinite(cof_ba) )
                    // {
                    //     ROS_WARN("!!!!!!!!!!!!!!!! Infinite influence pair detected between curve %d and curve %d, cannot compute influence pair", idxA, idxB );
                    // }

                    // if (cof_ab == 0 && cof_ba == 0)
                    // {
                    //     ROS_WARN("!!!!!!!!!!!!!!!! Zero influence pair detected between curve %d and curve %d, cannot compute influence pair", idxA, idxB );
                    
                    //     // // print for debug
                    //     // std::cout<<"--------------"<<std::endl;
                    //     // ROS_INFO("idxA: %d and %d", idxA, idxB);
                    //     // ROS_INFO("seg_ab_type: %d", seg_ab_type);
                    //     // ROS_INFO("a_head_b: %f", cof_ab);
                    //     // ROS_INFO("b_ahed_a: %f", cof_ba);
                    //     // ROS_INFO("a_b_starta: %d   and   a_b_enda: %d", start_ab_idx_a, end_ab_idx_a);
                    //     // ROS_INFO("a_b_startb: %d   and   a_b_endb: %d", start_ab_idx_b, end_ab_idx_b);
                    //     // ROS_INFO("b_a_startb: %d   and   b_a_endb: %d", start_ba_idx_b, end_ba_idx_b);
                    //     // ROS_INFO("b_a_starta: %d   and   b_a_enda: %d", start_ba_idx_a, end_ba_idx_a);
                    //     // std::cout<<"--------------"<<std::endl;
                    // }

                    influncepair pair;
                    pair.a_head_b = cof_ab;
                    pair.b_ahed_a = cof_ba;



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

        int preidxB = -10;

        for (int j = 0; j < nB; ++j)
        {
            if (influenceMatrix[i][j])
            {
                int dif_j = j - preidxB;
                if (dif_j > 9)
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


    // // 输出打印groups用以调试
    // for (const auto& groupEntry : groups) {
    //     const std::vector<int>& groupIndices = groupEntry.second;
    //     std::cout << "Group: ";
    //     for (size_t i = 0; i < groupIndices.size(); ++i) {
    //         std::cout << groupIndices[i] << " ";
    //     }
    //     std::cout << std::endl;
    //     std::cout << "----------------" << std::endl;

    // }


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
    curves_idxs_set.reserve(seg_size * 5);

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

        model.getEnv().set(GRB_IntParam_Threads, 8);
        // Set time limit and MIP gap
        model.getEnv().set(GRB_DoubleParam_TimeLimit, 10);
        model.getEnv().set(GRB_DoubleParam_MIPGap, 1e-3);
        model.getEnv().set(GRB_DoubleParam_MIPGapAbs, 1e-3);


        int num_curves = curves_idxs.size();
        // Create variables. first curves_idxs.size() are sacling factors for all influenced curves
        // and the rest are binary variables for each influence pair
        std::vector<GRBVar> vars(num_curves+ binary_num);
        // Create scaling factors for each curve
        for (int i = 0; i < num_curves; ++i)
        {
            vars[i] = model.addVar(1.0, 20, 0.0, GRB_CONTINUOUS);
        }
        // Create binary variables for each influence pair
        for (int i = num_curves; i < num_curves + binary_num; ++i)
        {
            vars[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }

        std::vector<double> durations(num_curves);
        // std::vector<double>duration_weithts(num_curves);
        // double max_duration = 0.0;
        for (int i = 0; i < num_curves; ++i)
        {
            durations[i] = curves[curves_idxs[i]]->_duration.back();
            // max_duration = std::max(max_duration, durations[i]);
        }
        // for (int i = 0; i < num_curves; ++i)
        // {
        //     duration_weithts[i] =  durations[i] /max_duration;
        // }


        // Set cost function
        GRBLinExpr objective = 0.0;
        for (int i = 0; i < num_curves; ++i)
        {
            // objective += vars[i];
            // objective += vars[i] * curves[curves_idxs[i]]->_duration.back();
            // objective += vars[i] * duration_weithts[i];
            objective += vars[i] * durations[i];


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

            if (it_B != curves_idxs.end())
            {
                idx_B_vars = std::distance(curves_idxs.begin(), it_B);
            }


            for (int j = 0; j < influenceSegments[i].influencePairs.size(); ++j)
            {
                double a_head_b = influenceSegments[i].influencePairs[j].a_head_b;
                double b_ahed_a = influenceSegments[i].influencePairs[j].b_ahed_a;

                if (a_head_b == 0.0 && b_ahed_a == 0.0)
                {
                    ROS_WARN("Zero scaling factor detected between curve %d and curve %d, cannot compute influence pair", idx_A, idx_B );
                    idx_binary += 1;
                    continue;
                }
                if (a_head_b == inf || b_ahed_a == inf)
                {
                    ROS_WARN("Infinite scaling factor detected between curve %d and curve %d, cannot compute influence pair", idx_A, idx_B );
                    idx_binary += 1;
                    continue;
                }


                model.addConstr(vars[idx_A_vars] <= vars[idx_B_vars] * a_head_b + M * vars[num_curves + idx_binary]);
                model.addConstr(vars[idx_B_vars] <= vars[idx_A_vars] * b_ahed_a + M * (1 - vars[num_curves + idx_binary]));
                idx_binary += 1;

            }

        }

        // Optimize model
        model.optimize();

        // std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

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
            // std::cout << "Scaling factor for curve " << curves_idxs[i] << ": " << sf[i] << std::endl;
        }

        // Scaling the curves
        Scaling();

    }

    catch(const GRBException& e)
    {
        std::cerr << "MULTI-PLANNING Error code = " << e.getErrorCode() << std::endl;
        std::cerr << "MULTI-PLANNING" << e.getMessage() << std::endl;
    }


}

void MultiTra_Planner:: Scaling()
{
    int cur_num = scaling_factors.size();


    for (int i = 0; i < cur_num; ++i)
    {
        double scale = scaling_factors[i].scale;
        int cur_idx = scaling_factors[i].cur_idx;

        if (scale != 1.0)
        {
            // std::cout<<"----------------"<<std::endl;
            // std::cout <<"Curve "<< cur_idx << " 's duration is "<< curves[cur_idx]->_duration.back() << " before scaling." << std::endl;
            // std::cout <<"Scaling factor for curve "<< cur_idx << " is "<< scale << std::endl;
            for (int j = 0; j < curves[cur_idx]->_duration.size(); ++j)
            {
                curves[cur_idx]->_duration[j] *= scale;
            }
            // std::cout <<"Curve "<< cur_idx << " 's duration is "<< curves[cur_idx]->_duration.back() << " after scaling." << std::endl;
            // std::cout<<"----------------"<<std::endl;

        }

    }


    PlanningSuccess = true;
}



// void MultiTra_Planner::Verification(int& vis_hz)
// {

//     int num_robots = curves.size();


//     std::vector<std::vector<double>> d_t;
//     d_t.resize(num_robots);

//     std::vector<std::vector<double>> v_t;
//     v_t.resize(num_robots);


//     for (int car_id = 0; car_id < num_robots; ++car_id)
//     {
//         const auto traj = curves[car_id];
//         int points_num = traj->_points.size();
//         std::cout<<"----------------"<<std::endl;
//         ROS_INFO("The duration of Curve %d is %f", car_id, traj->_duration.back());
//         std::cout<<"----------------"<<std::endl;

//         double distance = 0.0;
//         double dt = 0.0;
//         double vt = 0.0;
//         for (int i = 0; i < points_num-6; ++i)
//         {   std::cout<<"i = "<< i << "duration is "<< traj->_duration[i]   <<std::endl;
//             dt = traj->_duration[i+1] - traj->_duration[i];
//             distance = sqrt(pow(traj->_points[i+1].first - traj->_points[i].first, 2) + pow(traj->_points[i+1].second - traj->_points[i].second, 2));

//             if (dt != 0.0)
//             {
//                 vt = distance / dt;
//                 v_t[car_id].push_back(vt);
//             }
//             else
//             {
//                 ROS_WARN("Zero duration detected between i = %d and i = %d, cannot compute velocity", i, i+1);
//             }
//             d_t[car_id].push_back(dt);

//         }

//     }

//     std::vector<double> v_max;



//     // 输出打印v_max用以调试
//     for (int i = 0; i < num_robots; ++i)
//     {
//         double vmax = *std::max_element(v_t[i].begin(), v_t[i].end());
//         ROS_INFO(" The index of vmax  in v_t[i] is %d", std::distance(v_t[i].begin(), std::max_element(v_t[i].begin(), v_t[i].end())));
//         v_max.push_back(vmax);
//         std::cout << "v_max: " << vmax << std::endl;
//     }


// }

double MultiTra_Planner::calculatePathLength(const std::shared_ptr<Beziercurve>& curve) {
    double length = 0.0;
    for (size_t i = 1; i < curve->_points.size(); ++i) {
        double dx = curve->_points[i].first - curve->_points[i-1].first;
        double dy = curve->_points[i].second - curve->_points[i-1].second;
        length += std::hypot(dx, dy);
        // length += (dx*dx+dy*dy);

    }
    return length;
}

// // 调用示例
// for (size_t i = 0; i < merged_curves.size(); ++i) {
//     double length = calculatePathLength(merged_curves[i]);
//     ROS_INFO("Robot %zu Path Length: %.4f meters", i, length);
// }


double MultiTra_Planner::calculateJerkFromStoredData(const std::shared_ptr<Beziercurve>& curve) {
    double total_jerk = 0.0;
    // 假设 _a_ts 存储切向加速度，_a_ws 存储角加速度
    for (size_t i = 1; i < curve->_a_ts.size(); ++i) {
        // 计算加速度变化率（Jerk）
        double delta_a_t = curve->DYM.at_t[i] - curve->DYM.at_t[i-1];
        double delta_a_w = curve->DYM.aw_t[i] - curve->DYM.aw_t[i-1];
        
        // double delta_a_t = curve->_a_ts[i] - curve->_a_ts[i-1];
        // double delta_a_w = curve->_a_ws[i] - curve->_a_ws[i-1];

        // double dx = curve->_points[i].first - curve->_points[i-1].first;
        // double dy = curve->_points[i].second - curve->_points[i-1].second;
        // double distance = std::hypot(dx, dy);

        double dt = curve->_duration[i] - curve->_duration[i-1]; // 时间间隔
        // std::cout << "dt: " << dt << std::endl;
        if (dt > 0) {
            // total_jerk += std::sqrt(std::pow(delta_a_t / dt, 2) + std::pow(delta_a_w / dt, 2));
            total_jerk += (std::pow(delta_a_t / dt, 2) + std::pow(delta_a_w / dt, 2));

        }
    }
    return total_jerk;
}

// double MultiTra_Planner::calculateJerkFromStoredData(const std::shared_ptr<Beziercurve>& curve , double ds) {
//     double total_jerk = 0.0;
//     int n_points = curve->_points.size();

//     // 假设参数 λ 均匀分布在 [0,1]，步长 Δλ = 1/(n_points-1)
//     double dlambda = ds;
//     for (int k = 0; k < n_points; ++k) {
//         // 获取三阶导数 dddx 和 dddy（假设存储在 curve->_a_rs 中）
//         double dddx = curve->_x_third[k];  // 根据实际数据结构调整
//         double dddy = curve->_y_third[k];

//         // 计算模长平方
//         double jerk_sq = std::hypot(dddx, dddy);
//         // double jerk_sq = dddx * dddx + dddy * dddy;

//         // 积分累加（矩形法）
//         total_jerk += jerk_sq * dlambda;
//     }

//     return total_jerk;
// }


// // 计算累积弧长
// std::vector<double> computeCumulativeArcLength(const std::vector<std::pair<double, double>>& points) {
//     std::vector<double> s(points.size(), 0.0);
//     for (size_t i = 1; i < points.size(); ++i) {
//         double dx = points[i].first - points[i-1].first;
//         double dy = points[i].second - points[i-1].second;
//         s[i] = s[i-1] + std::hypot(dx, dy);
//     }
//     return s;
// }


// double calculateJerk(const std::vector<std::pair<double, double>>& points) {
//     const auto& s = computeCumulativeArcLength(points);
//     int n = points.size();
//     if (n < 5) {
//         ROS_WARN("至少需要5个采样点计算Jerk.");
//         return 0.0;
//     }

//     double total_jerk = 0.0;
//     for (int i = 2; i < n - 2; ++i) {
//         // 获取相邻点的弧长和坐标
//         double s_im2 = s[i-2], s_im1 = s[i-1], s_i = s[i], s_ip1 = s[i+1], s_ip2 = s[i+2];
//         double x_im2 = points[i-2].first, x_im1 = points[i-1].first, x_i = points[i].first, x_ip1 = points[i+1].first, x_ip2 = points[i+2].first;
//         double y_im2 = points[i-2].second, y_im1 = points[i-1].second, y_i = points[i].second, y_ip1 = points[i+1].second, y_ip2 = points[i+2].second;

//         // 计算局部平均步长 Δs
//         double delta_s = (s_ip2 - s_im2) / 4.0; // 平均步长近似

//         // 三阶导数近似（中心差分）
//         double d3x = (x_ip2 - 2*x_ip1 + 2*x_im1 - x_im2) / (2 * delta_s * delta_s * delta_s);
//         double d3y = (y_ip2 - 2*y_ip1 + 2*y_im1 - y_im2) / (2 * delta_s * delta_s * delta_s);

//         // 积分区间：当前点前后两个区间的平均弧长
//         double ds_left = s_i - s_im1;
//         double ds_right = s_ip1 - s_i;
//         double ds_avg = (ds_left + ds_right) / 2.0;

//         total_jerk += (d3x * d3x + d3y * d3y) * ds_avg;
//     }
//     return total_jerk;
// }



int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;
    // 创建Marker消息的发布器
    auto start_time_node = std::chrono::high_resolution_clock::now();

    std::shared_ptr<MultiTra_Planner> MultiTraPlanner = std::make_shared<MultiTra_Planner>(nh);



    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "\033[1m\033[32m Successfully find solution! \033[0m\n";


    std::chrono::duration<double> elapsed_time_planning = end_time - start_time_node - elapsed_time_gene_s_and_goal;
    // ROS_INFO("Execution time: %.6f seconds for the propsed Motion Planning Method.", elapsed_time_planning.count());
    // ROS_INFO("\033[1m\033[32m Execution time:  \033[0m %.6f seconds for the propsed Motion Planning Method.", elapsed_time_planning.count());
    std::cout << "\033[1m\033[32m Execution time:  \033[0m" << "\033[1m\033[32m" << elapsed_time_planning.count() << " s." << "\033[0m" << std::endl;

    // ROS_INFO("Execution time: %.6f seconds for overall code running.", elapsed_time.count());



    if (PlanningSuccess) {
        int points_num = 0;
        nh.getParam("points_num", points_num);
        double ds = 1.0 / points_num;

        double total_length = 0.0;
        double total_jerk = 0.0;

        for (size_t i = 0; i < MultiTraPlanner->curves.size(); ++i) {
            double length = MultiTraPlanner->calculatePathLength(MultiTraPlanner->curves[i]);

            // const auto& curve_points = MultiTraPlanner->curves[i]->_points;
            // double jerk = calculateJerk(curve_points);


            double jerk = MultiTraPlanner->calculateJerkFromStoredData(MultiTraPlanner->curves[i]);
            total_length += length;
            total_jerk += jerk;
            // ROS_INFO("Robot %zu Path Length: %.4f meters", i, length);
            // ROS_INFO("Robot %zu Jerk: %.4f", i, jerk);
        }

        double average_length = total_length / MultiTraPlanner->curves.size();
        double average_jerk = total_jerk / MultiTraPlanner->curves.size();
        // ROS_INFO("Average Path Length: %.4f meters", average_length);
        // ROS_INFO("Average Jerk: %.4f", average_jerk);



        double max_duration = 0.0;
        for (int i = 0; i < MultiTraPlanner->curves.size(); ++i)
        {
            max_duration = std::max(max_duration, MultiTraPlanner->curves[i]->_duration.back());
        }

        // ROS_INFO("max_duration: %f", max_duration);
        std::cout << "\033[1m\033[32m Makespan:  \033[0m" << "\033[1m\033[32m" << max_duration << " s." << "\033[0m" << std::endl;



        // 将结果写入文件（追加模式）
        std::ofstream outfile;
        outfile.open("/home/zt/文档/results.txt", std::ios_base::app);
        if (outfile.is_open()) {
            outfile << elapsed_time_planning.count() << " " << max_duration << std::endl;
            outfile.close();
        } else {
            ROS_ERROR("无法写入结果文件");
        }
    }
    else
    {
        std::cout << "\033[1m\033[91m No solution found! \033[0m\n";
    }

    //为了统计数据，暂不执行下属代码，但是实际运行时需要执行
    // // MultiTraPlanner->path_planner->plotting();
    // if (PlanningSuccess)
    // {

    //     int visual_hz = 20;

    //     // MultiTraPlanner->Verification(visual_hz);
    //     ros::Rate rate(visual_hz);
    //     std::unique_ptr<ResultPublisher> resultPublisher_obj =  std::make_unique<ResultPublisher>(nh, MultiTraPlanner);
    //     double current_time;
    //     double start_time = ros::Time::now().toSec();
    //     while (ros::ok())
    //     {
    //         current_time = ros::Time::now().toSec() - start_time;

    //         resultPublisher_obj->update(current_time);
    //         resultPublisher_obj->publish();
    //         ros::spinOnce();
    //         rate.sleep();
    //     }
    // }

    // // // 通过画图的形式显示合并后的曲线特性,for debug
    // // MultiTraPlanner->path_planner->plotting();



    return 0;
}

































// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "path_planner");
//     ros::NodeHandle nh;

//     // 创建Marker消息的发布器
//     auto start_time = std::chrono::high_resolution_clock::now();

//     std::unique_ptr<MultiTra_Planner> MultiTraPlanner = std::make_unique<MultiTra_Planner>(nh);

//     // 选择一个机器人，比如第一个机器人，发布其起止点以及走廊可视化
//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
//     // 选择一个机器人，比如第一个机器人，发布其multiple_curves,即生成的曲线，nav_msgs::Path
//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("visualization_path", 1);

//     ros::Publisher mulity_pub = nh.advertise<visualization_msgs::Marker>("multi_car_marker", 10);


//     ros::Rate rate(1);
//     while (ros::ok() && (!MultiTraPlanner->path_planner->mapReceived() || !MultiTraPlanner->path_planner->doubleMapReceived()))
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }


//     auto end_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed_time = end_time - start_time;
//     ROS_INFO("Execution time: %.6f seconds for whole code running.", elapsed_time.count());

//      elapsed_time = end_time - start_time - elapsed_time_gene_s_and_goal;

//     ROS_INFO("Execution time: %.6f seconds for the propsed Motion Planning Method.", elapsed_time.count());


//     // // 通过画图的形式显示合并后的曲线特性
//     // MultiTraPlanner->path_planner->plotting();


//     // // 选择一个机器人，比如第一个机器人，发布其路径可视化
//     // while (ros::ok())
//     // {
//     //     size_t robot_index = 0;
//     //     MultiTraPlanner->path_planner->publishPathVisualization(robot_index, marker_pub, path_pub);
//     //     ros::spinOnce();
//     //     rate.sleep();
//     // }


//     // 创建一个发布器的数组
//     std::vector<ros::Publisher> path_pubs(MultiTraPlanner->path_planner->merged_curves.size());
//     // 为每个曲线初始化一个发布器
//     for (size_t i = 0; i <MultiTraPlanner-> path_planner->merged_curves.size(); i++)
//     {
//         std::stringstream pub_name;
//         pub_name << "path_pub_" << i;
//         path_pubs[i] = nh.advertise<nav_msgs::Path>(pub_name.str(), 1);
//     }

//     // 面向所有机器人，发布其路径可视化
//     while (ros::ok())
//     {
//         // MultiTraPlanner->path_planner->publishPathsVisualization(path_pubs, marker_pub);
//         MultiTraPlanner-> visualization_test(mulity_pub);
//         ros::spinOnce();
//         rate.sleep();
//     }




//     return 0;
// }



