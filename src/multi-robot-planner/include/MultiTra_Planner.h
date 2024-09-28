#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <utility>
#include <unordered_map>
#include <nanoflann.hpp>
#include <limits>
#include<random>
#include<set>

#include "Path_Planner.h"

struct InfluenceInfo {
    int indexA;
    int indexB;
    bool Infulencetype; // true: acute, false: non-acute
};


struct influncepair
{
    double a_head_b; // a head to b cofficient
    double b_ahed_a; // b head to a cofficient
    // bool a_b_starta; // true :  a is start from the start in a head b; false: a is  not start from the start in a head b
    // bool a_b_enda;   // true :  a is end at the end in a head b; false: a is  not end at the end in a head b
    // bool b_a_startb; // true :  b is start from the start in b head a; false: b is  not start from the start in b head a
    // bool b_a_endb;  // true :  b is end at the end in b head a; false: b is  not end at the end in b head a
};

struct scaling_factor
{
    int cur_idx;
    double scale;
};


struct InfluenceSegment
{
    int curveAIndex;
    int curveBIndex;
    std::vector<influncepair> influencePairs;
};


// 并查集（用于对曲线进行分组）
class UnionFind {
public:
    UnionFind(int n) : parent(n) {
        for(int i = 0; i < n; ++i) parent[i] = i;
    }
    int find(int x) {
        if(parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    }
    void unite(int x, int y) {
        int fx = find(x), fy = find(y);
        if(fx != fy) parent[fy] = fx;
    }
private:
    std::vector<int> parent;
};






class MultiTra_Planner
{
private:

    double robot_radius;

    double influnce_factor;

    double InfluenceThreshold;

    std::vector <std::shared_ptr<Beziercurve>> curves;

    std::vector<scaling_factor> scaling_factors;

    bool curvesInfluenceEachOther( const Beziercurve& a, const Beziercurve& b, double threshold);

    void processCurvePair(const Beziercurve& a, const Beziercurve& b, int& idxA, int& idxB, double& threshold);

    bool computeInfluenceType(const Beziercurve& a, const Beziercurve& b, int& idxA, int& idxB);

    std::vector<std::vector<InfluenceInfo>> seg_processing(const Beziercurve& a, const Beziercurve& b, std::vector<std::vector<bool>>& influenceMatrix);

    void GuropSubstion ();

    void MILP_Adujust();

    void Scaling();

public:

    std::unique_ptr<Path_Planner> path_planner;

    std::vector<InfluenceSegment> influenceSegments;

    void visualization_test(ros::Publisher &marker_pub);

    MultiTra_Planner(ros::NodeHandle& nh);
    ~MultiTra_Planner()=default;
};



