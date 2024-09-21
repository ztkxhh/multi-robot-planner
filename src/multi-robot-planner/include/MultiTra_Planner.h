#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <utility>
#include <unordered_map>
#include <nanoflann.hpp>
#include "Path_Planner.h"

struct InfluenceInfo {
    int indexA;
    int indexB;
    bool Infulencetype; // true: acute, false: non-acute
};


struct influncepair
{
    int indexA;
    int indexB;
};


struct InfluenceSegment
{
    int curveAIndex;
    int curveBIndex;
    std::vector<influncepair> AB;
    std::vector<influncepair> BA;
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

    bool curvesInfluenceEachOther( const Beziercurve& a, const Beziercurve& b, double threshold);

    void processCurvePair(const Beziercurve& a, const Beziercurve& b, int& idxA, int& idxB, double& threshold);

    bool computeInfluenceType(const Beziercurve& a, const Beziercurve& b, int& idxA, int& idxB);


    void GuropSubstion ();

public:

    std::shared_ptr<Path_Planner> path_planner;

    std::vector<InfluenceSegment> influenceSegments;

    MultiTra_Planner(ros::NodeHandle& nh);
    ~MultiTra_Planner()=default;
};



