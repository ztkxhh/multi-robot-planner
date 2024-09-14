#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include "gurobi_c++.h"

struct Limits
{
    double v_max, w_max, aw_max, at_max, ar_max, cem;
};


struct Dynamics_T
{
    // std::vector<double> v_t_m, w_t_m, aw_t_m, at_t_m, ar_t_m; // dynamics at the middle point of each segement
    std::vector<double> v_t, w_t, aw_t, at_t, ar_t; // dynamics at each begining point of the trajectory
};

struct MultiVars
{
    std::vector<double> c_1, c_2, c_3, c_4, c_5, c_6, c_7, c_8, c_9,c_10,c_11,c_12,c_13,c_14,c_15;
};


struct Dynamics_S
{
    double v_s, w_s, a_ws, a_ts, a_rs, K_s, theta, x_fir, y_fir;
};


class Beziercurve
{
private:
    std::vector<std::pair<double, double>> _control_points;
    int _orn; // order of curve

    std::pair<double, double> fir_s(double &s);
    std::pair<double, double> sec_s(double &s);
    std::pair<double, double> thir_s(double &s);

public:


    int _total;
    double _T_s; // 1 / frenquence

    // parameters for TOTP
    MultiVars Vars; // variables for optimaztion
    std::vector<double> _T_i; // time for each segement
    std::vector<double>_duration; // duration until i-th segement
    std::vector<double> _a_i; // a_i
    std::vector<double> _b_i; // b_i
    Dynamics_T DYM;

    // parameters for Bezier curve path
    std::vector<std::pair<double, double>> _points; // points on curve from p_0 to p_N
    std::vector<double> _v_s;        // velocity
    std::vector<double> _w_s;        // angular velocity
    std::vector<double> _a_ws;       // angular acceleration
    std::vector<double> _a_ts;       // tangential acceleration
    std::vector<double> _a_rs;       // radial acceleration
    std::vector<double> _K_s;        // curvature
    std::vector<double> _theta;       // angle at each point

    // parameters for Time allocation, but calculate in Beziercurve.cpp
    std::vector<double>_x_fir;
    std::vector<double>_y_fir;


    Beziercurve(const int &total);
    ~Beziercurve()= default;

    void get_params(const std::vector<std::pair<double, double>> &c_points, const double &T_s);
    // Function to compute Bezier curve points from s
    std::pair<double, double> B_pt(double &s); // P(s)
    void B_pts(); // P(s), s /in [0,1]
    double get_ds(double &s);
    Dynamics_S get_dym(double &s);
    void Atrributes();  // Compute all attributes

    void TOTP(const Limits & lim, ros::NodeHandle &nh);


};