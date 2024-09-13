#include "Beziercurve.h"

using namespace std;

// Calculate factorials up to the highest needed value
// static int factorial(const int &n)
// {
//     int result = 1;
//     for (int i = 2; i <= n; i++)
//     {
//         result *= i;
//     }
//     return result;
// }


constexpr int factorial(const int &n)
{
    int result = 1;
    for (int i = 2; i <= n; i++)
    {
        result *= i;
    }
    return result;
}



static vector<double> Binomial(const int &n)
{
    vector<double> C_n_i(n + 1);
    for (int i = 0; i <= n; ++i)
    {
        C_n_i[i] = 1.0 * factorial(n) / (factorial(i) * factorial(n - i));
    }
    return C_n_i;
}


Beziercurve::Beziercurve(const int &total)
{
    this->_total = total;                          // total points on curve
    this->_points.reserve(total + 1);              // points on curve from p_0 to p_N
    this->_v_s.reserve(total + 1);                 // velocity
    this->_w_s.reserve(total + 1);                 // angular velocity
    this->_a_ws.reserve(total + 1);                // angular acceleration
    this->_a_ts.reserve(total + 1);                // tangential acceleration
    this->_a_rs.reserve(total + 1);                // curvature
    this->_arc_length.resize(total + 1);           // radial acceleration
    this->_K_s.reserve(total + 1);                 // accumulated arc_length for segement
    this->_theta.reserve(total + 1);               // angle at each point

    this->_x_fir.reserve(total + 1);
    this->_y_fir.reserve(total + 1);


    this->_a_i.resize(total + 1); // a_i
    this->_b_i.resize(total);     // b_i

    this->_duration.resize(total + 1); // duration until i-th segement
}


void Beziercurve::get_params(const vector<pair<double, double>> &c_points, const double &T_s)
{
    this->_control_points = c_points;              // control points
    this->_orn = this->_control_points.size() - 1; // order of curve
    this->_T_s = T_s;                              // time for each segement(1/frequence)

    this->Atrributes();
}

pair<double, double> Beziercurve::B_pt(double &s) // P(s)
{

    double x = 0.0, y = 0.0;

    auto C_n_i = Binomial(this->_orn);

    for (int i = 0; i < this->_orn + 1; ++i)
    {
        double bernstein = C_n_i[i] * pow(1 - s, this->_orn - i) * pow(s, i);
        x += bernstein * this->_control_points[i].first;
        y += bernstein * this->_control_points[i].second;
    }
    return {x, y};
}
void Beziercurve::B_pts() // P(s), s /in [0,1]
{
    if (this->_total <= 0 || this->_control_points.empty())
        return;

    // // vector<pair<double, double>> points;
    // this->_points.reserve(total + 1);

    double s_step = 1.0 / this->_total;
    for (int j = 0; j <= this->_total; ++j)
    {
        double s = j * s_step;
        auto current_point = this->B_pt(s);
        // this->_points.insert(this->_points.end(), current_point.begin(), current_point.end());
        this->_points.push_back(current_point);
    }

}

pair<double, double> Beziercurve::fir_s(double &s)
{

    double dx = 0.0, dy = 0.0;

    auto C_n_i = Binomial(this->_orn - 1);

    for (int i = 0; i < this->_orn; ++i)
    {
        double bernstein = C_n_i[i] * pow(1 - s, this->_orn - 1 - i) * pow(s, i) * this->_orn;
        dx += bernstein * (this->_control_points[i + 1].first - this->_control_points[i].first);
        dy += bernstein * (this->_control_points[i + 1].second - this->_control_points[i].second);
    }

    return {dx, dy};
}

pair<double, double> Beziercurve::sec_s(double &s)
{

    double ddx = 0.0, ddy = 0.0;

    auto C_n_i = Binomial(this->_orn - 2);

    for (int i = 0; i < this->_orn - 1; ++i)
    {
        double bernstein = C_n_i[i] * pow(1 - s, this->_orn - 2 - i) * pow(s, i) * this->_orn * (this->_orn - 1);
        ddx += bernstein * (this->_control_points[i + 2].first - 2 * this->_control_points[i + 1].first + this->_control_points[i].first);
        ddy += bernstein * (this->_control_points[i + 2].second - 2 * this->_control_points[i + 1].second + this->_control_points[i].second);
    }
    return {ddx, ddy};
}

pair<double, double> Beziercurve::thir_s(double &s)
{

    double dddx = 0.0, dddy = 0.0;

    auto C_n_i = Binomial(this->_orn - 3);

    for (int i = 0; i < this->_orn - 2; ++i)
    {
        double bernstein = C_n_i[i] * pow(1 - s, this->_orn - 3 - i) * pow(s, i) * this->_orn * (this->_orn - 1) * (this->_orn - 2);
        dddx += bernstein * (-1 * this->_control_points[i].first + 3 * this->_control_points[i + 1].first - 3 * this->_control_points[i + 2].first + this->_control_points[i + 3].first);
        dddy += bernstein * (-1 * this->_control_points[i].second + 3 * this->_control_points[i + 1].second - 3 * this->_control_points[i + 2].second + this->_control_points[i + 3].second);
    }
    return {dddx, dddy};
}

Dynamics_S Beziercurve::get_dym(double &s)
{
    Dynamics_S dym;
    pair<double, double> d = this->fir_s(s);    // dx, dy
    pair<double, double> dd = this->sec_s(s);   // ddx, ddy
    pair<double, double> ddd = this->thir_s(s); // dddx, dddy

    double par1 = d.first * d.first + d.second * d.second;     // dx*dx + dy*dy
    double par2 = d.first * dd.second - dd.first * d.second;   // dx*ddy - ddx*dy
    double par3 = d.first * dd.first + d.second * dd.second;   // dx*ddx + dy*ddy
    double par4 = d.first * ddd.second - d.second * ddd.first; // dx*dddy - dy*dddx

    dym.v_s = sqrt(par1);
    dym.w_s = par2 / par1;
    dym.a_ws = (par4 * par1 - 2 * par2 * par3) / (par1 * par1);
    dym.a_ts = par3 / dym.v_s;
    dym.a_rs = par2 / dym.v_s;
    dym.K_s = dym.w_s / dym.v_s;

    dym.theta = atan2(d.second, d.first);

    dym.x_fir = d.first;
    dym.y_fir = d.second;
    return dym;
}

double Beziercurve::get_ds(double &s)
{
    pair<double, double> d = this->fir_s(s);

    double vs = sqrt(d.first * d.first + d.second * d.second);

    return vs;
}

void Beziercurve::Atrributes()
{
    this-> B_pts();

    double s_step = 1.0 / this->_total;
    for (int j = 0; j <= this->_total; ++j)
    {
        double s = j * s_step;
        Dynamics_S current_dym = this->get_dym(s);
        // this->_points.insert(this->_points.end(), current_point.begin(), current_point.end());
        this->_v_s.push_back(current_dym.v_s);
        this->_w_s.push_back(current_dym.w_s);
        this->_a_ws.push_back(current_dym.a_ws);
        this->_a_ts.push_back(current_dym.a_ts);
        this->_a_rs.push_back(current_dym.a_rs);
        this->_K_s.push_back(current_dym.K_s);
        this->_theta.push_back(current_dym.theta);
        this->_x_fir.push_back(current_dym.x_fir);
        this->_y_fir.push_back(current_dym.y_fir);
        // cout << current_dym.w_s << endl;
    }

}

void Beziercurve::TOTP(const Limits & lim)
{
    this->Vars.c_1.resize(this->_total + 1);
    this->Vars.c_2.resize(this->_total);
    this->Vars.c_3.resize(this->_total);
    this->Vars.c_4.resize(this->_total);
    this->Vars.c_5.resize(this->_total);
    this->Vars.c_6.resize(this->_total);
    this->Vars.c_7.resize(this->_total);
    this->Vars.c_8.resize(this->_total);
    this->Vars.c_9.resize(this->_total);
    this->Vars.c_10.resize(this->_total-1);
    this->Vars.c_11.resize(this->_total-1);
    this->Vars.c_12.resize(this->_total-1);
    this->Vars.c_13.resize(this->_total-1);
    this->Vars.c_14.resize(this->_total-1);
    this->Vars.c_15.resize(this->_total-1);

    this->_T_i.resize(this->_total);


    /*
    Calculate variables for optimization
    */
    // Calculate c_1 //
    for (int i = 0; i < this->_total + 1; ++i)
    {
        // v_con
        double a = 2.0 / this->_T_s;
        double b = 1.0 / this->_K_s[i];
        double c = b - lim.cem;
        double ce = a * sqrt(b * b - c * c); // chord error
        double vc = std::min(lim.v_max, ce);
        double vci = vc * vc / (this->_v_s[i] * this->_v_s[i]);
        // w_con
        double wci = lim.w_max * lim.w_max / (this->_w_s[i] * this->_w_s[i]);
        // ar_con
        double arci = fabs(lim.ar_max / this->_a_rs[i]);

        double d = std::min(std::min(vci, wci), arci);

        this->Vars.c_1[i] = d;
    }

    // Calculate c_2, c_3, c_4, c_5  //
    double h = 1.0 / this->_total;

    for (int i = 0; i < this->_total; ++i)
    {
        // n_i for c_2, c_3 and c_4, c_5
        double a = this->_v_s[i] * this->_a_ts[i];
        double n_t = 0.0;
        if (a >= 0)
        {
            n_t = 1.0;
        }

        double b = this->_v_s[i] + 2 * h * this->_a_ts[i] * n_t;
        double c = this->_v_s[i] - 2 * h * this->_a_ts[i] * (1 - n_t);

        // c_2, c_3
        this->Vars.c_2[i] = b / c;
        this->Vars.c_3[i] = fabs(2.0 * h * lim.at_max / c);

        // c_4, c_5
        this->Vars.c_4[i] = c / b;
        this->Vars.c_5[i] = fabs(2.0 * h * lim.at_max / b);

        // n_i for c_6, c_7 and c_8, c_9
        a = this->_w_s[i] * this->_a_ws[i];
        double n_w = 0.0;
        if (a >= 0)
        {
            n_w = 1.0;
        }

        double d = this->_w_s[i] + 2 * h * this->_a_ws[i] * n_w;
        double e = this->_w_s[i] - 2 * h * this->_a_ws[i] * (1 - n_w);

        // c_6, c_7
        this->Vars.c_6[i] = d / e;
        this->Vars.c_7[i] = fabs(2.0 * h * lim.aw_max / e);

        // c_8, c_9
        this->Vars.c_8[i] = e / d;
        this->Vars.c_9[i] = fabs(2.0 * h * lim.aw_max / d);
    }

    // c_10 ~ c_15
    for (int i = 0; i < this->_total-1; ++i)
    {
        double a = this->_v_s[i] * this->_a_ts[i];
        double ap1 = this->_v_s[i+1] * this->_a_ts[i+1];
        double n_t = 0.0;
        double n_tp1 = 0.0;
        if (a >= 0)
        {
            n_t = 1.0;
        }
        if (ap1 >= 0)
        {
            n_tp1 = 1.0;
        }

        this->Vars.c_10[i] = 2.0 * h *  this->_a_ts[i] *  n_t  +  this->_v_s[i] - 2.0 * h * this->_a_ts[i+1] * (1-n_tp1) + this->_v_s[i+1];
        this->Vars.c_11[i] = -2.0 * h * this->_a_ts[i+1] *  n_tp1 - this->_v_s[i+1];
        this->Vars.c_12[i] = 2.0 * h *  this->_a_ts[i] * (1-n_t) -  this->_v_s[i];


        double c = this->_v_s[i] * this->_a_ts[i];
        double cp1 = this->_v_s[i+1] * this->_a_ts[i+1];
        double n_w = 0.0;
        double n_wp1 = 0.0;
        if (c >= 0)
        {
            n_w = 1.0;
        }
        if (cp1 >= 0)
        {
            n_wp1 = 1.0;
        }

        this->Vars.c_13[i] = 2.0 * h * this->_a_ws[i] * n_w + this->_w_s[i] - 2.0 * h * this->_a_ws[i+1] * (1-n_wp1) + this->_w_s[i+1];
        this->Vars.c_14[i] = -2.0 * h * this->_a_ws[i+1] * n_wp1 - this->_w_s[i+1];
        this->Vars.c_15[i] = 2.0 * h * this->_a_ws[i] * (1-n_w) - this->_w_s[i];
    }

    this->Vars = this->Vars;



    /*
        LP OPTIMIZATION FOR TOTP
    */
    try
    {
        // Create an environment
        GRBEnv env = GRBEnv(true);
        //禁止打印输出信息
        // env.set("OutputFlag", "0");

        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);

        model.getEnv().set(GRB_IntParam_Threads, 12);

        int N = 2 * this->_total + 1;
        // Number of varibles [0]-[this->_total]是 a_i
        // [this->_total+1]-[2*this->_total]是 b_i

        // 创建变量
        std::vector<GRBVar> vars(N);
        for (int i = 0; i < N; ++i)
        {
            if (i < this->_total + 1)
            {
                // 对于前M个变量，范围是大于0
                vars[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            }
            else
            {
                // 对于其他变量，范围是任意实数
                vars[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            }
        }

        // Set cost function
        GRBLinExpr objective = 0.0;
        // max a_i
        for (int i = 0; i < this->_total + 1; ++i)
        {
            objective += vars[i];
        }

        model.setObjective(objective, GRB_MAXIMIZE);

        // Add constraints a_i < c_1 for i = 0,...,this->_total
        for (int i = 0; i < this->_total + 1; ++i)
        {
            model.addConstr(vars[i] <= this->Vars.c_1[i]);
        }

        // Add LP constraints for a_i and a_{i+1} for i = 0,...,this->_total-1
        for (int i = 0; i < this->_total; ++i)
        {
            /*a_t*/
            // Add constraints a_i < c_2 * a_{i+1} + c_3 for i = 0,...,this->_total-1
            model.addConstr(vars[i] <= this->Vars.c_2[i] * vars[i + 1] + this->Vars.c_3[i]);

            // Add constraints a_{i+1} < c_4 * a_{i} + c_5 for i = 0,...,this->_total-1
            model.addConstr(vars[i + 1] <= this->Vars.c_4[i] * vars[i] + this->Vars.c_5[i]);

            /*a_w*/
            // Add constraints a_i < c_6 * a_{i+1} + c_7 for i = 0,...,this->_total-1
            model.addConstr(vars[i] <= this->Vars.c_6[i] * vars[i + 1] + this->Vars.c_7[i]);

            // Add constraints a_{i+1} < c_8 * a_{i} + c_9 for i = 0,...,this->_total-1
            model.addConstr(vars[i + 1] <= this->Vars.c_8[i] * vars[i] + this->Vars.c_9[i]);


            // Add constraints b_i = (a_{i+1}-a_i)/(2*h)
            model.addConstr(vars[this->_total + 1 + i] == (vars[i + 1] - vars[i]) / (2.0 / this->_total));

            // // Add constraints c_i = fabs(b_i)
            // model.addConstr(vars[2*this->_total + 1 + i] >= vars[this->_total + 1 + i]);
            // model.addConstr(vars[2*this->_total + 1 + i] >= -vars[this->_total + 1 + i]);
        }

        // Add constraints a_0 = 0 and a_{this->_total} = 0
        model.addConstr(vars[0] == 0);
        model.addConstr(vars[this->_total] == 0);
        // Add constraints b_0 = 0 and b_{this->_total-1} = 0
        model.addConstr(vars[this->_total + 1] == 0);
        model.addConstr(vars[2 * this->_total] == 0);


        double lim_dif = 0.1;
        double h = 1.0 / this->_total;

        for (int i = 0; i < this->_total-2; ++i)
        {
            model.addConstr( this->Vars.c_10[i] * vars[i+1]  + this->Vars.c_11[i] * vars[i+2] + this->Vars.c_12[i] * vars[i] <= 2.0 * h * lim_dif );
            model.addConstr( this->Vars.c_10[i] * vars[i+1]  + this->Vars.c_11[i] * vars[i+2] + this->Vars.c_12[i] * vars[i] >= -2.0 * h * lim_dif );
            model.addConstr( this->Vars.c_13[i] * vars[i+1]  + this->Vars.c_14[i] * vars[i+2] + this->Vars.c_15[i] * vars[i] <= 2.0 * h * lim_dif) ;
            model.addConstr( this->Vars.c_13[i] * vars[i+1]  + this->Vars.c_14[i] * vars[i+2] + this->Vars.c_15[i] * vars[i] >= -2.0 * h * lim_dif) ;

        }

        // 优化模型
        model.optimize();

        // 将结果分别存入a_i和b_i
        for (int i = 0; i < this->_total + 1; ++i)
        {
            // this->_a_i[i] = vars[i].get(GRB_DoubleAttr_X);
            // this->_a_i[i] = this->_a_i[i];

            this->_a_i[i] = vars[i].get(GRB_DoubleAttr_X);
        }
        for (int i = 0; i < this->_total; ++i)
        {
            // this->_b_i[i] = vars[this->_total + 1 + i].get(GRB_DoubleAttr_X);
            // this->_b_i[i] = this->_b_i[i];

            this->_b_i[i] = vars[this->_total + 1 + i].get(GRB_DoubleAttr_X);
        }
    }
    catch (GRBException& e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }
    catch (const std::exception &e)
    {
        ROS_INFO("Error: TOTP for first optimization failed");
    }




    /*
        Calculate T_i and Get duration
    */
    // Calculate T_i
    for (int i = 0; i < this->_total; ++i)
    {
        double a = sqrt(this->_a_i[i]) + sqrt(this->_a_i[i + 1]);
        if (a == 0.0)
        {
            this->_T_i[i] = 0.0;
        }
        else
        {
            this->_T_i[i] = 2.0 * h / a;
        }
    }

    // Calculate T_total
    // this->_T_total[0] = 0;
    this->_duration[0] = 0;
    for (int i = 0; i < this->_total; ++i)
    {
        // this->_T_total[i + 1] = this->_T_total[i] + this->_T_i[i];
        this->_duration[i+1] = this->_duration[i] + this->_T_i[i];
    }



}

