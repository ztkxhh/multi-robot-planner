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


Beziercurve::Beziercurve(const vector<pair<double, double>> &c_points, const int &total, const double &T_s)
{
    this->_control_points = c_points;              // control points
    this->_orn = this->_control_points.size() - 1; // order of curve
    this->_total = total;                          // total points on curve
    this->_T_s = T_s;                              // time for each segement(1/frequence)
    this->_points.reserve(total + 1);              // points on curve from p_0 to p_N
    this->_ds.reserve(total);                      // arc_length for each segement
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

    this->_duration.resize(total + 1);

    this-> B_pts();
    this->Atrributes();
}

Beziercurve::~Beziercurve()
{
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


    // 计算每个segement的arc_length
    this->_arc_length[0] = 0.0;
    // b的首个元素为0，并计算其余元素
    for (int i = 1; i <= this->_total; ++i)
    {
        this->_arc_length[i] = this->_arc_length[i - 1] + this->_ds[i - 1];
        // cout << this->_arc_length[i] << endl;
    }
}
