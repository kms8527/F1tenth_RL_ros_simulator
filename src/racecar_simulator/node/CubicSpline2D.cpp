#include "CubicSpline2D.h"

#include <algorithm>
#include <cmath>
#include <numeric>

using namespace std;

// Default constructor
CubicSpline2D::CubicSpline2D() = default;

// Construct the 2-dimensional cubic spline
// CubicSpline2D::CubicSpline2D(const vector<double> &x,
//                             const vector<double> &y) {
CubicSpline2D::CubicSpline2D(const vector<geometry_msgs::PointStamped> &global_path) {
    s.clear();
    filtered_points.clear();

    filtered_points = remove_collinear_points(global_path);
    calc_s(filtered_points[0],
           filtered_points[1]); // filtered_points[0] : filtered_global_path's x ,
                                // filtered_points[1] : filtered_global_path's y
    sx = CubicSpline1D(s, filtered_points[0]);
    sy = CubicSpline1D(s, filtered_points[1]);
}

void CubicSpline2D::setCubicSpline2D(const vector<geometry_msgs::PointStamped> &global_path) {

    filtered_points = remove_collinear_points(global_path);
    calc_s(filtered_points[0],
           filtered_points[1]); // filtered_points[0] : filtered_global_path's x ,
                                // filtered_points[1] : filtered_global_path's y
    sx = CubicSpline1D(s, filtered_points[0]);
    sy = CubicSpline1D(s, filtered_points[1]);
    std::cout << "Cubicspline 1D set" << std::endl;
}

// Calculate the s values for interpolation given x, y
void CubicSpline2D::calc_s(const vector<double> &x, const vector<double> &y) {
    int nx = x.size();
    vector<double> dx(nx);
    vector<double> dy(nx);
    adjacent_difference(x.begin(), x.end(), dx.begin());
    adjacent_difference(y.begin(), y.end(), dy.begin());
    dx.erase(dx.begin());
    dy.erase(dy.begin());

    double cum_sum = 0.0;
    s.push_back(cum_sum);
    for (int i = 0; i < nx - 1; i++) {
        cum_sum += hypot(dx[i], dy[i]);
        s.push_back(cum_sum);
    }
    s.erase(unique(s.begin(), s.end()), s.end());
}

// CubicSpline은 보간 기법으로 일련의 점들을 부드럽게 연결하여 곡선을 형성
// Calculate the x position along the spline at given t
double CubicSpline2D::calc_x(double t) { return sx.calc_der0(t); }

double CubicSpline2D::calc_xdot(double t) { return sx.calc_der1(t); }

// Calculate the y position along the spline at given t
double CubicSpline2D::calc_y(double t) { return sy.calc_der0(t); }

double CubicSpline2D::calc_ydot(double t) { return sy.calc_der1(t); }

// Calculate the curvature along the spline at given t
double CubicSpline2D::calc_curvature(double t) {
    double dx = sx.calc_der1(t);
    double ddx = sx.calc_der2(t);
    double dy = sy.calc_der1(t);
    double ddy = sy.calc_der2(t);
    double ref_nom = ddy * dx - ddx * dy;
    double ref_denom = pow(pow(dx, 2) + pow(dy, 2), 1.5);
    if (std::fabs(ref_nom) < 1e-7)
        ref_nom = 0;
    if (std::fabs(ref_denom) < 1e-7)
        ref_denom = 1e-7;

    double k = ref_nom / ref_denom;
    // double k = (ddy * dx - ddx * dy) / pow(pow(dx, 2) + pow(dy, 2), 1.5);
    return k;
}

// Calculate the yaw along the spline at given t
double CubicSpline2D::calc_yaw(double t) {
    double dx = sx.calc_der1(t);
    double dy = sy.calc_der1(t);
    double yaw = atan2(dy, dx);
    while (yaw > M_PI) {
        yaw -= 2 * M_PI;
    }
    while (yaw < -M_PI) {
        yaw += 2 * M_PI;
    }
    return yaw;
}

// Given x, y positions and an initial guess s0, find the closest s value
double CubicSpline2D::find_s(double x, double y, double s0) {
    double s_closest = s0;
    double closest = INFINITY;
    double si = s.front();

    do {
        if (si > s.back())
            si -= s.back();
        double px = calc_x(si);
        double py = calc_y(si);
        double dist = hypot(x - px, y - py);
        if (dist < closest) {
            closest = dist;
            s_closest = si;
        }
        if (dist < 0.01) {
            return s_closest;
        }
        si += 0.01;
    } while (si < s.back());
    return s_closest;
}
// Given x, y positions and an initial guess s0, find the closest s value
double CubicSpline2D::local_find_s(double x, double y, double s0, double search_range) {
    double s_closest = s0;
    double closest = INFINITY;
    double si = s0;
    double range = 0;
    do {
        if (si > s.back())
            si -= s.back();
        double px = calc_x(si);
        double py = calc_y(si);
        double dist = hypot(x - px, y - py);
        if (dist < closest) {
            closest = dist;
            s_closest = si;
        }
        if (dist < 0.01) {
            return s_closest;
        }
        range += 0.01;
        si = s0 + range;
    } while (range < search_range);
    return s_closest;
}

// Calculate the lateral error at given x, y, s
/**
 * @brief calculate the lateral deviation of the given point (x, y) from the global path
 *
 * @param x : x position of the given point
 * @param y : y position of the given point
 * @param s0 : spline length of the given point
 * @return double : lateral deviation of the given point from the global path
 * + : right side of the global path
 * - : left side of the global path
 */
double CubicSpline2D::calc_lateral_deviation(double x, double y, double s0) {
    double s_closest = s0; // find_s(x, y, s0);
    double px = calc_x(s0);
    double py = calc_y(s0);
    // double lat_error = hypot(x - px, y - py);
    //  calculate the sign of the lateral error
    double dx_dtheta = calc_xdot(s0);
    double dy_dtheta = calc_ydot(s0);

    double lateral_error = (x - px) * dy_dtheta - (y - py) * dx_dtheta;

    // Vector2d(dy_dtheta, -dx_dtheta).normalized()

    return lateral_error;
}

// Remove any collinear points from given list of points by the triangle rule
vector<vector<double>> CubicSpline2D::remove_collinear_points(const vector<geometry_msgs::PointStamped> &global_path) {
    vector<vector<double>> filtered_points;
    vector<double> x_, y_;
    x_.push_back(global_path[0].point.x);
    x_.push_back(global_path[1].point.x);
    y_.push_back(global_path[0].point.y);
    y_.push_back(global_path[1].point.y);
    for (size_t i = 2; i < global_path.size() - 1; i++) {
        bool collinear = are_collinear(global_path[i - 2].point.x, global_path[i - 2].point.y, global_path[i - 1].point.x, global_path[i - 1].point.y,
                                       global_path[i].point.x, global_path[i].point.y);
        if (collinear) {
            continue;
        }
        x_.push_back(global_path[i].point.x);
        y_.push_back(global_path[i].point.y);
    }
    // make sure to add the last point in case all points are collinear
    x_.push_back(global_path.back().point.x);
    y_.push_back(global_path.back().point.y);
    filtered_points.push_back(x_);
    filtered_points.push_back(y_);
    return filtered_points;
}

// Determine if 3 points are collinear using the triangle area rule
bool CubicSpline2D::are_collinear(double x1, double y1, double x2, double y2, double x3, double y3) {
    double a = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
    return abs(a) <= 1.0e-10; // this value smaller -> bigger filtered points
}
