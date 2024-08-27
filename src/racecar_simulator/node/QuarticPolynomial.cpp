#include "QuarticPolynomial.h"

#include <Eigen/LU>
#include <cmath>

using namespace Eigen;

/**
 * @brief Construct a new Quartic Polynomial:: Quartic Polynomial object
 *
 * @param xs : start point
 * @param vxs : start velocity
 * @param axs : start acceleration
 * @param vxe : end velocity
 * @param axe : end acceleration
 * @param t : time
 */
QuarticPolynomial::QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double t) : a0(xs), a1(vxs) {
    a2 = axs / 2.0;
    Matrix2d A;
    Vector2d B;
    A << 3 * pow(t, 2), 4 * pow(t, 3), 6 * t, 12 * pow(t, 2);
    B << vxe - a1 - 2 * a2 * t, axe - 2 * a2;
    Matrix2d A_inv = A.inverse();
    Vector2d x = A_inv * B;
    a3 = x[0];
    a4 = x[1];
}

/**
 * @brief Calculate the point on the quartic polynomial at time t
 *
 * @param t : time
 * @return double
 */
double QuarticPolynomial::calc_point(double t) { return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4); }

/**
 * @brief Calculate the first derivative of the quartic polynomial at time t
 *
 * @param t : time
 * @return double
 */
double QuarticPolynomial::calc_first_derivative(double t) { return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3); }

/**
 * @brief Calculate the second derivative of the quartic polynomial at time t
 *
 * @param t : time
 * @return double
 */
double QuarticPolynomial::calc_second_derivative(double t) { return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2); }

/**
 * @brief Calculate the third derivative of the quartic polynomial at time t
 *
 * @param t : time
 * @return double
 */
double QuarticPolynomial::calc_third_derivative(double t) { return 6 * a3 + 24 * a4 * t; }