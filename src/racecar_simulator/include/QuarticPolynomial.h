#ifndef FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H

class QuarticPolynomial {
public:
    QuarticPolynomial() = default;
    QuarticPolynomial(double xs, double vxs, double axs, double vxe,
                      double axe, double t);
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
private:
    double a0, a1, a2, a3, a4; // 4차 다항식 계수 // a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4
};

#endif //FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H
