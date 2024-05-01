#include "models.h"
#include <dynamic_cast>

// Moel Reference URL : https://arxiv.org/pdf/2403.11648

// ODEModel 업데이트 로직 구현
ODEState ODEModel::update(IState &state) {
    // state를 ODEState로 안전하게 캐스팅합니다.
    // 캐스팅 실패 시 예외 처리가 필요합니다.
    try {
        ODEState &ODEState = dynamic_cast<ODEState &>(state);
        ODEState end;

        double thresh = 1.03; // cut off to avoid singular behavior
        double err = .03;     // deadband to avoid flip flop

        // if velocity is low or negative, use normal Kinematic Single Track
        // dynamics
        if (start.velocity < thresh) {
            return update_k(start, accel, steer_angle_vel, p, dt);
        }

        double g = 9.81; // m/s^2

        // compute first derivatives of state
        double x_dot =
            start.velocity * std::cos(start.theta + start.slip_angle);
        double y_dot =
            start.velocity * std::sin(start.theta + start.slip_angle);
        double v_dot = accel;
        double steer_angle_dot = steer_angle_vel;
        double theta_dot = start.angular_velocity;

        // for eases of next two calculations
        double rear_val = g * p.l_r - accel * p.h_cg;
        double front_val = g * p.l_f + accel * p.h_cg;

        // in case velocity is 0
        double vel_ratio, first_term;
        if (start.velocity == 0) {
            vel_ratio = 0;
            first_term = 0;
        } else {
            vel_ratio = start.angular_velocity / start.velocity;
            first_term = p.friction_coeff / (start.velocity * (p.l_r + p.l_f));
        }

        double theta_double_dot =
            (p.friction_coeff * p.mass / (p.I_z * p.wheelbase)) *
            (p.l_f * p.cs_f * start.steer_angle * (rear_val) +
             start.slip_angle *
                 (p.l_r * p.cs_r * (front_val)-p.l_f * p.cs_f * (rear_val)) -
             vel_ratio * (std::pow(p.l_f, 2) * p.cs_f * (rear_val) +
                          std::pow(p.l_r, 2) * p.cs_r * (front_val)));

        double slip_angle_dot =
            (first_term) *
                (p.cs_f * start.steer_angle * (rear_val)-start.slip_angle *
                     (p.cs_r * (front_val) + p.cs_f * (rear_val)) +
                 vel_ratio * (p.cs_r * p.l_r * (front_val)-p.cs_f * p.l_f *
                              (rear_val))) -
            start.angular_velocity;

        // update state
        end.x = start.x + x_dot * dt;
        end.y = start.y + y_dot * dt;
        end.theta = start.theta + theta_dot * dt;
        end.velocity = start.velocity + v_dot * dt;
        end.steer_angle = start.steer_angle + steer_angle_dot * dt;
        end.angular_velocity = start.angular_velocity + theta_double_dot * dt;
        end.slip_angle = start.slip_angle + slip_angle_dot * dt;

        if (end.theta > M_PI)
            end.theta -= 2 * M_PI;
        else if (end.theta < -M_PI)
            end.theta += 2 * M_PI;

        return end;

    } catch (const std::bad_cast &e) {
        // 캐스팅 실패 처리
        throw std::runtime_error("State casting failed in ODEModel::update");
    }
}

// PacejkaModel 업데이트 로직 구현
PacejkaState PacejkaModel::update(IState &state) {
    // state를 PacejkaState로 안전하게 캐스팅합니다.
    // 캐스팅 실패 시 예외 처리가 필요합니다.
    try {
        PacejkaState &pacejkaState = dynamic_cast<PacejkaState &>(state);
        // 여기에 PacejkaModel의 상태 업데이트 로직을 구현합니다.
        // 예를 들어, slip angle, s, vs와 같은 파라미터를 업데이트 할 수
        // 있습니다. 예시: pacejkaState.s += someCalculationFunction();
    } catch (const std::bad_cast &e) {
        // 캐스팅 실패 처리
        throw std::runtime_error(
            "State casting failed in PacejkaModel::update");
    }
}
