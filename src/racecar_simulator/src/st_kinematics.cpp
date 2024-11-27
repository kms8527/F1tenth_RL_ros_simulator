#include <cmath>

#include "racecar_simulator/car_state.hpp"
#include "racecar_simulator/st_kinematics.hpp"
#include <iostream>

using namespace racecar_simulator;

// Implementation based off of Single Track Dynamics defined in CommonRoad:
// Vehicle Models
// https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf

CarState STKinematics::update(const CarState start, double accel, double steer_angle_vel, CarParams p, double dt) {

    double thresh = 1.03; // cut off to avoid singular behavior
    double err = .03;     // deadband to avoid flip flop

    // if velocity is low or negative, use normal Kinematic Single Track
    // dynamics
    if (start.velocity < thresh) {
        return update_k(start, accel, steer_angle_vel, p, dt);
    }

    CarState end;

    double g = 9.81; // m/s^2

    // compute first derivatives of state
    double x_dot = start.velocity * std::cos(start.theta + start.slip_angle);
    double y_dot = start.velocity * std::sin(start.theta + start.slip_angle);
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
        (p.l_f * p.cs_f * start.steer_angle * (rear_val) + start.slip_angle * (p.l_r * p.cs_r * (front_val)-p.l_f * p.cs_f * (rear_val)) -
         vel_ratio * (std::pow(p.l_f, 2) * p.cs_f * (rear_val) + std::pow(p.l_r, 2) * p.cs_r * (front_val)));

    double slip_angle_dot = (first_term) * (p.cs_f * start.steer_angle * (rear_val)-start.slip_angle * (p.cs_r * (front_val) + p.cs_f * (rear_val)) +
                                            vel_ratio * (p.cs_r * p.l_r * (front_val)-p.cs_f * p.l_f * (rear_val))) -
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
}

CarState STKinematics::update_k(const CarState start, double accel, double steer_angle_vel, CarParams p, double dt) {

    CarState end;

    // compute first derivatives of state
    double x_dot = start.velocity * std::cos(start.theta);
    double y_dot = start.velocity * std::sin(start.theta);
    double v_dot = accel;
    double steer_angle_dot = steer_angle_vel;
    double theta_dot = start.velocity / p.wheelbase * std::tan(start.steer_angle);
    double theta_double_dot = accel / p.wheelbase * std::tan(start.steer_angle) +
                              start.velocity * steer_angle_vel / (p.wheelbase * std::pow(std::cos(start.steer_angle), 2));
    double slip_angle_dot = 0;

    // update state
    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.velocity = start.velocity + v_dot * dt;
    end.steer_angle = start.steer_angle + steer_angle_dot * dt;
    end.angular_velocity = 0; // start.angular_velocity + theta_double_dot * dt;
    end.slip_angle = 0;       // start.slip_angle + slip_angle_dot * dt;

    if (end.theta > M_PI)
        end.theta -= 2 * M_PI;
    else if (end.theta < -M_PI)
        end.theta += 2 * M_PI;

    return end;
}

CarState STKinematics::update_with_pacejka(const CarState start, double accel, double steer_angle_vel, const CarParams &p, double dt) {
    double thresh = 1.03; // cut off to avoid singular behavior
    double err = .03;     // deadband to avoid flip flop

    // if velocity is low or negative, use normal Kinematic Single Track
    // dynamics
    if (start.velocity < thresh) {
        return update_k(start, accel, steer_angle_vel, p, dt);
    }

    CarState end;

    double g = 9.81; // m/s^2

    // compute first derivatives of state
    double x_dot = start.velocity * std::cos(start.theta + start.slip_angle);
    double y_dot = start.velocity * std::sin(start.theta + start.slip_angle);
    double v_dot = accel;
    double steer_angle_dot = steer_angle_vel;
    double theta_dot = start.angular_velocity;

    // F_fy와 F_ry 계산
    double F_fy = getForceFront(start, p); // 전륜 측면력
    double F_ry = getForceRear(start, p);  // 후륜 측면력

    // theta_double_dot과 slip_angle_dot 계산
    double theta_double_dot = (p.l_f * F_fy * std::cos(start.steer_angle) - p.l_r * F_ry) / p.I_z;
    double slip_angle_dot = ((F_fy + F_ry) / (p.mass * start.velocity)) - start.angular_velocity;

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
}

double STKinematics::getForceFront(const CarState &state, const CarParams &p) {

    // 슬립 각도를 라디안으로 변환
    double alpha = getFrontSlipAngle(state, p);

    // D * sin(C * arctan(B * alpha)) 형태로 측면 힘을 계산
    return p.D * std::sin(p.C * std::atan(p.B * alpha));
}

double STKinematics::getForceRear(const CarState &state, const CarParams &p) {

    // 슬립 각도를 라디안으로 변환
    double alpha = getRearSlipAngle(state, p);
    return p.D * std::sin(p.C * std::atan(p.B * alpha));
}

double STKinematics::getFrontSlipAngle(const CarState &state, const CarParams &p) {
    double vx = state.velocity * std::cos(state.slip_angle);
    double vy = state.velocity * std::sin(state.slip_angle);
    double front_slip_angle = -std::atan2(vy + p.l_f * state.angular_velocity, vx) + state.steer_angle;
    return front_slip_angle;
}

double STKinematics::getRearSlipAngle(const CarState &state, const CarParams &p) {
    double vx = state.velocity * std::cos(state.slip_angle);
    double vy = state.velocity * std::sin(state.slip_angle);
    double rear_slip_angle = -std::atan2(vy - p.l_r * state.angular_velocity, vx);
    return rear_slip_angle;
}