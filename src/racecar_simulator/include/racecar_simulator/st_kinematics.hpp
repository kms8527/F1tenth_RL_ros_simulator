#pragma once

#include "racecar_simulator/car_params.hpp"
#include "racecar_simulator/car_state.hpp"

#include <Eigen/Dense>

namespace racecar_simulator {

class STKinematics {
  public:
    static CarState update(const CarState start, double accel,
                           double steer_angle_vel, CarParams p, double dt);

    static CarState update_k(const CarState start, double accel,
                             double steer_angle_vel, CarParams p, double dt);

    static CarState update_with_pacejka(const CarState start, double accel,
                                        double steer_angle_vel,
                                        const CarParams &p, double dt);
    // static CarState RK4(const CarState &x, const double accel,
    //                     const double steer, const double ts, const CarParams
    //                     p);
    static double getForceFront(const CarState &state, const CarParams &p);
    static double getForceRear(const CarState &state, const CarParams &p);

    static double getFrontSlipAngle(const CarState &state, const CarParams &p);
    static double getRearSlipAngle(const CarState &state, const CarParams &p);
};

} // namespace racecar_simulator
