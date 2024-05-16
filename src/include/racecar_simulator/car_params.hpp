#pragma once

namespace racecar_simulator {

struct CarParams {
    double wheelbase;
    double friction_coeff;
    double h_cg; // height of car's CG
    double l_f; // length from CG to front axle
    double l_r; // length from CG to rear axle
    double cs_f; // cornering stiffness coeff for front wheels
    double cs_r; // cornering stiffness coeff for rear wheels
    double mass;
    double I_z; // moment of inertia about z axis from CG

    double B; // Pacejka tire model parameter : stiffness factor
    double C; // Pacejka tire model parameter : shape factor
    double D; // Pacejka tire model parameter : peak value of longitudinal force
};

}
