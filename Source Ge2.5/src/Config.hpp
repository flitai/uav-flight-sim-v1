#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Eigen/Dense>

// This file holds simulation constants and UAV parameters. For a real application, consider loading these from a JSON/YAML file as suggested in your document.

namespace Config {
    // Simulation Parameters
    constexpr double SIMULATION_STEP_SIZE = 0.01; // s (inner loop and dynamics)
    constexpr double OUTER_LOOP_DT = 0.05;        // s (20 Hz)
    constexpr double MIDDLE_LOOP_DT = 0.02;       // s (50 Hz)
    constexpr double INNER_LOOP_DT = 0.01;        // s (100 Hz)

    // Physical constants
    constexpr double GRAVITY = 9.80665; // m/s^2

    // UAV Parameters (example values, replace with actual data)
    constexpr double MASS = 1.5; // kg
    const Eigen::Matrix3d INERTIA_TENSOR = (Eigen::Matrix3d() <<
        0.1, 0.0, 0.0,
        0.0, 0.2, 0.0,
        0.0, 0.0, 0.25).finished(); // kg*m^2 (Ixx, Iyy, Izz on diagonal for simplicity)

    // Aerodynamic Reference Values (example)
    constexpr double WING_AREA = 0.5;   // m^2 (S)
    constexpr double WING_SPAN = 2.0;   // m (b)
    constexpr double MEAN_AERO_CHORD = 0.25; // m (c_bar)
    constexpr double AIR_DENSITY = 1.225; // kg/m^3 (rho, at sea level)

    // Aerodynamic Coefficients (from document Table 3.3)
    // Longitudinal
    constexpr double C_L0 = 0.2;         //
    constexpr double C_L_alpha = 5.7;    // 1/rad
    constexpr double C_D0 = 0.02;        //
    constexpr double K_induced_drag = 0.066; // (k for CD = C_D0 + k*C_L^2)
    constexpr double C_m0 = 0.05;        //
    constexpr double C_m_alpha = -0.38;  // 1/rad
    constexpr double C_L_delta_e = 0.8;  // 1/rad
    constexpr double C_m_delta_e = -1.1; // 1/rad
    // Placeholder: Damping derivatives often important but not in simple table
    constexpr double C_L_q = 0.0; // Typically non-zero, e.g., 7.0
    constexpr double C_m_q = 0.0; // Typically non-zero, e.g., -12.0

    // Lateral-Directional (Placeholders - these need to be defined for full 6DOF)
    // Side force
    constexpr double C_Y0 = 0.0;
    constexpr double C_Y_beta = -0.83;    // Example value
    constexpr double C_Y_p = 0.0;
    constexpr double C_Y_r = 0.0;
    constexpr double C_Y_delta_a = 0.0;
    constexpr double C_Y_delta_r = 0.2;   // Example value (rudder side force)

    // Rolling moment
    constexpr double C_l0 = 0.0;
    constexpr double C_l_beta = -0.12;   // Example value
    constexpr double C_l_p = -0.5;       // Example value (roll damping)
    constexpr double C_l_r = 0.1;        // Example value
    constexpr double C_l_delta_a = 0.25; // Example value (aileron roll moment)
    constexpr double C_l_delta_r = 0.01; // Example value

    // Yawing moment
    constexpr double C_n0 = 0.0;
    constexpr double C_n_beta = 0.15;    // Example value (weathercock stability)
    constexpr double C_n_p = -0.05;      // Example value
    constexpr double C_n_r = -0.2;       // Example value (yaw damping)
    constexpr double C_n_delta_a = 0.01; // Example value (aileron adverse yaw)
    constexpr double C_n_delta_r = -0.1; // Example value (rudder yaw moment)

    // Propulsion
    constexpr double MAX_THRUST = 10.0; // Newtons

    // Control Surface Limits (radians)
    constexpr double MAX_AILERON_DEFLECTION = 30.0 * M_PI / 180.0;
    constexpr double MAX_ELEVATOR_DEFLECTION = 25.0 * M_PI / 180.0;
    constexpr double MAX_RUDDER_DEFLECTION = 30.0 * M_PI / 180.0;

    // PID Gains (from document Section 4)
    // Outer Loop
    constexpr double K_p_psi = 1.2;    // Heading to Roll Command (phi_d)
    constexpr double K_i_psi = 0.01;   //
    constexpr double K_p_h = 0.8;      // Altitude to Pitch Command (theta_d)
    constexpr double K_i_h = 0.005;    //
    constexpr double K_p_V = 0.5;      // Airspeed to Thrust Command
    constexpr double K_i_V = 0.02;     //
    constexpr double K_p_psi_r = 0.5;  // Yaw tracking to Yaw Rate Command (r_d) (If used)
    constexpr double K_i_psi_r = 0.02; //

    // Middle Loop (Attitude Hold)
    constexpr double K_p_phi = 5.5;    // Roll angle to Roll Rate Command (p_d)
    constexpr double K_d_phi = 1.2;    //
    constexpr double K_p_theta = 6.0;  // Pitch angle to Pitch Rate Command (q_d)
    constexpr double K_i_theta = 0.2;  //
    constexpr double K_d_theta = 1.0;  //

    // Inner Loop (Rate Damping)
    constexpr double K_p_p = 8.0;      // Roll rate to Aileron (delta_a)
    constexpr double K_d_p = 1.5;      //
    constexpr double K_p_q = 9.0;      // Pitch rate to Elevator (delta_e)
    constexpr double K_d_q = 1.8;      //
    constexpr double K_p_r = 4.0;      // Yaw rate to Rudder (delta_r)
    constexpr double K_d_r = 0.5;      //

} // namespace Config

#endif // CONFIG_HPP