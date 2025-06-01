#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <Eigen/Dense>

// This file will define the basic data structures used throughout the simulation.

// State vector: [x, y, z, u, v, w, phi, theta, psi, p, q, r]^T
// x, y, z: inertial positions
// u, v, w: body frame velocities
// phi, theta, psi: Euler angles (roll, pitch, yaw)
// p, q, r: body frame angular rates
struct UAVState {
    Eigen::Vector3d position_earth;      // [x, y, z] in Earth frame
    Eigen::Vector3d velocity_body;       // [u, v, w] in body frame
    Eigen::Vector3d euler_angles;        // [phi, theta, psi]
    Eigen::Vector3d angular_rates_body;  // [p, q, r] in body frame

    // Convenience constructor
    UAVState() {
        position_earth.setZero();
        velocity_body.setZero();
        euler_angles.setZero();
        angular_rates_body.setZero();
    }

    // Full state vector (12x1) for derivative calculations if needed
    Eigen::Matrix<double, 12, 1> getFullState() const {
        Eigen::Matrix<double, 12, 1> x;
        x.segment<3>(0) = position_earth;
        x.segment<3>(3) = velocity_body;
        x.segment<3>(6) = euler_angles;
        x.segment<3>(9) = angular_rates_body;
        return x;
    }

    void setFromFullState(const Eigen::Matrix<double, 12, 1>& x) {
        position_earth = x.segment<3>(0);
        velocity_body = x.segment<3>(3);
        euler_angles = x.segment<3>(6);
        angular_rates_body = x.segment<3>(9);
    }
};

// Control inputs from the innermost loop to the dynamics model
struct ControlInputs {
    double thrust;     // Normalized thrust (e.g., 0.0 to 1.0) or actual force
    double delta_a;    // Aileron deflection (radians)
    double delta_e;    // Elevator deflection (radians)
    double delta_r;    // Rudder deflection (radians)

    ControlInputs() : thrust(0.0), delta_a(0.0), delta_e(0.0), delta_r(0.0) {}
};

// Desired values for the outer loop
struct FlightPathCommand {
    double desired_altitude;   // m
    double desired_airspeed;   // m/s
    double desired_heading;    // radians (psi_d)
    // Potentially add waypoint (x,y) for more advanced navigation
};

// Outputs from outer loop controller
struct OuterLoopCommands {
    double desired_roll_phi;    // radians (phi_d)
    double desired_pitch_theta; // radians (theta_d)
    double desired_yaw_rate_r;  // rad/s (r_d)
    double thrust_command;      // command for propulsion system (e.g. normalized, or direct force)
};

// Outputs from middle loop controller (desired angular rates)
struct AttitudeRateCommands {
    double desired_roll_rate_p;   // rad/s (p_d)
    double desired_pitch_rate_q;  // rad/s (q_d)
    double desired_yaw_rate_r;    // rad/s (r_d) - can be passed from outer loop or modified
};

#endif // COMMON_TYPES_HPP