#include "DynamicsModel.hpp"
#include <cmath> // For sin, cos, tan
#include <iostream> // For debug


SixDOFModel::SixDOFModel() {
    // Initialize with a default state (e.g., on ground or specific altitude)
    _current_state.position_earth = Eigen::Vector3d(0, 0, -100); // Example: 100m altitude (NED coordinates)
    _current_state.velocity_body = Eigen::Vector3d(20, 0, 0); // Example: 20 m/s forward speed
}

void SixDOFModel::initializeState(const UAVState& initial_state) {
    _current_state = initial_state;
}

const UAVState& SixDOFModel::getCurrentState() const {
    return _current_state;
}

Eigen::Matrix3d SixDOFModel::getRotationMatrixBodyToEarth(const Eigen::Vector3d& euler_angles) {
    double phi = euler_angles.x();
    double theta = euler_angles.y();
    double psi = euler_angles.z();

    Eigen::Matrix3d R_roll;
    R_roll << 1, 0, 0,
              0, cos(phi), sin(phi),
              0, -sin(phi), cos(phi);

    Eigen::Matrix3d R_pitch;
    R_pitch << cos(theta), 0, -sin(theta),
               0, 1, 0,
               sin(theta), 0, cos(theta);

    Eigen::Matrix3d R_yaw;
    R_yaw << cos(psi), sin(psi), 0,
             -sin(psi), cos(psi), 0,
             0, 0, 1;
    // For ZYX sequence (Yaw, Pitch, Roll) to transform from Body to Earth (NED)
    // R_earth_to_body = R_roll.transpose() * R_pitch.transpose() * R_yaw.transpose()
    // R_body_to_earth = R_yaw * R_pitch * R_roll
    // However, the common aerospace sequence is Yaw-Pitch-Roll for body to inertial
    Eigen::Matrix3d R_b_e; // Body to Earth
    double cphi = cos(phi), sphi = sin(phi);
    double cth = cos(theta), sth = sin(theta);
    double cpsi = cos(psi), spsi = sin(psi);

    R_b_e(0,0) = cth*cpsi;
    R_b_e(0,1) = sphi*sth*cpsi - cphi*spsi;
    R_b_e(0,2) = cphi*sth*cpsi + sphi*spsi;

    R_b_e(1,0) = cth*spsi;
    R_b_e(1,1) = sphi*sth*spsi + cphi*cpsi;
    R_b_e(1,2) = cphi*sth*spsi - sphi*cpsi;

    R_b_e(2,0) = -sth;
    R_b_e(2,1) = sphi*cth;
    R_b_e(2,2) = cphi*cth;
    return R_b_e;
}

Eigen::Matrix3d SixDOFModel::getEulerRatesMatrix(const Eigen::Vector3d& euler_angles) {
    double phi = euler_angles.x();
    double theta = euler_angles.y();
    // Standard transformation from body rates (p,q,r) to Euler rates (phi_dot, theta_dot, psi_dot)
    // [phi_dot; theta_dot; psi_dot] = E * [p; q; r]
    Eigen::Matrix3d E;
    double cphi = cos(phi), sphi = sin(phi);
    double tth = tan(theta), scth = 1.0/cos(theta); // sec(theta)

    if (std::abs(cos(theta)) < 1e-6) { // Singularity at theta = +/- 90 deg
        // Handle singularity, e.g., by using quaternions or limiting pitch
        // For now, let's assume it won't be hit perfectly or use a very large number for tan/sec
        scth = 1e6; // A large number to avoid NaN, but this indicates an issue
        tth = sin(theta) * scth;
    }

    E << 1, sphi * tth, cphi * tth,
         0, cphi,       -sphi,
         0, sphi * scth, cphi * scth;
    return E;
}


Eigen::Vector3d SixDOFModel::gravityForceBody(const Eigen::Vector3d& euler_angles) {
    Eigen::Matrix3d R_b_e = getRotationMatrixBodyToEarth(euler_angles);
    Eigen::Vector3d gravity_earth(0, 0, Config::MASS * Config::GRAVITY); // NED: Z-axis is down
    return R_b_e.transpose() * gravity_earth; // R_e_b = R_b_e^T
}

void SixDOFModel::calculateAerodynamics(const UAVState& state, const ControlInputs& controls,
                                       Eigen::Vector3d& forces_body, Eigen::Vector3d& moments_body) {
    // Unpack state
    const Eigen::Vector3d& v_body = state.velocity_body; // u, v, w
    const Eigen::Vector3d& ang_rates = state.angular_rates_body; // p, q, r

    double u = v_body.x();
    double v = v_body.y();
    double w = v_body.z();

    double p = ang_rates.x();
    double q = ang_rates.y();
    double r = ang_rates.z();

    double delta_e = controls.delta_e;
    double delta_a = controls.delta_a;
    double delta_r = controls.delta_r;

    // Airspeed and angles
    double V_a = v_body.norm(); // Airspeed
    if (V_a < 0.1) { // Avoid division by zero at low speeds
        forces_body.setZero();
        moments_body.setZero();
        return;
    }

    double alpha = atan2(w, u); // Angle of attack
    double beta = asin(v / V_a);  // Sideslip angle (small angle approx: v/V_a, more general: asin(v/Va))

    // Dynamic pressure
    double q_bar = 0.5 * Config::AIR_DENSITY * V_a * V_a;

    // Longitudinal coefficients
    double C_L = Config::C_L0 + Config::C_L_alpha * alpha + Config::C_L_delta_e * delta_e;
    // Add C_L_q term if available: + Config::C_L_q * (Config::MEAN_AERO_CHORD / (2 * V_a)) * q;
    // Simplified: use C_L_q without normalization factor first
    C_L += Config::C_L_q * q;


    double C_D = Config::C_D0 + Config::K_induced_drag * C_L * C_L; // Parabolic drag polar

    double C_m = Config::C_m0 + Config::C_m_alpha * alpha + Config::C_m_delta_e * delta_e;
    // Add C_m_q term if available: + Config::C_m_q * (Config::MEAN_AERO_CHORD / (2 * V_a)) * q;
    C_m += Config::C_m_q * q;


    // Lateral-directional coefficients (using placeholder values from Config)
    // Normalize angular rates for aero derivatives (p_hat, q_hat, r_hat)
    double p_hat = (V_a > 1.0) ? p * Config::WING_SPAN / (2.0 * V_a) : 0.0;
    double r_hat = (V_a > 1.0) ? r * Config::WING_SPAN / (2.0 * V_a) : 0.0;

    double C_Y = Config::C_Y0 + Config::C_Y_beta * beta + Config::C_Y_p * p_hat + Config::C_Y_r * r_hat +
                 Config::C_Y_delta_a * delta_a + Config::C_Y_delta_r * delta_r;

    double C_l = Config::C_l0 + Config::C_l_beta * beta + Config::C_l_p * p_hat + Config::C_l_r * r_hat +
                 Config::C_l_delta_a * delta_a + Config::C_l_delta_r * delta_r;

    double C_n = Config::C_n0 + Config::C_n_beta * beta + Config::C_n_p * p_hat + Config::C_n_r * r_hat +
                 Config::C_n_delta_a * delta_a + Config::C_n_delta_r * delta_r;


    // Aerodynamic forces (Stability Axes -> Body Axes)
    // Assuming Lift (L) and Drag (D) are initially computed in stability axes, then rotated.
    // Or, if C_L, C_D are already body-frame coeffs (X,Z direction), then it's simpler.
    // The common approach: L is perp to V_a, D is parallel to V_a.
    // F_x_stability = -D, F_z_stability = -L
    // Rotate by alpha to get to body frame:
    // F_x_body = F_x_stability * cos(alpha) - F_z_stability * sin(alpha)
    // F_z_body = F_x_stability * sin(alpha) + F_z_stability * cos(alpha)
    // Lift = q_bar * S * C_L, Drag = q_bar * S * C_D, SideForce = q_bar * S * C_Y
    double Lift = q_bar * Config::WING_AREA * C_L;
    double Drag = q_bar * Config::WING_AREA * C_D;
    double SideForce = q_bar * Config::WING_AREA * C_Y;

    // Assuming forces are in wind frame initially (X_w along V, Z_w down perp to V)
    // F_x_wind = -Drag, F_y_wind = SideForce, F_z_wind = -Lift
    // Transform wind to body frame (alpha for pitch, beta for yaw)
    // Simplified: Assume C_L, C_D contributions are mainly to body Z and X forces
    // This is a common simplification where C_L acts primarily as -Z_body and C_D as -X_body
    // This is not entirely accurate without full stability-to-body transformation for forces.
    // For a more standard model:
    // F_X = q_bar * S * ( C_L * sin(alpha) - C_D * cos(alpha) ) (if alpha is positive up)
    // F_Z = q_bar * S * (-C_L * cos(alpha) - C_D * sin(alpha) )
    // F_Y = q_bar * S * C_Y
    // Here, following Stevens & Lewis / Beard & McLain type representation for forces in body frame:
    // F_ax = -Drag * cos(alpha) + Lift * sin(alpha)
    // F_az = -Drag * sin(alpha) - Lift * cos(alpha)
    // F_ay = SideForce
    forces_body.x() = q_bar * Config::WING_AREA * (C_L * sin(alpha) - C_D * cos(alpha) + 0); // Simplified C_X
    forces_body.y() = q_bar * Config::WING_AREA * C_Y;
    forces_body.z() = q_bar * Config::WING_AREA * (-C_L * cos(alpha) - C_D * sin(alpha) + 0); // Simplified C_Z

    // Aerodynamic moments (already in body frame by definition of C_l, C_m, C_n)
    moments_body.x() = q_bar * Config::WING_AREA * Config::WING_SPAN * C_l;   // L_moment
    moments_body.y() = q_bar * Config::WING_AREA * Config::MEAN_AERO_CHORD * C_m; // M_moment
    moments_body.z() = q_bar * Config::WING_AREA * Config::WING_SPAN * C_n;   // N_moment
}


Eigen::Matrix<double, 12, 1> SixDOFModel::stateDerivative(const UAVState& state, const ControlInputs& controls) {
    Eigen::Matrix<double, 12, 1> x_dot;

    // Unpack state variables
    const Eigen::Vector3d& pos_earth = state.position_earth;
    const Eigen::Vector3d& vel_body = state.velocity_body; // u, v, w
    const Eigen::Vector3d& euler = state.euler_angles;   // phi, theta, psi
    const Eigen::Vector3d& ang_rates = state.angular_rates_body; // p, q, r

    // --- Forces ---
    // Aerodynamic forces
    Eigen::Vector3d aero_forces_body;
    Eigen::Vector3d aero_moments_body;
    calculateAerodynamics(state, controls, aero_forces_body, aero_moments_body);

    // Gravity force
    Eigen::Vector3d gravity_force_b = gravityForceBody(euler);

    // Propulsion force (assuming along body x-axis)
    Eigen::Vector3d thrust_force_body(controls.thrust * Config::MAX_THRUST, 0, 0);

    Eigen::Vector3d total_forces_body = aero_forces_body + gravity_force_b + thrust_force_body;

    // --- Translational Dynamics (Body Frame Velocities) ---
    // m * (v_dot_body + omega_body x v_body) = F_body
    // v_dot_body = F_body / m - omega_body x v_body
    Eigen::Vector3d v_dot_body = (total_forces_body / Config::MASS) - ang_rates.cross(vel_body);
    x_dot.segment<3>(3) = v_dot_body; // du/dt, dv/dt, dw/dt

    // --- Rotational Dynamics ---
    // I * omega_dot_body + omega_body x (I * omega_body) = M_body
    // omega_dot_body = I_inv * (M_body - omega_body x (I * omega_body))
    Eigen::Vector3d total_moments_body = aero_moments_body;
    // Add propulsion moments if any (e.g. M_t) - assuming zero for now

    Eigen::Matrix3d I = Config::INERTIA_TENSOR;
    // If I is diagonal, I_inv is just 1/Ixx, 1/Iyy, 1/Izz. For general I, use inverse().
    Eigen::Matrix3d I_inv = I.inverse(); // Cache if I is constant and performance critical

    Eigen::Vector3d ang_accel_body = I_inv * (total_moments_body - ang_rates.cross(I * ang_rates));
    x_dot.segment<3>(9) = ang_accel_body; // dp/dt, dq/dt, dr/dt

    // --- Kinematics ---
    // Position derivative (Earth frame)
    Eigen::Matrix3d R_b_e = getRotationMatrixBodyToEarth(euler);
    Eigen::Vector3d pos_dot_earth = R_b_e * vel_body;
    x_dot.segment<3>(0) = pos_dot_earth; // dx/dt, dy/dt, dz/dt

    // Euler angle rates derivative
    Eigen::Matrix3d E_matrix = getEulerRatesMatrix(euler);
    Eigen::Vector3d euler_rates_dot = E_matrix * ang_rates;
    x_dot.segment<3>(6) = euler_rates_dot; // dphi/dt, dtheta/dt, dpsi/dt

    return x_dot;
}

void SixDOFModel::updateState(const ControlInputs& controls, double dt) {
    // RK4 Integration
    Eigen::Matrix<double, 12, 1> x0 = _current_state.getFullState();

    Eigen::Matrix<double, 12, 1> k1 = stateDerivative(_current_state, controls);

    UAVState temp_state_k2 = _current_state;
    temp_state_k2.setFromFullState(x0 + 0.5 * dt * k1);
    Eigen::Matrix<double, 12, 1> k2 = stateDerivative(temp_state_k2, controls);

    UAVState temp_state_k3 = _current_state;
    temp_state_k3.setFromFullState(x0 + 0.5 * dt * k2);
    Eigen::Matrix<double, 12, 1> k3 = stateDerivative(temp_state_k3, controls);

    UAVState temp_state_k4 = _current_state;
    temp_state_k4.setFromFullState(x0 + dt * k3);
    Eigen::Matrix<double, 12, 1> k4 = stateDerivative(temp_state_k4, controls);

    Eigen::Matrix<double, 12, 1> x_new = x0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    _current_state.setFromFullState(x_new);

    // Normalize Euler angles (optional, but good practice for psi and phi)
    _current_state.euler_angles.x() =remainder(_current_state.euler_angles.x(), 2.0 * M_PI); // Phi to +/- PI (using remainder for -pi to pi)
    _current_state.euler_angles.z() =remainder(_current_state.euler_angles.z(), 2.0 * M_PI); // Psi to +/- PI

    // Pitch angle theta should ideally be kept within -pi/2 to pi/2 to avoid gimbal lock issues with Euler rates
    // if (_current_state.euler_angles.y() > M_PI / 2.0) _current_state.euler_angles.y() = M_PI / 2.0 - 1e-6;
    // if (_current_state.euler_angles.y() < -M_PI / 2.0) _current_state.euler_angles.y() = -M_PI / 2.0 + 1e-6;
}