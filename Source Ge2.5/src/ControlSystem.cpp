#include "ControlSystem.hpp"
#include <cmath> // For fmod, M_PI
#include <algorithm> // For std::clamp

// Helper to normalize angle difference to [-PI, PI]
double normalizeAngleError(double error) {
    error = fmod(error + M_PI, 2.0 * M_PI);
    if (error < 0) error += 2.0 * M_PI;
    return error - M_PI;
}


OuterLoopController::OuterLoopController() :
    _heading_to_roll_pid(Config::K_p_psi, Config::K_i_psi, 0.0,          // K_d for heading is 0
                         -45.0 * M_PI / 180.0, 45.0 * M_PI / 180.0,  // Roll command limits (+/- 45 deg)
                         -Config::K_i_psi * 5.0, Config::K_i_psi * 5.0), // Integral limits
    _altitude_to_pitch_pid(Config::K_p_h, Config::K_i_h, 0.0,           // K_d for altitude is 0
                           -30.0 * M_PI / 180.0, 30.0 * M_PI / 180.0, // Pitch command limits (+/- 30 deg)
                           -Config::K_i_h * 10.0, Config::K_i_h * 10.0),
    _airspeed_to_thrust_pid(Config::K_p_V, Config::K_i_V, 0.0,          // K_d for airspeed is 0
                            0.0, 1.0,                                  // Thrust command limits (normalized 0-1)
                            -Config::K_i_V * 10.0, Config::K_i_V * 10.0),
    _yaw_err_to_yaw_rate_pid(Config::K_p_psi_r, Config::K_i_psi_r, 0.0, // K_d for yaw rate is 0
                             -45.0 * M_PI / 180.0, 45.0 * M_PI / 180.0, // Yaw rate command limits (+/- 45 deg/s)
                             -Config::K_i_psi_r * 5.0, Config::K_i_psi_r * 5.0)
{}

OuterLoopCommands OuterLoopController::update(const FlightPathCommand& targets, const UAVState& current_state, double dt) {
    OuterLoopCommands cmds;

    // Heading control (generates desired roll angle phi_d)
    double heading_error = normalizeAngleError(targets.desired_heading - current_state.euler_angles.z()); // psi_d - psi
    cmds.desired_roll_phi = _heading_to_roll_pid.update(0.0, -heading_error, dt); // Control to drive error to 0

    // Altitude control (generates desired pitch angle theta_d)
    double altitude = -current_state.position_earth.z(); // Assuming NED, so z is depth. altitude = -z.
    double altitude_error = targets.desired_altitude - altitude; // h_d - h
    cmds.desired_pitch_theta = _altitude_to_pitch_pid.update(0.0, -altitude_error, dt);

    // Airspeed control (generates thrust command)
    double current_airspeed = current_state.velocity_body.norm(); // Or just use u if assuming no wind and small alpha/beta
    if (current_state.velocity_body.x() > 0) { // use u if positive, otherwise norm.
        current_airspeed = current_state.velocity_body.x();
    }
    double airspeed_error = targets.desired_airspeed - current_airspeed; // V_d - V
    cmds.thrust_command = _airspeed_to_thrust_pid.update(0.0, -airspeed_error, dt);

    // Yaw tracking (generates desired yaw rate r_d) (Can be used for coordinated turns or direct yaw control)
    // The document calls this "偏航跟踪" and provides K_p_psi_r, K_i_psi_r.
    // This might be intended for something like commanding a yaw rate to achieve the heading,
    // or for side-slip control. For now, using heading_error to command r_d.
    cmds.desired_yaw_rate_r = _yaw_err_to_yaw_rate_pid.update(0.0, -heading_error, dt);
    // Or, it could be a feed-forward term for turns: r_d = g * tan(phi_cmd) / V_a
    // For now, using the PID as per structure.

    return cmds;
}

void OuterLoopController::reset() {
    _heading_to_roll_pid.reset();
    _altitude_to_pitch_pid.reset();
    _airspeed_to_thrust_pid.reset();
    _yaw_err_to_yaw_rate_pid.reset();
}


MiddleLoopController::MiddleLoopController() :
    _roll_to_roll_rate_pid(Config::K_p_phi, 0.0, Config::K_d_phi, // Ki for roll is 0
                           -150.0 * M_PI / 180.0, 150.0 * M_PI / 180.0, // p_cmd limits (+/- 150 deg/s)
                           0.0, 0.0), // No integral
    _pitch_to_pitch_rate_pid(Config::K_p_theta, Config::K_i_theta, Config::K_d_theta,
                             -90.0 * M_PI / 180.0, 90.0 * M_PI / 180.0,  // q_cmd limits (+/- 90 deg/s)
                             -Config::K_i_theta * 2.0, Config::K_i_theta * 2.0) // Integral limits
{}

AttitudeRateCommands MiddleLoopController::update(const OuterLoopCommands& attitude_cmds, const UAVState& current_state, double dt) {
    AttitudeRateCommands rate_cmds;

    // Roll angle to roll rate command (phi_d -> p_d)
    double roll_error = normalizeAngleError(attitude_cmds.desired_roll_phi - current_state.euler_angles.x());
    rate_cmds.desired_roll_rate_p = _roll_to_roll_rate_pid.update(0.0, -roll_error, dt);

    // Pitch angle to pitch rate command (theta_d -> q_d)
    double pitch_error = normalizeAngleError(attitude_cmds.desired_pitch_theta - current_state.euler_angles.y());
    rate_cmds.desired_pitch_rate_q = _pitch_to_pitch_rate_pid.update(0.0, -pitch_error, dt);

    // Pass through desired yaw rate from outer loop (or could be modified here)
    rate_cmds.desired_yaw_rate_r = attitude_cmds.desired_yaw_rate_r;

    return rate_cmds;
}

void MiddleLoopController::reset() {
    _roll_to_roll_rate_pid.reset();
    _pitch_to_pitch_rate_pid.reset();
}

InnerLoopController::InnerLoopController() :
    _roll_rate_to_aileron_pid(Config::K_p_p, 0.0, Config::K_d_p, // Ki for p is 0
                              -Config::MAX_AILERON_DEFLECTION, Config::MAX_AILERON_DEFLECTION,
                              0.0, 0.0),
    _pitch_rate_to_elevator_pid(Config::K_p_q, 0.0, Config::K_d_q, // Ki for q is 0
                                -Config::MAX_ELEVATOR_DEFLECTION, Config::MAX_ELEVATOR_DEFLECTION,
                                0.0, 0.0),
    _yaw_rate_to_rudder_pid(Config::K_p_r, 0.0, Config::K_d_r,   // Ki for r is 0
                            -Config::MAX_RUDDER_DEFLECTION, Config::MAX_RUDDER_DEFLECTION,
                            0.0, 0.0)
{}

ControlInputs InnerLoopController::update(const AttitudeRateCommands& rate_cmds, const UAVState& current_state, double dt) {
    ControlInputs actuator_cmds;

    // Roll rate to aileron (p_d -> delta_a)
    double p_error = rate_cmds.desired_roll_rate_p - current_state.angular_rates_body.x();
    actuator_cmds.delta_a = _roll_rate_to_aileron_pid.update(0.0, -p_error, dt);

    // Pitch rate to elevator (q_d -> delta_e)
    double q_error = rate_cmds.desired_pitch_rate_q - current_state.angular_rates_body.y();
    actuator_cmds.delta_e = _pitch_rate_to_elevator_pid.update(0.0, -q_error, dt);

    // Yaw rate to rudder (r_d -> delta_r)
    double r_error = rate_cmds.desired_yaw_rate_r - current_state.angular_rates_body.z();
    actuator_cmds.delta_r = _yaw_rate_to_rudder_pid.update(0.0, -r_error, dt);
    
    // Thrust is passed through from outer loop commands typically
    // (Handled by CascadedController)

    return actuator_cmds;
}

void InnerLoopController::reset() {
    _roll_rate_to_aileron_pid.reset();
    _pitch_rate_to_elevator_pid.reset();
    _yaw_rate_to_rudder_pid.reset();
}


CascadedController::CascadedController() {
    // Initialize last commands to safe/neutral values
    _last_outer_cmds.desired_roll_phi = 0.0;
    _last_outer_cmds.desired_pitch_theta = 0.0;
    _last_outer_cmds.desired_yaw_rate_r = 0.0;
    _last_outer_cmds.thrust_command = 0.0; // Assuming 0 thrust is idle/off

    _last_attitude_rate_cmds.desired_roll_rate_p = 0.0;
    _last_attitude_rate_cmds.desired_pitch_rate_q = 0.0;
    _last_attitude_rate_cmds.desired_yaw_rate_r = 0.0;

    _last_control_inputs.thrust = 0.0;
    _last_control_inputs.delta_a = 0.0;
    _last_control_inputs.delta_e = 0.0;
    _last_control_inputs.delta_r = 0.0;
}

void CascadedController::reset() {
    _outer_loop.reset();
    _middle_loop.reset();
    _inner_loop.reset();
    // Re-initialize last commands
     _last_outer_cmds.desired_roll_phi = 0.0;
    _last_outer_cmds.desired_pitch_theta = 0.0;
    _last_outer_cmds.desired_yaw_rate_r = 0.0;
    _last_outer_cmds.thrust_command = 0.0;
    _last_attitude_rate_cmds.desired_roll_rate_p = 0.0;
    _last_attitude_rate_cmds.desired_pitch_rate_q = 0.0;
    _last_attitude_rate_cmds.desired_yaw_rate_r = 0.0;
    _last_control_inputs.thrust = 0.0;
    _last_control_inputs.delta_a = 0.0;
    _last_control_inputs.delta_e = 0.0;
    _last_control_inputs.delta_r = 0.0;
}

ControlInputs CascadedController::update(const FlightPathCommand& targets, const UAVState& current_state,
                                       bool run_outer_loop, bool run_middle_loop, bool run_inner_loop, double dt_sim) {
    if (run_outer_loop) {
        _last_outer_cmds = _outer_loop.update(targets, current_state, Config::OUTER_LOOP_DT);
    }

    if (run_middle_loop) {
        _last_attitude_rate_cmds = _middle_loop.update(_last_outer_cmds, current_state, Config::MIDDLE_LOOP_DT);
    }
    // Pass through yaw rate if not updated by middle loop, or if middle loop has its own yaw logic
    _last_attitude_rate_cmds.desired_yaw_rate_r = _last_outer_cmds.desired_yaw_rate_r;


    if (run_inner_loop) {
        _last_control_inputs = _inner_loop.update(_last_attitude_rate_cmds, current_state, Config::INNER_LOOP_DT);
        // Thrust command comes from the outer loop
        _last_control_inputs.thrust = _last_outer_cmds.thrust_command;
    }
    
    return _last_control_inputs;
}