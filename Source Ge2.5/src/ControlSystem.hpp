#ifndef CONTROL_SYSTEM_HPP
#define CONTROL_SYSTEM_HPP

#include "CommonTypes.hpp"
#include "PIDController.hpp"
#include "Config.hpp" // For PID gains

class OuterLoopController {
public:
    OuterLoopController();
    OuterLoopCommands update(const FlightPathCommand& targets, const UAVState& current_state, double dt);
    void reset();
private:
    PIDController _heading_to_roll_pid;  // psi_err -> phi_cmd
    PIDController _altitude_to_pitch_pid; // h_err -> theta_cmd
    PIDController _airspeed_to_thrust_pid; // V_err -> thrust_cmd
    // Optional: Yaw error to yaw rate command for turn coordination / direct yaw control
    PIDController _yaw_err_to_yaw_rate_pid; // psi_err -> r_cmd (as per doc "偏航跟踪")
};

class MiddleLoopController {
public:
    MiddleLoopController();
    AttitudeRateCommands update(const OuterLoopCommands& attitude_cmds, const UAVState& current_state, double dt);
    void reset();
private:
    PIDController _roll_to_roll_rate_pid;  // phi_err -> p_cmd
    PIDController _pitch_to_pitch_rate_pid; // theta_err -> q_cmd
};

class InnerLoopController {
public:
    InnerLoopController();
    ControlInputs update(const AttitudeRateCommands& rate_cmds, const UAVState& current_state, double dt);
    void reset();
private:
    PIDController _roll_rate_to_aileron_pid; // p_err -> delta_a
    PIDController _pitch_rate_to_elevator_pid; // q_err -> delta_e
    PIDController _yaw_rate_to_rudder_pid;   // r_err -> delta_r
};


class CascadedController {
public:
    CascadedController();
    void reset();
    ControlInputs update(const FlightPathCommand& targets, const UAVState& current_state,
                         bool run_outer_loop, bool run_middle_loop, bool run_inner_loop, double dt_sim);

    // To get intermediate commands if needed for logging/debugging
    OuterLoopCommands getLastOuterLoopCommands() const { return _last_outer_cmds; }
    AttitudeRateCommands getLastMiddleLoopCommands() const { return _last_attitude_rate_cmds; }

private:
    OuterLoopController _outer_loop;
    MiddleLoopController _middle_loop;
    InnerLoopController _inner_loop;

    // Store last commands from higher loops to feed to lower loops
    OuterLoopCommands _last_outer_cmds;
    AttitudeRateCommands _last_attitude_rate_cmds;
    ControlInputs _last_control_inputs; // Last computed control inputs
};


#endif // CONTROL_SYSTEM_HPP