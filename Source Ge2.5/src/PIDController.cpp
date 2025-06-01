#include "PIDController.hpp"
#include <algorithm> // For std::clamp

PIDController::PIDController(double Kp, double Ki, double Kd,
                             double output_min, double output_max,
                             double integral_min, double integral_max)
    : _Kp(Kp), _Ki(Ki), _Kd(Kd),
      _output_min(output_min), _output_max(output_max),
      _integral_min(integral_min), _integral_max(integral_max),
      _integral(0.0), _previous_error(0.0), _first_run(true) {}

double PIDController::update(double desired, double actual, double dt) {
    if (dt <= 0.0) return _output_min; // Avoid division by zero or weird behavior

    double error = desired - actual;

    // Proportional term
    double P_out = _Kp * error;

    // Integral term
    _integral += error * dt;
    _integral = std::clamp(_integral, _integral_min, _integral_max); // Anti-windup
    double I_out = _Ki * _integral;

    // Derivative term
    double derivative = 0.0;
    if (!_first_run && dt > 0.0) {
        derivative = (error - _previous_error) / dt;
    }
    double D_out = _Kd * derivative;

    // Total output
    double output = P_out + I_out + D_out;

    // Update state for next iteration
    _previous_error = error;
    _first_run = false;

    return std::clamp(output, _output_min, _output_max);
}

void PIDController::reset() {
    _integral = 0.0;
    _previous_error = 0.0;
    _first_run = true;
}

void PIDController::setGains(double Kp, double Ki, double Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}