#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd, double output_min, double output_max, double integral_min, double integral_max);

    double update(double desired, double actual, double dt);
    void reset();
    void setGains(double Kp, double Ki, double Kd);

private:
    double _Kp, _Ki, _Kd;
    double _output_min, _output_max;
    double _integral_min, _integral_max;
    double _integral;
    double _previous_error;
    bool _first_run;
};

#endif // PID_CONTROLLER_HPP