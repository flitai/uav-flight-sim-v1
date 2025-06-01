#ifndef DYNAMICS_MODEL_HPP
#define DYNAMICS_MODEL_HPP

#include "CommonTypes.hpp"
#include "Config.hpp" // For physical parameters and aero coeffs

class IDynamicsModel {
public:
    virtual ~IDynamicsModel() = default;
    virtual void initializeState(const UAVState& initial_state) = 0;
    virtual void updateState(const ControlInputs& controls, double dt) = 0;
    virtual const UAVState& getCurrentState() const = 0;
};

class SixDOFModel : public IDynamicsModel {
public:
    SixDOFModel();
    void initializeState(const UAVState& initial_state) override;
    void updateState(const ControlInputs& controls, double dt) override;
    const UAVState& getCurrentState() const override;

private:
    UAVState _current_state;

    // Function to compute state derivatives: dx/dt = f(x, u)
    // x_full = [pos_e, vel_b, euler, ang_rates_b]^T
    Eigen::Matrix<double, 12, 1> stateDerivative(const UAVState& state, const ControlInputs& controls);

    // Helper for aerodynamic forces and moments
    void calculateAerodynamics(const UAVState& state, const ControlInputs& controls,
                               Eigen::Vector3d& forces_body, Eigen::Vector3d& moments_body);

    // Helper for gravity force in body frame
    Eigen::Vector3d gravityForceBody(const Eigen::Vector3d& euler_angles);

    // Helper for rotation matrix from body to Earth frame
    Eigen::Matrix3d getRotationMatrixBodyToEarth(const Eigen::Vector3d& euler_angles);
    // Helper for Euler angle rates transformation matrix
    Eigen::Matrix3d getEulerRatesMatrix(const Eigen::Vector3d& euler_angles);
};

#endif // DYNAMICS_MODEL_HPP