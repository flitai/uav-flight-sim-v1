#ifndef SIMULATION_ENGINE_HPP
#define SIMULATION_ENGINE_HPP

#include "CommonTypes.hpp"
#include "DynamicsModel.hpp"
#include "ControlSystem.hpp"
#include <memory> // For std::unique_ptr

class SimulationEngine {
public:
    SimulationEngine(std::unique_ptr<IDynamicsModel> model,
                     std::unique_ptr<CascadedController> controller);

    void initialize(const UAVState& initial_state, const FlightPathCommand& initial_targets);
    void runStep();
    const UAVState& getCurrentUAVState() const;
    double getCurrentTime() const;

private:
    std::unique_ptr<IDynamicsModel> _model;
    std::unique_ptr<CascadedController> _controller;
    FlightPathCommand _current_targets;
    double _simulation_time;
    long long _step_count; // To manage different loop frequencies

    // Simulation step size, same as inner loop and dynamics
    const double _dt_sim = Config::SIMULATION_STEP_SIZE;
};

#endif // SIMULATION_ENGINE_HPP