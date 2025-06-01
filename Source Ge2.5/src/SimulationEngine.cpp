#include "SimulationEngine.hpp"
#include "Config.hpp" // For loop delta times

SimulationEngine::SimulationEngine(std::unique_ptr<IDynamicsModel> model,
                                   std::unique_ptr<CascadedController> controller)
    : _model(std::move(model)),
      _controller(std::move(controller)),
      _simulation_time(0.0),
      _step_count(0) {}

void SimulationEngine::initialize(const UAVState& initial_state, const FlightPathCommand& initial_targets) {
    _model->initializeState(initial_state);
    _current_targets = initial_targets;
    _simulation_time = 0.0;
    _step_count = 0;
    _controller->reset();
}

void SimulationEngine::runStep() {
    const UAVState& current_uav_state = _model->getCurrentState();

    // Determine which loops to run based on timing
    // Using simple modulo for step counts. For precise timing, use simulation_time.
    // Assuming _dt_sim is the smallest time unit (e.g., 0.01s)
    // Outer loop: 20 Hz -> 0.05s -> every 5 steps if _dt_sim = 0.01s
    // Middle loop: 50 Hz -> 0.02s -> every 2 steps if _dt_sim = 0.01s
    // Inner loop: 100 Hz -> 0.01s -> every 1 step if _dt_sim = 0.01s

    bool run_outer = (_step_count % static_cast<long long>(Config::OUTER_LOOP_DT / _dt_sim + 0.5)) == 0;
    bool run_middle = (_step_count % static_cast<long long>(Config::MIDDLE_LOOP_DT / _dt_sim + 0.5)) == 0;
    bool run_inner = (_step_count % static_cast<long long>(Config::INNER_LOOP_DT / _dt_sim + 0.5)) == 0;
    // Ensure inner loop always runs if its DT matches sim DT
    if (Config::INNER_LOOP_DT == _dt_sim) run_inner = true;


    ControlInputs actuator_commands = _controller->update(
        _current_targets,
        current_uav_state,
        run_outer,
        run_middle,
        run_inner,
        _dt_sim // dt for the innermost part of controller if logic needs it directly
    );

    _model->updateState(actuator_commands, _dt_sim);

    _simulation_time += _dt_sim;
    _step_count++;
}

const UAVState& SimulationEngine::getCurrentUAVState() const {
    return _model->getCurrentState();
}

double SimulationEngine::getCurrentTime() const {
    return _simulation_time;
}