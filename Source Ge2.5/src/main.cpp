#include <iostream>
#include <fstream>
#include <iomanip> // For std::fixed, std::setprecision
#include "CommonTypes.hpp"
#include "Config.hpp"
#include "DynamicsModel.hpp"
#include "ControlSystem.hpp"
#include "SimulationEngine.hpp"
#include <memory> // For std::make_unique

\\ This is a basic example of how to use the simulation components.



int main() {
    // 1. Create simulation components
    auto model = std::make_unique<SixDOFModel>();
    auto controller = std::make_unique<CascadedController>();
    SimulationEngine engine(std::move(model), std::move(controller));

    // 2. Define initial state and targets
    UAVState initial_uav_state;
    initial_uav_state.position_earth = Eigen::Vector3d(0, 0, -100); // 100m altitude (NED)
    initial_uav_state.velocity_body = Eigen::Vector3d(20, 0, 0);   // 20 m/s forward
    initial_uav_state.euler_angles = Eigen::Vector3d(0, 0, 0);     // Level flight, north heading
    initial_uav_state.angular_rates_body.setZero();

    FlightPathCommand targets;
    targets.desired_altitude = 100.0; // m
    targets.desired_airspeed = 22.0;  // m/s
    targets.desired_heading = 0.0 * M_PI / 180.0; // Radians (North)

    engine.initialize(initial_uav_state, targets);

    // 3. Simulation loop
    double total_simulation_time = 60.0; // seconds
    std::ofstream log_file("simulation_log.csv");
    log_file << "Time,X,Y,Z,u,v,w,phi,theta,psi,p,q,r,thrust,delta_a,delta_e,delta_r,phi_cmd,theta_cmd,p_cmd,q_cmd,r_cmd\n";
    log_file << std::fixed << std::setprecision(4);


    std::cout << "Starting simulation..." << std::endl;
    std::cout << std::fixed << std::setprecision(3);

    while (engine.getCurrentTime() < total_simulation_time) {
        engine.runStep();
        const UAVState& current_state = engine.getCurrentUAVState();

        // Log data (example)
        if (static_cast<int>(engine.getCurrentTime() * 1000) % 100 == 0) { // Log every 0.1s
            log_file << engine.getCurrentTime() << ","
                     << current_state.position_earth.x() << "," << current_state.position_earth.y() << "," << current_state.position_earth.z() << ","
                     << current_state.velocity_body.x() << "," << current_state.velocity_body.y() << "," << current_state.velocity_body.z() << ","
                     << current_state.euler_angles.x() * 180.0 / M_PI << "," << current_state.euler_angles.y() * 180.0 / M_PI << "," << current_state.euler_angles.z() * 180.0 / M_PI << ","
                     << current_state.angular_rates_body.x() * 180.0 / M_PI << "," << current_state.angular_rates_body.y() * 180.0 / M_PI << "," << current_state.angular_rates_body.z() * 180.0 / M_PI << ",";

            ControlInputs actual_inputs = controller->update(targets, current_state, false,false,false,0); // get last inputs
            OuterLoopCommands ol_cmds = controller->getLastOuterLoopCommands();
            AttitudeRateCommands mid_cmds = controller->getLastMiddleLoopCommands();

            log_file << actual_inputs.thrust << "," << actual_inputs.delta_a * 180.0/M_PI << "," << actual_inputs.delta_e* 180.0/M_PI << "," << actual_inputs.delta_r* 180.0/M_PI << ","
                     << ol_cmds.desired_roll_phi * 180.0/M_PI << "," << ol_cmds.desired_pitch_theta * 180.0/M_PI << ","
                     << mid_cmds.desired_roll_rate_p * 180.0/M_PI << "," << mid_cmds.desired_pitch_rate_q * 180.0/M_PI << "," << mid_cmds.desired_yaw_rate_r * 180.0/M_PI
                     << "\n";

             // Console output for quick check
             std::cout << "T: " << engine.getCurrentTime()
                       << " Alt: " << -current_state.position_earth.z()
                       << " V: " << current_state.velocity_body.x()
                       << " Psi: " << current_state.euler_angles.z() * 180.0 / M_PI
                       << " Phi: " << current_state.euler_angles.x() * 180.0 / M_PI
                       << " Theta: " << current_state.euler_angles.y() * 180.0 / M_PI
                       << std::endl;
        }

        // Example: Change target heading after some time
        if (engine.getCurrentTime() > 30.0 && targets.desired_heading == 0.0) {
            std::cout << "Changing target heading to 45 degrees." << std::endl;
            targets.desired_heading = 45.0 * M_PI / 180.0;
        }

    }

    log_file.close();
    std::cout << "Simulation finished. Log saved to simulation_log.csv" << std::endl;

    return 0;
}