#include <filesystem>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>

#include "rules_cc/cc/runfiles/runfiles.h"

#include "odrive-api/interface/odrive_driver.h"
#include "odrive-api/interface/odrive_socket_driver.h"
#include "odrive-api/communication/odrive_socket.h"

using rules_cc::cc::runfiles::Runfiles;


const std::string CAN_IFC = "can0";

int main(int argc, char** argv) {
    std::string error;
    std::unique_ptr<Runfiles> runfiles(
        Runfiles::Create(argv[0], BAZEL_CURRENT_REPOSITORY, &error)
    );

    // ODrive Socket:
    const std::vector<canid_t> motor_ids = {0, 1};
    auto odrive_socket = std::make_shared<ODriveSocketDriver>(
        CAN_IFC, motor_ids
    );

    // ODrive Driver:
    auto odrive_driver = ODriveDriver(
        std::vector<std::shared_ptr<ODriveSocketDriver>>{odrive_socket}
    );

    // Set control mode and Axis State:
    odrive_driver.set_axis_state(ODriveAxisState::CLOSED_LOOP_CONTROL);
    odrive_driver.set_control_mode(
        ODriveControlMode::POSITION
    );

    // Wait for ODrive
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Get Current Positions and set as default Motor Command Positions:
    std::vector<MotorState> motor_states = odrive_driver.get_motor_states();

    // Print Motor Positions:
    for(const auto& motor_state : motor_states) {
        std::cout << motor_state.position[0] << " " << motor_state.position[1] << std::endl;
    }

    std::vector<MotorCommand> motor_commands(motor_states.size());
    for(size_t i = 0; i < motor_states.size(); ++i) {
        std::copy(
            motor_states[i].position.begin(),
            motor_states[i].position.end(),
            motor_commands[i].position_setpoint.begin()
        );
        motor_commands[i].stiffness = {50.0f, 50.0f};
        motor_commands[i].damping = {0.15f, 0.15f};
    }

    // Send Motor Commands:
    odrive_driver.update_command(motor_commands);

    // Initialize Control Loop:
    odrive_driver.initialize_thread();
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // Stop Control Loop:
    odrive_driver.stop_thread();
    
    // Set Axis State to IDLE:
    odrive_driver.set_axis_state(ODriveAxisState::IDLE);

    return 0;
}
