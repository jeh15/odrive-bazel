#include <chrono>
#include <memory>
#include <string>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <ranges>
#include <array>

#include "src/communication/odrive_socket.h"
#include "src/controller/low_level_controller.h"
#include "src/controller/low_level_types.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

const std::string CAN_IFC = "can0";
const uint32_t CTRL_MODE = 1;


int main(void) {
    // Setup Logger:
    std::filesystem::path filepath = "/home/pi/repository/bazel-odrive/logs";
    std::filesystem::path filename = "data_log.txt";
    filepath /= filename;
    auto logger = spdlog::basic_logger_mt("odrive_data_logger", filepath);

    // Create Shared ODrive Socket
    auto odrv = std::make_shared<ODriveSocket>(CAN_IFC);

    // Create Shared ODrive Socket and Motor Controller:
    const std::vector<canid_t> motor_ids = { 0 };
    MotorController motor_controller(
        odrv,
        motor_ids
    );

    // Set Control Mode, AxisState, and initialize control thread:
    motor_controller.set_axis_state(ODriveAxisState::CLOSED_LOOP_CONTROL);
    motor_controller.set_control_mode(ODriveControlMode::TORQUE);
    motor_controller.initialized_control_thread();

    // Initialize Command Struct:
    lowleveltypes::MotorCommand motor_command = { 0 };

    const int num_iterations = 100; 
    for(const auto& i : std::views::iota(1, num_iterations)) {
        std::ignore = i;
        // Set and Send Motor Command:
        /* Motor Command Struct Goes Here */

        // Get Position, Velocity, and Torque Estimate Data:
        lowleveltypes::MotorState motor_states = motor_controller.get_motor_states();
        std::array<float, lowleveltypes::num_motors> position = motor_states.position;
        std::array<float, lowleveltypes::num_motors> velocity = motor_states.velocity;
        std::array<float, lowleveltypes::num_motors> torque = motor_states.torque_estimate;

        // Log Data:
        /* Need to loop this operation to work with an array */
        // logger->info(
        //     "Postion: {0}, Velocity: {1}, Torque: {2}",
        //     position, velocity, torque
        // );

        // Sleep for 20ms:
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return 0;
}
