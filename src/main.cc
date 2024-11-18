#include <chrono>
#include <memory>
#include <string>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <ranges>

#include "utils/odrive_socket.h"
#include "utils/motor_controller.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

const std::string CAN_IFC = "can0";
const int MOTOR_ID = 0;
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
    MotorController motor_controller(
        odrv,
        MOTOR_ID
    );

    // Set Control Mode and AxisState:
    motor_controller.set_axis_state(ODriveAxisState::CLOSED_LOOP_CONTROL);
    motor_controller.set_control_mode(ODriveControlMode::TORQUE);

    const int num_iterations = 100; 
    for(const int i : std::views::iota(1, num_iterations)) {
        float torque_setpoint = 0.1f;

        // Send Torque Command:
        motor_controller.set_torque(torque_setpoint);

        // Get Position, Velocity, and Torque Estimate Data:
        float position = motor_controller.get_position();
        float velocity = motor_controller.get_velocity();
        float torque = motor_controller.get_torque_estimate();

        // Log Data:
        logger->info(
            "Postion: {0}, Velocity: {1}, Torque: {2}",
            position, velocity, torque
        );

        // Sleep for 20ms:
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return 0;
}
