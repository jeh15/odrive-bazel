#include <chrono>
#include <string>
#include <filesystem>
#include <iostream>
#include <format>

#include "utils/odrive_socket.h"
#include "utils/motor_controller.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h

const std::string CAN_IFC = "can0";
const int MOTOR_ID = 0;
const uint32_t CTRL_MODE = 1;


int main(void) {
    // Setup Logger:
    std::filesystem::path directory = std::filesystem::current_path();
    std::string filename = "logs/basic-log.txt";
    std::string log_path = std::format("{}{}", directory, filename);

    try {
        auto logger = spdlog::basic_logger_mt("basic_logger", "logs/basic-log.txt");
    }
    catch (const spdlog::spdlog_ex &ex) {
        std::cout << "Log init failed: " << ex.what() << std::endl;
    }

    auto odrv = std::make_shared<ODriveSocket>(CAN_IFC);

    // Create Shared ODrive Socket and Motor Controller:
    MotorController motor_controller(
        odrv,
        MOTOR_ID
    );

    // Set Control Mode and AxisState:
    motor_controller.set_axis_state(ODriveAxisState::CLOSED_LOOP_CONTROL);
    motor_controller.set_control_mode(ODriveControlMode::TORQUE);

    while(true){
        float torque_setpoint = 0.1f;

        // Send Torque Command:
        motor_controller.set_torque(torque_setpoint);

        // Get Position, Velocity, and Torque Estimate Data:
        float position = motor_controller.get_position();
        float velocity = motor_controller.get_velocity();
        float torque = motor_controller.get_torque_estimate();

        // Log Data:
        std::string log_data = std::format("Position: {}, Velocity: {}, Torque: {} \n", position, velocity, torque);
        logger->info(log_data);

        // Sleep for 20ms:
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
