#pragma once

#include <cstdint>
#include <array>


namespace odrive::containers {
    constexpr uint8_t num_motors = 2;

    struct MotorCommand {
        std::array<float, num_motors> position_setpoint = { 0 };
        std::array<float, num_motors> velocity_setpoint = { 0 };
        std::array<float, num_motors> torque_feedforward = { 0 };
        std::array<float, num_motors> damping = { 0 };
        std::array<float, num_motors> velocity_integrator = { 0 };
        std::array<float, num_motors> stiffness = { 0 };
    };

    struct MotorState {
        std::array<float, num_motors> position = { 0 };
        std::array<float, num_motors> velocity = { 0 };
        std::array<float, num_motors> torque_estimate = { 0 };
        std::array<float, num_motors> current_setpoint = { 0 };
        std::array<float, num_motors> current_measured = { 0 };
    };

    struct FullMotorState {
        std::array<float, num_motors> position = { 0 };
        std::array<float, num_motors> velocity = { 0 };
        std::array<float, num_motors> torque_estimate = { 0 };
        std::array<float, num_motors> current_setpoint = { 0 };
        std::array<float, num_motors> current_measured = { 0 };
        std::array<float, num_motors> fet_temperature = { 0 };
    };

}
