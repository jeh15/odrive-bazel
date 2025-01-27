#pragma once

#include <cstdint>
#include <array>


namespace lowleveltypes {
    constexpr uint8_t num_motors = 2;

    struct MotorCommand {
        std::array<float, num_motors> position_setpoint = { 0 };
        std::array<float, num_motors> velocity_setpoint = { 0 };
        std::array<float, num_motors> torque_feedforward = { 0 };
        std::array<float, num_motors> damping = { 0 };
        std::array<float, num_motors> stiffness = { 0 };
        std::array<float, num_motors> kp = { 0 };
        std::array<float, num_motors> kd = { 0 };
    };

    struct MotorState {
        std::array<float, num_motors> position = { 0 };
        std::array<float, num_motors> velocity = { 0 };
        std::array<float, num_motors> torque_estimate = { 0 };
        std::array<float, num_motors> current_setpoint = { 0 };
        std::array<float, num_motors> current_measured = { 0 };
    };
}
