#pragma once

#include <linux/can.h>
#include <thread>
#include <cstdint>
#include <mutex>
#include <atomic>
#include <memory>
#include <vector>

#include "src/communication/odrive_socket.h"
#include "src/utils/estop.h"

#include "src/controller/low_level_types.h"


class MotorController : public Estop {
    public:
        MotorController(std::shared_ptr<ODriveSocket> odrv, std::vector<canid_t> motor_ids)
            : Estop(), odrv_socket(odrv), motor_ids(motor_ids) { }

        ~MotorController() { 
            for(const canid_t motor_id : motor_ids)
                odrv_socket->setAxisState(motor_id, ODriveAxisState::IDLE);
        }

        // Public members:
        ODriveControlMode control_mode = ODriveControlMode::VELOCITY;

        // Public methods:
        void set_axis_state(const ODriveAxisState axis_state) {
            for(const canid_t motor_id : motor_ids)
                odrv_socket->setAxisState(motor_id, axis_state);
        }

        void set_control_mode(const ODriveControlMode control_mode, const ODriveInputMode input_mode = ODriveInputMode::PASSTHROUGH) {
            for(const canid_t motor_id : motor_ids)
                odrv_socket->setControlMode(motor_id, control_mode, input_mode);
        }

        std::vector<ODriveAxisState> get_axis_state() {
            std::vector<ODriveAxisState> axis_state;
            for(const canid_t motor_id : motor_ids) {
                uint8_t state = odrv_socket->getAxisState(motor_id);
                switch (state) {
                    case ODriveAxisState::UNDEFINED:
                        axis_state.push_back(ODriveAxisState::UNDEFINED);
                        break;
                    case ODriveAxisState::IDLE:
                        axis_state.push_back(ODriveAxisState::IDLE);
                        break;
                    case ODriveAxisState::CLOSED_LOOP_CONTROL:
                        axis_state.push_back(ODriveAxisState::CLOSED_LOOP_CONTROL);
                        break;
                    default:
                        axis_state.push_back(ODriveAxisState::UNDEFINED);
                        break;
                }
            }
            return axis_state;
        }

        void initialize_control_thread() {
            thread = std::thread(&MotorController::control_loop, this);
        }

        void stop_control_thread() {
            running = false;
            thread.join();
        }

        void update_command(lowleveltypes::MotorCommand& command) {
            std::lock_guard<std::mutex> lock(mutex);
            for(const canid_t motor_id : motor_ids) {
                motor_commands.position_setpoint[motor_id] = command.position_setpoint[motor_id];
                motor_commands.velocity_setpoint[motor_id] = command.velocity_setpoint[motor_id];
                motor_commands.torque_feedforward[motor_id] = command.torque_feedforward[motor_id];
                motor_commands.damping[motor_id] = command.damping[motor_id];
                motor_commands.stiffness[motor_id] = command.stiffness[motor_id];
                motor_commands.kp[motor_id] = command.kp[motor_id];
                motor_commands.kd[motor_id] = command.kd[motor_id];
            }
        }

        lowleveltypes::MotorState get_motor_states() {
            lowleveltypes::MotorState motor_states = { 0 };
            for(const canid_t motor_id : motor_ids) {
                motor_states.position[motor_id] = odrv_socket->getPositionEstimate(motor_id);
                motor_states.velocity[motor_id] = odrv_socket->getVelocityEstimate(motor_id);
                motor_states.torque_estimate[motor_id] = odrv_socket->getTorqueEstimate(motor_id);
                motor_states.current_setpoint[motor_id] = odrv_socket->getIqSetpoint(motor_id);
                motor_states.current_measured[motor_id] = odrv_socket->getIqMeasured(motor_id);
            }
            return motor_states;
        }

    private:
        // ODrive Variables:
        std::shared_ptr<ODriveSocket> odrv_socket;
        std::vector<canid_t> motor_ids;
        float torque_constant_knee = 8.27F / 330.0F;
        float torque_constant_hip = 8.27f / 150.0f;
        // Motor Command Struct:
        lowleveltypes::MotorCommand motor_commands = { 0 };
        // Control Loop Thread Variables:
        uint8_t control_rate_ms = 2;
        std::atomic<bool> running{true};
        std::mutex mutex;
        std::thread thread;

        void estop(int sig) override {
            printf("Running ESTOP\n");
            for(const canid_t motor_id : motor_ids)
                odrv_socket->setAxisState(motor_id, ODriveAxisState::IDLE);
        }

        void control_loop() {
            while(running) {
                /* Lock Guard Scope */
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    for(const canid_t motor_id : motor_ids) {
                        float q_error = motor_commands.position_setpoint[motor_id] - odrv_socket->getPositionEstimate(motor_id);
                        float qd_error = motor_commands.velocity_setpoint[motor_id] - odrv_socket->getVelocityEstimate(motor_id);
                        float torque_input = motor_commands.torque_feedforward[motor_id] + motor_commands.kp[motor_id] * q_error + motor_commands.kd[motor_id] * qd_error;
                        switch(control_mode) {
                            case ODriveControlMode::POSITION:
                                odrv_socket->set_stiffness(motor_id, motor_commands.stiffness[motor_id]);
                                odrv_socket->set_damping(motor_id, motor_commands.damping[motor_id]);
                                odrv_socket->position_command(
                                    motor_id,
                                    motor_commands.position_setpoint[motor_id],
                                    motor_commands.velocity_setpoint[motor_id],
                                    torque_input
                                );
                                break;
                            case ODriveControlMode::VELOCITY:
                                odrv_socket->set_damping(motor_id, motor_commands.damping[motor_id]);
                                odrv_socket->velocity_command(
                                    motor_id,
                                    motor_commands.velocity_setpoint[motor_id],
                                    torque_input
                                );
                                break;
                            case ODriveControlMode::TORQUE:
                                odrv_socket->torque_command(motor_id, torque_input);
                                break;
                            case ODriveControlMode::VOLTAGE:
                                break;
                        }
                    }
                }
                // Control Rate:
                std::this_thread::sleep_for(std::chrono::milliseconds(control_rate_ms));
            }
        }

};