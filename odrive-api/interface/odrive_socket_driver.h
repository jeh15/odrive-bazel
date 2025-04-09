#pragma once

#include <linux/can.h>
#include <cstdint>
#include <memory>
#include <vector>
#include <string>

#include "odrive-api/communication/odrive_socket.h"

#include "odrive-api/containers.h"


using namespace odrive::containers;


class ODriveSocketDriver {
    public:
        ODriveSocketDriver(std::shared_ptr<ODriveSocket> odrv_socket, std::vector<canid_t> motor_ids)
            : odrv_socket(odrv_socket), motor_ids(motor_ids) { }
        ~ODriveSocketDriver() { 
            for(const canid_t motor_id : motor_ids)
                odrv_socket->setAxisState(motor_id, ODriveAxisState::IDLE);
        }

        // Public methods:
        void set_axis_state(const ODriveAxisState axis_state) {
            for(const canid_t motor_id : motor_ids)
                odrv_socket->setAxisState(motor_id, axis_state);
        }

        void set_control_mode(const ODriveControlMode control_mode, const ODriveInputMode input_mode = ODriveInputMode::PASSTHROUGH) {
            ctrl_mode = control_mode;
            for(const canid_t motor_id : motor_ids)
                odrv_socket->setControlMode(motor_id, control_mode, input_mode);
        }

        std::vector<std::string> get_axis_state(void) {
            std::vector<std::string> axis_state;
            for(const canid_t motor_id : motor_ids) {
                uint8_t state = odrv_socket->getAxisState(motor_id);
                switch (state) {
                    case ODriveAxisState::UNDEFINED:
                        axis_state.push_back("UNDEFINED");
                        break;
                    case ODriveAxisState::IDLE:
                        axis_state.push_back("IDLE");
                        break;
                    case ODriveAxisState::CLOSED_LOOP_CONTROL:
                        axis_state.push_back("CLOSED_LOOP_CONTROL");
                        break;
                    default:
                        axis_state.push_back("UNKNOWN");
                        break;
                }
            }
            return axis_state;
        }

        void update_command(MotorCommand& command) {
            for(const canid_t motor_id : motor_ids) {
                motor_commands.position_setpoint[motor_id] = command.position_setpoint[motor_id];
                motor_commands.velocity_setpoint[motor_id] = command.velocity_setpoint[motor_id];
                motor_commands.torque_feedforward[motor_id] = command.torque_feedforward[motor_id];
                motor_commands.damping[motor_id] = command.damping[motor_id];
                motor_commands.velocity_integrator[motor_id] = command.velocity_integrator[motor_id];
                motor_commands.stiffness[motor_id] = command.stiffness[motor_id];
            }
        }

        MotorState get_motor_states() {
            MotorState motor_states = { 0 };
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
            std::shared_ptr<ODriveSocket> odrv_socket;
            std::vector<canid_t> motor_ids;
            MotorCommand motor_commands = { 0 };
            ODriveControlMode ctrl_mode;
}