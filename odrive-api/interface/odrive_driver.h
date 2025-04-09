#pragma once

#include <linux/can.h>
#include <thread>
#include <cstdint>
#include <mutex>
#include <atomic>
#include <memory>
#include <vector>
#include <string>

#include "odrive-api/interface/odrive_socket_driver.h"
#include "odrive-api/communication/odrive_socket.h"
#include "odrive-api/utils/estop.h"

#include "odrive-api/containers.h"


using namespace odrive::containers;


class ODriveDriver : public Estop {
    public:
        ODriveDriver(std::vector<std::shared_ptr<ODriveSocketDriver>> odrvs, int control_rate_us = 2000)
            : Estop(), odrvs(odrvs), motor_ids(motor_ids), control_rate_us(control_rate_us) { }

        ~ODriveDriver() { 
            for(const canid_t motor_id : motor_ids)
                odrv_socket->setAxisState(motor_id, ODriveAxisState::IDLE);
        }

        // Public methods:
        void set_axis_state(const ODriveAxisState axis_state) {
            for(const std::shared_ptr<ODriveSocketDriver> odrv : odrvs)
                odrv->set_axis_state(axis_state);
        }

        void set_control_mode(const ODriveControlMode control_mode, const ODriveInputMode input_mode = ODriveInputMode::PASSTHROUGH) {
            ctrl_mode = control_mode;
            for(const std::shared_ptr<ODriveSocketDriver> odrv : odrvs)
                odrv->set_control_mode(control_mode, input_mode);
        }

        std::vector<std::string> get_axis_state(void) {
            std::vector<std::string> axis_states;
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

        void initialize_thread() {
            thread = std::thread(&ODriveDriver::control_loop, this);
        }

        void stop_thread() {
            running = false;
            thread.join();
        }

        void update_command(MotorCommand& command) {
            std::lock_guard<std::mutex> lock(mutex);
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
        // ODrive Variables:
        std::shared_ptr<ODriveSocket> odrv_socket;
        std::vector<canid_t> motor_ids;
        float torque_constant_knee = 8.27F / 330.0F;
        float torque_constant_hip = 8.27f / 150.0f;
        ODriveControlMode ctrl_mode;
        // Motor Command Struct:
        MotorCommand motor_commands = { 0 };
        // Control Loop Thread Variables:
        int control_rate_us;
        std::chrono::microseconds control_rate = std::chrono::microseconds(control_rate_us);
        std::atomic<bool> running{true};
        std::mutex mutex;
        std::thread thread;

        void estop(int sig) override {
            printf("Running ESTOP\n");
            for(const canid_t motor_id : motor_ids)
                odrv_socket->setAxisState(motor_id, ODriveAxisState::IDLE);
        }

        void control_loop() {
            using Clock = std::chrono::steady_clock;
            auto next_time = Clock::now();
            while(running) {
                next_time += control_rate;
                /* Lock Guard Scope */
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    for(const canid_t motor_id : motor_ids) {
                        float torque_input = motor_commands.torque_feedforward[motor_id];
                        switch(ctrl_mode) {
                            case ODriveControlMode::POSITION:
                                odrv_socket->set_stiffness(motor_id, motor_commands.stiffness[motor_id]);
                                odrv_socket->set_damping(motor_id, motor_commands.damping[motor_id], motor_commands.velocity_integrator[motor_id]);
                                odrv_socket->position_command(
                                    motor_id,
                                    motor_commands.position_setpoint[motor_id],
                                    motor_commands.velocity_setpoint[motor_id],
                                    torque_input
                                );
                                break;
                            case ODriveControlMode::VELOCITY:
                                odrv_socket->set_damping(motor_id, motor_commands.damping[motor_id], motor_commands.velocity_integrator[motor_id]);
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
                auto now = Clock::now();
                if (now < next_time) {
                    std::this_thread::sleep_until(next_time);
                }
                else {
                    auto overrun = std::chrono::duration_cast<std::chrono::microseconds>(now - next_time);
                    std::cout << "Control loop overrun: " << overrun.count() << "us" << std::endl;
                    next_time = now;
                }
            }
        }

};