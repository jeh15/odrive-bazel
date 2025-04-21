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

        ~ODriveDriver() {}

        // Public methods:
        void set_axis_state(const ODriveAxisState axis_state) {
            for(std::shared_ptr<ODriveSocketDriver>& odrv : odrvs)
                odrv->set_axis_state(axis_state);
        }

        void set_control_mode(const ODriveControlMode control_mode, const ODriveInputMode input_mode = ODriveInputMode::PASSTHROUGH) {
            for(std::shared_ptr<ODriveSocketDriver>& odrv : odrvs)
                odrv->set_control_mode(control_mode, input_mode);
        }

        std::vector<std::string> get_axis_state(void) {
            std::vector<std::string> axis_states;
            for(const std::shared_ptr<ODriveSocketDriver>& odrv : odrvs) {
                std::vector<std::string> axis_state = odrv->get_axis_state();
                axis_states.insert(axis_states.end(), axis_state.begin(), axis_state.end());
            }
            return axis_states;
        }

        void initialize_thread() {
            thread = std::thread(&ODriveDriver::control_loop, this);
        }

        void stop_thread() {
            running = false;
            thread.join();
        }

        void update_command(const std::vector<MotorCommand>& commands) {
            for(auto& [odrv, command] : std::views::zip(odrvs, commands)) {
                odrv->update_command(command);
            }
        }

        std::vector<MotorState> get_motor_states() {
            std::vector<MotorState> motor_states;
            for(const std::shared_ptr<ODriveSocketDriver>& odrv : odrvs) {
                MotorState motor_state = odrv->get_motor_state();
                motor_states.push_back(motor_state);
            }
            return motor_states;
        }

    private:
        // ODrive Variables:
        std::vector<std::shared_ptr<ODriveSocketDriver>> odrvs;
        float torque_constant_knee = 8.27F / 330.0F;
        float torque_constant_hip = 8.27f / 150.0f;
        // Control Loop Thread Variables:
        int control_rate_us;
        std::chrono::microseconds control_rate = std::chrono::microseconds(control_rate_us);
        std::atomic<bool> running{true};
        std::mutex mutex;   // Currently not used, but may be needed in the future
        std::thread thread;

        void estop(int sig) override {
            printf("Running ESTOP\n");
            for(std::shared_ptr<ODriveSocketDriver>& odrv : odrvs)
                odrv->set_axis_state(ODriveAxisState::IDLE);
        }

        void control_loop() {
            using Clock = std::chrono::steady_clock;
            auto next_time = Clock::now();
            while(running) {
                next_time += control_rate;

                /* ODrive Socket Driver handles its own locks */
                for(const std::shared_ptr<ODriveSocketDriver>& odrv : odrvs) {
                    odrv->send_command();
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