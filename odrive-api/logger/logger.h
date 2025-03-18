#pragma once

#include <filesystem>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

#include "odrive-api/communication/odrive_socket.h"
#include "odrive-api/containers.h"


using namespace odrive::containers;


class Logger {
    public:
        Logger(
            std::shared_ptr<ODriveSocket> odrv_socket,
            std::vector<canid_t> motor_ids,
            const std::filesystem::path filepath,
            const int log_rate_us = 10000
        ) :
            odrv_socket(odrv_socket),
            motor_ids(motor_ids),
            filepath(filepath),
            log_rate_us(log_rate_us) {}
        ~Logger() {}

        void initialize() {
            // Check filepath:
            if (!std::filesystem::exists(filepath.parent_path())) {
                assert((void("Logger: Filepath not found."), false));
            }
            // Might need to wrap in pointer:
            logger = spdlog::basic_logger_mt("odrive_logger", filepath);
            initialized = true;
        }

        void initialize_thread() {
            if (!initialized) {
                assert((void("Logger: Logger not initialized."), false));
            }
            thread = std::thread(&Logger::log_loop, this);
            thread_initialized = true;
        }

        void stop_thread() {
            if (!thread_initialized) {
                assert((void("Logger: Logger thread not initialized."), false));
            }
            running = false;
            thread.join();
        }

    private:
        // Odrive Socket:
        std::shared_ptr<ODriveSocket> odrv_socket;
        std::vector<canid_t> motor_ids;
        // Variables:
        std::filesystem::path filepath;
        int log_rate_us;
        std::chrono::microseconds log_rate = std::chrono::microseconds(log_rate_us);
        std::shared_ptr<spdlog::logger> logger;
        bool initialized = false;
        bool thread_initialized = false;
        // Thread variables:
        std::atomic<bool> running{true};
        std::mutex mutex;
        std::thread thread;

        void log_loop() {
            using Clock = std::chrono::steady_clock;
            auto next_time = Clock::now();
            while (running) {
                next_time += log_rate;
                /* Lock Guard Scope */
                {
                    std::lock_guard<std::mutex> lock(mutex);

                    LogData data = { 0 };
                    for (const canid_t motor_id : motor_ids) {
                        data.position[motor_id] = odrv_socket->getPositionEstimate(motor_id);
                        data.velocity[motor_id] = odrv_socket->getVelocityEstimate(motor_id);
                        data.torque_estimate[motor_id] = odrv_socket->getTorqueEstimate(motor_id);
                        data.current_setpoint[motor_id] = odrv_socket->getIqSetpoint(motor_id);
                        data.current_measured[motor_id] = odrv_socket->getIqMeasured(motor_id);
                        data.fet_temperature[motor_id] = odrv_socket->getFETTemperature(motor_id);
                    }

                    // Log data: (TODO: jeh15 generate string format)
                    logger->info(
                        "Motor States: Position: {0}, {1}, Velocity: {2}, {3}, Torque Estimate: {4}, {5}, Current Setpoint: {6}, {7}, Current Measured: {8}, {9}, FET Temperature: {10}, {11}",
                        data.position[0], data.position[1],
                        data.velocity[0], data.velocity[1],
                        data.torque_estimate[0], data.torque_estimate[1],
                        data.current_setpoint[0], data.current_setpoint[1],
                        data.current_measured[0], data.current_measured[1],
                        data.fet_temperature[0], data.fet_temperature[1]
                    );
                }
                // Log Rate:
                auto now = Clock::now();
                if (now < next_time) {
                    std::this_thread::sleep_until(next_time);
                }
                else {
                    auto overrun = std::chrono::duration_cast<std::chrono::microseconds>(now - next_time);
                    std::cout << "Log loop overrun: " << overrun.count() << "us" << std::endl;
                    next_time = now;
                }
            }
        }
};
