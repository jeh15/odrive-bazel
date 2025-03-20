#include <filesystem>
#include <string>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"

#include "odrive-api/communication/odrive_socket.h"
#include "odrive-api/logger/logger.h"

using rules_cc::cc::runfiles::Runfiles;


const std::string CAN_IFC = "can0";
constexpr uint32_t CTRL_MODE = 1;

int main(int argc, char** argv) {
    std::string error;
    std::unique_ptr<Runfiles> runfiles(
        Runfiles::Create(argv[0], BAZEL_CURRENT_REPOSITORY, &error)
    );

    // Log Filepath:
    std::filesystem::path log_path = runfiles->Rlocation("odrive-bazel/logs/log.log");

    // ODrive Socket:
    auto odrive_socket = std::make_shared<ODriveSocket>(CAN_IFC);
    const std::vector<canid_t> motor_ids = {0, 1};

    int log_rate_us = 1000;
    const size_t queue_size = 16384; //32768;
    const size_t num_async_threads = 2;
    auto logger = Logger(odrive_socket, motor_ids, log_path, log_rate_us, queue_size, num_async_threads);
    logger.initialize();
    logger.initialize_thread();

    while(true){
        continue;
    }

    logger.stop_thread();
    logger.shutdown();

    return 0;
}
