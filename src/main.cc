#include <chrono>
#include <string>

#include "utils/odrive_socket.h"
#include "utils/motor_controller.h"

const std::string CAN_IFC = "can0";
const int MOTOR_ID = 0;
const uint32_t CTRL_MODE = 1;


int main(void) {
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

        // Sleep for 20ms:
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
