#include <chrono>
#include <string>

#include "utils/odrive_socket.h"
#include "utils/motor_controller.h"

const std::string CAN_IFC = "can0";
const int MOTOR_ID = 0;
const uint32_t CTRL_MODE = 1;


int main(void) {
    // Create Shared ODrive Socket and Motor Controller:
    MotorController odrv(
        std::make_shared<ODriveSocket>(ODriveSocket(CAN_IFC)),
        MOTOR_ID
    );

    // Set Control Mode and AxisState:
    odrv.set_control_mode(CTRL_MODE);
    odrv.set_axis_state(8);   

    while(true){
        float torque_setpoint = 0.1f;

        // Send Torque Command:
        odrv.set_torque(torque_setpoint);

        // Sleep for 20ms:
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
