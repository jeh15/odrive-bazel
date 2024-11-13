#include <chrono>
#include <string>

#include "odrive_socket/odrive_socket.h"

const std::string CAN_IFC = "can0";
const int MOTOR_ID = 0;
const uint32_t CTRL_MODE = 1;

int main(void) {
    // Create ODriveSocket Object:
    ODriveSocket odrv(CAN_IFC);

    // Set Control Mode and AxisState:
    odrv.setControlMode(MOTOR_ID, CTRL_MODE);
    odrv.setAxisState(MOTOR_ID, 8);   

    while(true){
        float torque_setpoint = 0.1f;

        // Send Torque Command:
        odrv.setTorque(MOTOR_ID, torque_setpoint);

        // Print Position and Velocity:
        float position = odrv.getPositionEstimate(MOTOR_ID);
        float velocity = odrv.getVelocityEstimate(MOTOR_ID);
        printf("Position: %f, Velocity: %f\n", position, velocity);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}