#include <chrono>
#include <string>

#include "odrive_socket/odrive_socket.h"

const std::string CAN_IFC = "can0";
const int MOTOR_ID = 0;

int main(void) {
    ODriveSocket odrv(CAN_IFC);

    while(true) {
        float position = odrv.getPositionEstimate(MOTOR_ID);
        float velocity = odrv.getVelocityEstimate(MOTOR_ID);
        float torque = odrv.getTorqueEstimate(MOTOR_ID);

        printf("Position: %f, Velocity: %f, Torque: %f\n", position, velocity, torque);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}