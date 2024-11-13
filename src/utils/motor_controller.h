#pragma once

#include <memory>

#include "odrive_socket.h"


class MotorController {
    private:
        std::shared_ptr<ODriveSocket> odrv_ptr;
    
    public:
        int MOTOR_ID = 0;

        MotorController(
            std::shared_ptr<ODriveSocket> odrv, int motor_id
        ) {
            odrv_ptr = *odrv;
            MOTOR_ID = motor_id;
        }

        ~MotorController() {
            odrv_ptr->setAxisState(MOTOR_ID, 1);
        }

        void set_axis_state(const uint8_t axis_state) {
            odrv_ptr->setAxisState(MOTOR_ID, axis_state);
        }

        void set_control_mode(const uint32_t control_mode, const uint32_t input_mode = 0x1) {
            odrv_ptr->setControlMode(MOTOR_ID, control_mode, input_mode);
        }

        void set_torque(const float torque_setpoint) {
            odrv_ptr->setTorque(MOTOR_ID, torque_setpoint);
        }
};
