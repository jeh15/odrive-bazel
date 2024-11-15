#pragma once

#include <memory>

#include "odrive_socket.h"
#include "estop.h"

class MotorController : public Estop
{
private:
    std::shared_ptr<ODriveSocket> _odrv_socket;
    canid_t _motor_id;

    void estop(int sig) override {
        printf("Running ESTOP\n");
        _odrv_socket->setAxisState(_motor_id, ODriveAxisState::IDLE);
    }

public:
    MotorController(std::shared_ptr<ODriveSocket> odrv, int motor_id)
        : Estop(), _odrv_socket(odrv), _motor_id(motor_id) { }

    ~MotorController() { 
        _odrv_socket->setAxisState(_motor_id, ODriveAxisState::IDLE);
    }

    void set_axis_state(const ODriveAxisState axis_state) {
        _odrv_socket->setAxisState(_motor_id, axis_state);
    }

    void set_control_mode(const ODriveControlMode control_mode, const ODriveInputMode input_mode = ODriveInputMode::PASSTHROUGH) {
        _odrv_socket->setControlMode(_motor_id, control_mode, input_mode);
    }

    void set_torque(const float torque_setpoint) {
        _odrv_socket->setTorque(_motor_id, torque_setpoint);
    }

    float get_position() {
        return _odrv_socket->getPositionEstimate(_motor_id);
    }

    float get_velocity() {
        return _odrv_socket->getVelocityEstimate(_motor_id);
    }

    float get_torque_estimate() {
        return _odrv_socket->getTorqueEstimate(_motor_id);
    }

};