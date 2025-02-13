from absl import app
import time

import lowlevelapi


def main(argv=None):
    CAN_IFC = "can0"
    motor_ids = [0, 1]
    odrive_socket = lowlevelapi.ODriveSocket(CAN_IFC)

    # Create motor controller driver:
    motor_controller = lowlevelapi.MotorController(odrive_socket, motor_ids)

    # Set Control Mode, Axis State, and initialize the control thread:
    motor_controller.set_axis_state(
        lowlevelapi.ODriveAxisState.CLOSED_LOOP_CONTROL,
    )
    motor_controller.set_control_mode(
        lowlevelapi.ODriveControlMode.VELOCITY,
    )

    # Read Motor States:
    start_time = time.time()
    while (time.time() - start_time < 10.0):
        motor_states = motor_controller.get_motor_states()
        print(f"Motor 1: \t \t Motor 2:")
        print(f"Position: {motor_states.position[0]} \t \t Position: {motor_states.position[1]}")
        print(f"Velocity: {motor_states.velocity[0]} \t \t Velocity: {motor_states.velocity[1]}")
        print(f"Torque Estimate: {motor_states.torque_estimate[0]} \t \t Torque Estimate: {motor_states.torque_estimate[1]}")
        print(f"Current Setpoint: {motor_states.current_setpoint[0]} \t \t Current Setpoint: {motor_states.current_setpoint[1]}")
        print(f"Current Measured: {motor_states.current_measured[0]} \t \t Current Measured: {motor_states.current_measured[1]}")
        time.sleep(0.1)


if __name__ == "__main__":
    app.run(main)
