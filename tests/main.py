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
        lowlevelapi.ODriveControlMode.POSITION,
    )
    motor_controller.initialize_control_thread()

    # Create Motor Command Struct:
    motor_command = lowlevelapi.MotorCommand()
    motor_command.position_setpoint = [1.0, 0.0]
    motor_command.velocity_setpoint = [0.0, 0.0]
    motor_command.torque_feedforward = [0.0, 0.0]
    motor_command.kp = [0.0, 0.0]
    motor_command.kd = [0.0, 0.0]
    motor_command.stiffness = [50.0, 0.0]
    motor_command.damping = [0.1, 0.1]

    # Send Motor Command:
    motor_controller.update_command(motor_command)

    # Sleep for 2 seconds:
    time.sleep(1)

    # Stop Control Thread:
    motor_controller.stop_control_thread()

    # Set Axis State to IDLE:
    motor_controller.set_axis_state(
        lowlevelapi.ODriveAxisState.IDLE,
    )


if __name__ == '__main__':
    app.run(main)
