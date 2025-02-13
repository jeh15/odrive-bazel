from absl import app

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
    motor_states = motor_controller.get_motor_states()
    print(motor_states)


if __name__ == "__main__":
    app.run(main)
