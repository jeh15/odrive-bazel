from absl import app
from pathlib import Path

from python.runfiles import Runfiles

import odrive_api


def main(argv=None):
    CAN_IFC = "can0"
    motor_ids = [0, 1]
    odrive_socket = odrive_api.ODriveSocket(CAN_IFC)

    r = Runfiles.Create()
    log_path = Path(
        r.Rlocation(
            path="odrive-bazel/logs/log.log",
        ),
    )

    logger = odrive_api.Logger(odrive_socket, motor_ids, log_path)
    logger.initialize()
    logger.initialize_thread()

    print(f"Logging to {log_path}")

    while(True):
        pass

    logger.stop_thread()


if __name__ == "__main__":
    app.run(main)
