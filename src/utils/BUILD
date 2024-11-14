cc_library(
    name = "odrive_socket",
    srcs = ["odrive_socket.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "estop",
    srcs = ["estop.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "motor_controller",
    srcs = ["motor_controller.h"],
    deps = [
        ":odrive_socket",
        ":estop",
    ],
    visibility = ["//visibility:public"],
)
