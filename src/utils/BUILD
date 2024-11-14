copts = select({
    "//conditions:default": ["-fPIC"],
})

linkopts = select({
    "//conditions:default": ["-pthread"],
})

cc_library(
    name = "odrive_socket",
    srcs = ["odrive_socket.h"],
    copts = copts,
    linkopts = linkopts,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "estop",
    srcs = ["estop.h"],
    copts = copts,
    linkopts = linkopts,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "motor_controller",
    srcs = ["motor_controller.h"],
    deps = [
        ":odrive_socket",
        ":estop",
    ],
    copts = copts,
    linkopts = linkopts,
    visibility = ["//visibility:public"],
)
