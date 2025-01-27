cc_library(
    name = "low_level_controller",
    srcs = ["low_level_controller.h"],
    deps = [
        ":low_level_types",
        "//src/utils:estop",
        "//src/communication:odrive_socket",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "low_level_types",
    srcs = ["low_level_types.h"],
    visibility = ["//visibility:public"],
)
