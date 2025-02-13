load("@rules_python//python:defs.bzl", "py_binary")

# cc_binary(
#     name = "main",
#     srcs = ["main.cc"],
#     deps = [
#         "//src/communication:odrive_socket",
#         "//src/controller:low_level_controller",
#         "//src/controller:low_level_types",
#         "@spdlog//:spdlog",
#     ],
# )

py_binary(
    name = "main",
    srcs = ["main.py"],
    deps = [
        "//bindings:lowlevelapi_library",
        "@pypi//absl_py:pkg",
    ],
)

py_binary(
    name = "read",
    srcs = ["read.py"],
    deps = [
        "//bindings:lowlevelapi_library",
        "@pypi//absl_py:pkg",
    ],
)
