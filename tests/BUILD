load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    deps = [
        "//odrive-api/communication:odrive_socket",
        "//odrive-api/logger:logger",
        "@rules_cc//cc/runfiles:runfiles",
    ],
)

# py_binary(
#     name = "main",
#     srcs = ["main.py"],
#     deps = [
#         "//bindings:lowlevelapi_library",
#         "@pypi//absl_py:pkg",
#     ],
# )

# py_binary(
#     name = "read",
#     srcs = ["read.py"],
#     deps = [
#         "//bindings:lowlevelapi_library",
#         "@pypi//absl_py:pkg",
#     ],
# )
