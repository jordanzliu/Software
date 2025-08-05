package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

py_library(
    name = "simulator_env",
    srcs = ["simulator_env.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":utils",
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//proto:import_all_protos",
        "//proto/message_translation:py_tbots_protobuf",
        "//software/thunderscope:proto_unix_io",
        "//software/thunderscope/binary_context_managers:simulator",
        requirement("gymnasium"),
        requirement("numpy"),
        requirement("matplotlib"),
        requirement("tensorboard"),
        requirement("numpy-quaternion"),
    ],
)

py_library(
    name = "utils",
    srcs = ["utils.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//proto:import_all_protos",
        "//proto/message_translation:py_tbots_protobuf",
        requirement("numpy"),
        requirement("numpy-quaternion"),
    ],
)

py_binary(
    name = "thunderzero_train",
    srcs = ["thunderzero_train.py"],
    data = ["//software:python_bindings"],
    deps = [
        ":simulator_env",
        ":utils",
        requirement("gymnasium"),
        requirement("stable-baselines3"),
        requirement("pillow"),
        requirement("opencv-python"),
        requirement("numpy"),
        requirement("numpy-quaternion"),
    ],
)
