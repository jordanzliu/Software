package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

py_library(
    name = "simulator_env",
    srcs = ["simulator_env.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//proto:import_all_protos",
        "//proto/message_translation:py_tbots_protobuf",
        "//software/thunderscope:proto_unix_io",
        "//software/thunderscope/binary_context_managers:simulator",
        requirement("gymnasium"),
        requirement("numpy"),
        requirement("matplotlib"),
    ],
)

py_binary(
    name = "thunderzero_train",
    srcs = ["thunderzero_train.py"],
    data = ["//software:python_bindings"],
    deps = [
        ":simulator_env",
        requirement("gymnasium"),
        requirement("stable-baselines3"),
        requirement("pillow"),
        requirement("opencv-python"),
        requirement("numpy"),
    ],
)
