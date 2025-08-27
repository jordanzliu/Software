package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

py_library(
    name = "reward_functions",
    srcs = ["reward_functions.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":utils",
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//proto:import_all_protos",
        "//proto/message_translation:py_tbots_protobuf",
        requirement("numpy"),
        requirement("numpy-quaternion"),
    ],
)

py_library(
    name = "simulator_env",
    srcs = ["simulator_env.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":render",
        ":utils",
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//proto:import_all_protos",
        "//proto/message_translation:py_tbots_protobuf",
        "//software/ml:reward_functions",
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
    name = "selfplay_simulator_env",
    srcs = ["selfplay_simulator_env.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":utils",
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//proto:import_all_protos",
        "//proto/message_translation:py_tbots_protobuf",
        "//software/ml:reward_functions",
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

py_library(
    name = "render",
    srcs = ["render.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":reward_functions",
        ":utils",
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
        requirement("imageio"),
        requirement("requests"),
        requirement("torch"),
    ],
)

py_binary(
    name = "thunderzero_train_with_ray",
    srcs = ["thunderzero_train_with_ray.py"],
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
        requirement("imageio"),
        requirement("requests"),
        requirement("ray"),
    ],
)

py_binary(
    name = "selfplay_train_with_ray",
    srcs = ["selfplay_train_with_ray.py"],
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
        requirement("imageio"),
        requirement("requests"),
        requirement("ray"),
    ],
)
