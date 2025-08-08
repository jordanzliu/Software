package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

py_test(
    name = "utils_test",
    srcs = ["utils_test.py"],
    deps = [
        "//software/ml:utils",
        requirement("numpy"),
        requirement("numpy-quaternion"),
    ],
)

py_test(
    name = "reward_functions_test",
    srcs = ["reward_functions_test.py"],
    deps = [
        "//proto:import_all_protos",
        "//software/ml:reward_functions",
        requirement("numpy"),
    ],
)
